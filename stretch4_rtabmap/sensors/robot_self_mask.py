"""Paint the robot's own body out of the head stereo images before RTAB-Map sees them."""

import os
import struct
import time
import xml.etree.ElementTree as ET

import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

NEAR_PLANE = 0.02  # metres; box parts closer to the camera than this are clipped away

FILL_MODES = ('constant', 'blur', 'inpaint')

# RTAB-Map's defaults for Vis/FeatureType 8 (GFTT/ORB): Vis/MaxFeatures, GFTT/QualityLevel,
# GFTT/MinDistance, GFTT/BlockSize.
GFTT = {'maxCorners': 1000, 'qualityLevel': 0.001, 'minDistance': 7, 'blockSize': 3}

# Pairs of AABB corner indices (bit i of the index selects min/max on axis i) sharing an edge.
BOX_EDGES = [(a, a | (1 << axis)) for a in range(8) for axis in range(3) if not a & (1 << axis)]


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def load_stl_vertices(path):
    """Return an (N, 3) array of the vertices of a binary or ASCII STL file."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        count = struct.unpack_from('<I', data, 80)[0]
        if len(data) == 84 + 50 * count:
            tris = np.frombuffer(data, dtype=np.uint8, offset=84).reshape(count, 50)
            return tris[:, 12:48].copy().view('<f4').reshape(-1, 3).astype(np.float64)
    verts = [line.split()[1:4] for line in data.decode('ascii', 'ignore').splitlines()
             if line.strip().startswith('vertex')]
    return np.array(verts, dtype=np.float64).reshape(-1, 3)


def resolve_mesh_path(uri):
    if uri.startswith('file://'):
        return uri[len('file://'):]
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package, _, rest = uri[len('package://'):].partition('/')
        return os.path.join(get_package_share_directory(package), rest)
    return uri


def geometry_bounds(geometry):
    """Return (min, max) of a URDF <geometry> element in its own frame, or None."""
    shape = geometry[0]
    if shape.tag == 'mesh':
        verts = load_stl_vertices(resolve_mesh_path(shape.get('filename')))
        if len(verts) == 0:
            return None
        scale = np.array([float(s) for s in shape.get('scale', '1 1 1').split()])
        verts = verts * scale
        return verts.min(axis=0), verts.max(axis=0)
    if shape.tag == 'box':
        half = np.array([float(s) for s in shape.get('size').split()]) / 2.0
        return -half, half
    if shape.tag == 'cylinder':
        r, half_len = float(shape.get('radius')), float(shape.get('length')) / 2.0
        return np.array([-r, -r, -half_len]), np.array([r, r, half_len])
    if shape.tag == 'sphere':
        r = float(shape.get('radius'))
        return np.full(3, -r), np.full(3, r)
    return None


def box_corners(lo, hi):
    """Return the 8 corners of an AABB, ordered so bit i of the index picks hi on axis i."""
    return np.array([[hi[i] if c & (1 << i) else lo[i] for i in range(3)] for c in range(8)])


def link_boxes_from_urdf(urdf_xml, exclude=(), padding=0.0):
    """Return {link: (lo, hi)}, the padded AABB of each link's collision geometry in its frame."""
    boxes = {}
    for link in ET.fromstring(urdf_xml).findall('link'):
        name = link.get('name')
        if name in exclude:
            continue
        points = []
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) == 0:
                continue
            bounds = geometry_bounds(geometry)
            if bounds is None:
                continue
            origin = collision.find('origin')
            xyz = np.array([float(v) for v in (origin.get('xyz', '0 0 0') if origin is not None
                                               else '0 0 0').split()])
            rpy = [float(v) for v in (origin.get('rpy', '0 0 0') if origin is not None
                                      else '0 0 0').split()]
            points.append(box_corners(*bounds) @ rpy_to_matrix(*rpy).T + xyz)
        if points:
            points = np.vstack(points)
            boxes[name] = (points.min(axis=0) - padding, points.max(axis=0) + padding)
    return boxes


def clip_box_to_near_plane(corners_cam, near=NEAR_PLANE):
    """Return the vertices of a box (8 corners, camera frame) clipped to z >= near."""
    z = corners_cam[:, 2]
    points = [corners_cam[z >= near]]
    for a, b in BOX_EDGES:
        if (z[a] >= near) != (z[b] >= near):
            t = (near - z[a]) / (z[b] - z[a])
            points.append((corners_cam[a] + t * (corners_cam[b] - corners_cam[a]))[None])
    return np.vstack(points)


def project_hull(points_cam, projection, width, height):
    """Return the convex polygon (int32) of the projected points inside the image, or None."""
    if len(points_cam) < 3:
        return None
    homogeneous = np.hstack([points_cam, np.ones((len(points_cam), 1))]) @ projection.T
    pixels = (homogeneous[:, :2] / homogeneous[:, 2:3]).astype(np.float32)
    hull = cv2.convexHull(pixels)
    image = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)
    area, clipped = cv2.intersectConvexConvex(hull, image)
    if area <= 0.0 or clipped is None:
        return None
    return np.round(clipped).astype(np.int32)


def ellipse_kernel(radius):
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))


def build_mask(polygons, width, height, dilate_px=0, static=None):
    """Return a uint8 mask (255 = robot) of all polygons, OR static, dilated by dilate_px."""
    mask = np.zeros((height, width), dtype=np.uint8)
    for polygon in polygons:
        cv2.fillConvexPoly(mask, polygon, 255)
    if static is not None:
        mask[static] = 255
    if dilate_px > 0 and mask.any():
        mask = cv2.dilate(mask, ellipse_kernel(dilate_px))
    return mask


def fill_image(image, mask, mode, fill_value=0, blur_ksize=51, inpaint_scale=4):
    """Return the image with what the mask covers replaced according to mode."""
    if mode == 'constant':
        return np.full_like(image, fill_value)
    if mode == 'blur':
        # Box blur is O(1) per pixel whatever the size; inside the mask it is just the local mean.
        return cv2.blur(image, (blur_ksize, blur_ksize))
    if mode == 'inpaint':
        # Inpainting grows with mask area; at 1/inpaint_scale the result is smooth anyway.
        h, w = mask.shape
        small_size = (max(1, w // inpaint_scale), max(1, h // inpaint_scale))
        small = cv2.resize(image, small_size, interpolation=cv2.INTER_AREA)
        small_mask = cv2.dilate(
            cv2.resize(mask, small_size, interpolation=cv2.INTER_NEAREST), ellipse_kernel(1))
        filled = cv2.inpaint(small, small_mask, 3, cv2.INPAINT_TELEA)
        return cv2.resize(filled, (w, h), interpolation=cv2.INTER_LINEAR)
    raise ValueError(f'fill_mode must be one of {FILL_MODES}, not "{mode}".')


def feather_alpha(mask, feather_px):
    """
    Return the blend weight of the fill per pixel (float32 0..1), 1 inside the mask.

    None means no feathering: a hard edge. Depends only on the mask, so it is cached with it.
    """
    if feather_px <= 0:
        return None
    k = 2 * feather_px + 1
    alpha = cv2.GaussianBlur(mask.astype(np.float32) / 255.0, (k, k), 0)
    alpha[mask > 0] = 1.0
    return alpha


def blend(image, filled, mask, alpha):
    """Return image with `filled` cross-faded in by alpha (feather_alpha), or hard by mask."""
    if alpha is None:
        out = image.copy()
        inside = mask > 0
        out[inside] = filled[inside]
        return out
    if image.ndim == 3:
        alpha = alpha[:, :, None]
    out = image.astype(np.float32) * (1.0 - alpha) + filled.astype(np.float32) * alpha
    return np.clip(np.round(out), 0, np.iinfo(image.dtype).max).astype(image.dtype)


def apply_mask(image, mask, mode, fill_value=0, blur_ksize=51, feather_px=0):
    """
    Return the image with the mask filled.

    The fill is cross-faded over feather_px around the mask edge; pixels inside the mask are
    always fully replaced.
    """
    if not mask.any():
        return image
    filled = fill_image(image, mask, mode, fill_value, blur_ksize)
    return blend(image, filled, mask, feather_alpha(mask, feather_px))


def moving_joints_from_urdf(urdf_xml):
    """
    Return the names of the joints that change the mask: prismatic and revolute ones.

    Continuous joints (the wheels) turn all the time while driving but leave a wheel looking the
    same, and the mask only covers each link's box, so they are left out.
    """
    return {j.get('name') for j in ET.fromstring(urdf_xml).findall('joint')
            if j.get('type') in ('prismatic', 'revolute')}


def joints_moved(reference, current, names, tolerance):
    """
    Return True if any joint in names moved more than tolerance since reference.

    reference None (no mask yet) counts as moved. Joints missing from current are skipped.
    """
    if reference is None:
        return True
    for name in names:
        if name in current and (name not in reference
                                or abs(current[name] - reference[name]) > tolerance):
            return True
    return False


def to_gray(image, encoding):
    if image.ndim == 2:
        return image
    code = cv2.COLOR_RGB2GRAY if encoding.startswith('rgb') else cv2.COLOR_BGR2GRAY
    return cv2.cvtColor(image, code)


def classify_features(gray, mask, edge_band):
    """Return (near, far) GFTT corners as (N, 2) arrays: on/within edge_band px of mask, or not."""
    corners = cv2.goodFeaturesToTrack(gray, **GFTT)
    if corners is None:
        empty = np.zeros((0, 2), dtype=np.float32)
        return empty, empty
    corners = corners.reshape(-1, 2)
    near_mask = cv2.dilate(mask, ellipse_kernel(edge_band)) if edge_band > 0 else mask
    h, w = mask.shape
    u = np.clip(np.round(corners[:, 0]).astype(int), 0, w - 1)
    v = np.clip(np.round(corners[:, 1]).astype(int), 0, h - 1)
    near = near_mask[v, u] > 0
    return corners[near], corners[~near]


def robot_features(gray_unmasked, mask):
    """Return the GFTT corners (N, 2) the UNMASKED image has on the mask: what masking removes."""
    corners = cv2.goodFeaturesToTrack(gray_unmasked, **GFTT)
    if corners is None:
        return np.zeros((0, 2), dtype=np.float32)
    corners = corners.reshape(-1, 2)
    h, w = mask.shape
    u = np.clip(np.round(corners[:, 0]).astype(int), 0, w - 1)
    v = np.clip(np.round(corners[:, 1]).astype(int), 0, h - 1)
    return corners[mask[v, u] > 0]


def draw_debug(gray, mask, near, far, label, removed=None):
    """
    Return a BGR image of the masked image, the mask outline and the corners.

    green circle = corner elsewhere, red circle = corner still on or next to the mask after
    masking, magenta cross = corner the unmasked image had on the robot (removed by the mask).
    """
    debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(debug, contours, -1, (0, 255, 255), 1)
    removed = np.zeros((0, 2)) if removed is None else removed
    for u, v in removed:
        cv2.drawMarker(debug, (int(round(u)), int(round(v))), (255, 0, 255),
                       cv2.MARKER_TILTED_CROSS, 7, 1)
    for points, color in ((far, (0, 200, 0)), (near, (0, 0, 255))):
        for u, v in points:
            cv2.circle(debug, (int(round(u)), int(round(v))), 3, color, 1)
    lines = [label, f'on robot (x): {len(removed)}  near mask: {len(near)}  elsewhere: {len(far)}']
    for row, text in enumerate(lines):
        for color, thickness in (((0, 0, 0), 3), ((255, 255, 255), 1)):
            cv2.putText(debug, text, (8, 22 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color,
                        thickness)
    return debug


def transform_matrix(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    matrix = np.eye(4)
    matrix[:3, :3] = quat_to_matrix(q.x, q.y, q.z, q.w)
    matrix[:3, 3] = (t.x, t.y, t.z)
    return matrix


class RobotSelfMask(Node):

    def __init__(self):
        super().__init__('robot_self_mask')
        # The cameras are inside head_link's box, so it cannot be projected.
        self.declare_parameter('exclude_links', ['head_link'])
        self.declare_parameter('padding', 0.03)
        self.declare_parameter('fill_mode', 'blur')
        self.declare_parameter('apply_mask', True)
        self.declare_parameter('debug', False)
        self.declare_parameter('fill_value', 0)
        self.declare_parameter('blur_ksize', 51)
        self.declare_parameter('dilate_px', 10)
        self.declare_parameter('feather_px', 15)
        self.declare_parameter('edge_band', 6)
        self.declare_parameter('tf_timeout', 0.2)
        # The newest link transforms are used instead of the image stamp's when TF lags by at
        # most this much: the arm and head hardly move in that time while navigating.
        self.declare_parameter('tf_tolerance', 0.5)
        self.declare_parameter('stall_error_after', 3.0)   # s without output while images arrive
        self.declare_parameter('joint_tolerance', 0.005)   # m or rad moved before a mask rebuild
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_static_mask', '')
        self.declare_parameter('right_static_mask', '')

        self.boxes, self.corners = None, None
        self.static_masks = {}
        for side in ('left', 'right'):
            path = os.path.expanduser(self.get_parameter(f'{side}_static_mask').value)
            if path:
                mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if mask is None:
                    raise RuntimeError(f'{side}_static_mask: cannot read "{path}".')
                self.static_masks[side] = mask > 0

        self.urdf_xml = None
        self.last_output = time.monotonic()
        self.last_input = None
        self.last_drop = ''
        self.tf_drops = 0               # frames dropped in a row for missing TF
        self.joints = {}                # latest position of every joint on /joint_states
        self.moving_joints = set()      # the ones that change the mask, from the URDF
        self.cache = None               # {side: (mask, alpha)} for cache_joints
        self.cache_joints = None        # joint positions the cached mask was built for
        self.cache_shape = None
        self.rebuilds = 0
        self.load_settings()
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.add_post_set_parameters_callback(self.on_parameters_set)
        self.warned_inside = set()

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        # Own thread: the lookups below block waiting for TF inside the image callback.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.info = {'left': None, 'right': None}
        pub_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.image_pubs, self.info_pubs = {}, {}
        for side in ('left', 'right'):
            self.create_subscription(
                CameraInfo, f'{side}/camera_info_in',
                lambda msg, s=side: self.info.__setitem__(s, msg), qos_profile_sensor_data)
            self.image_pubs[side] = self.create_publisher(Image, f'{side}/image_rect_out', pub_qos)
            self.info_pubs[side] = self.create_publisher(
                CameraInfo, f'{side}/camera_info_out', pub_qos)
        self.debug_pub = (self.create_publisher(Image, '~/debug/stereo', 2)
                          if self.get_parameter('debug').value else None)

        self.create_subscription(
            JointState, self.get_parameter('joint_states_topic').value, self.on_joint_states,
            QoSProfile(depth=10))
        self.create_subscription(
            String, '/robot_description', self.on_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        left = message_filters.Subscriber(
            self, Image, 'left/image_rect_in', qos_profile=qos_profile_sensor_data)
        right = message_filters.Subscriber(
            self, Image, 'right/image_rect_in', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([left, right], 5, 0.02)
        self.sync.registerCallback(self.on_images)
        self.create_timer(1.0, self.check_stall)

    def check_stall(self):
        """Shout when images come in and nothing goes out: RTAB-Map is then getting nothing."""
        now = time.monotonic()
        if self.last_input is None or now - self.last_input > 1.0:
            return
        stalled = now - self.last_output
        if stalled > self.stall_error_after:
            self.get_logger().error(
                f'No masked image published for {stalled:.0f} s while images arrive: RTAB-Map '
                f'gets no camera data. Last reason: {self.last_drop or "unknown"}',
                throttle_duration_sec=10.0)

    def load_settings(self):
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        self.fill_mode = value('fill_mode')
        self.apply_mask = bool(value('apply_mask'))
        self.fill_value = int(value('fill_value'))
        self.blur_ksize = int(value('blur_ksize'))
        self.dilate_px = int(value('dilate_px'))
        self.feather_px = int(value('feather_px'))
        self.edge_band = int(value('edge_band'))
        self.tf_timeout = Duration(seconds=value('tf_timeout'))
        self.tf_tolerance = value('tf_tolerance')
        self.joint_tolerance = value('joint_tolerance')
        self.cache = None               # any setting may change the mask
        self.stall_error_after = value('stall_error_after')

    def validate_parameters(self, params):
        for p in params:
            if p.name == 'fill_mode' and p.value not in FILL_MODES:
                return SetParametersResult(
                    successful=False, reason=f'fill_mode must be one of {FILL_MODES}')
            if p.name == 'blur_ksize' and (p.value < 1 or p.value % 2 == 0):
                return SetParametersResult(successful=False, reason='blur_ksize must be odd')
            if p.name in ('dilate_px', 'feather_px', 'edge_band', 'padding') and p.value < 0:
                return SetParametersResult(successful=False, reason=f'{p.name} must be >= 0')
        return SetParametersResult(successful=True)

    def on_parameters_set(self, params):
        self.load_settings()
        if self.urdf_xml is not None and {'padding', 'exclude_links'} & {p.name for p in params}:
            self.build_boxes()
        self.get_logger().info(
            'Set ' + ', '.join(f'{p.name}={p.value}' for p in params))

    def build_boxes(self):
        exclude = list(self.get_parameter('exclude_links').value)
        boxes = link_boxes_from_urdf(
            self.urdf_xml, exclude=exclude, padding=self.get_parameter('padding').value)
        self.corners = {link: box_corners(lo, hi) for link, (lo, hi) in boxes.items()}
        self.boxes = boxes
        self.moving_joints = moving_joints_from_urdf(self.urdf_xml)
        self.cache = None
        self.warned_inside.clear()
        self.get_logger().info(
            f'Masking {len(boxes)} links (excluded: {", ".join(exclude) or "none"}).')

    def on_joint_states(self, msg):
        # Merge: a message may carry only some of the joints.
        self.joints.update(zip(msg.name, msg.position))

    def on_robot_description(self, msg):
        self.urdf_xml = msg.data
        self.build_boxes()

    def lookup(self, camera_frame, link, stamp, timeout):
        """Transform at the image stamp, else the newest one if it is within tf_tolerance."""
        try:
            return self.tf_buffer.lookup_transform(camera_frame, link, stamp, timeout)
        except TransformException as exact_error:
            try:
                latest = self.tf_buffer.lookup_transform(camera_frame, link, Time())
            except TransformException:
                raise exact_error
            lag = (stamp - Time.from_msg(latest.header.stamp)).nanoseconds * 1e-9
            if abs(lag) > self.tf_tolerance:
                raise TransformException(
                    f'{exact_error}; the newest {link} transform is {lag:.2f} s older than the '
                    f'image (tf_tolerance {self.tf_tolerance} s): the TF listener is lagging')
            return latest

    def link_corners_in_camera(self, camera_frame, stamp):
        """Return {link: 8x3 corners in camera_frame}, or None if any transform is missing."""
        result = {}
        timeout = self.tf_timeout       # wait once, for the first link; the rest are there too
        for link, corners in self.corners.items():
            try:
                tf = self.lookup(camera_frame, link, stamp, timeout)
            except TransformException as e:
                self.last_drop = f'no TF {link} -> {camera_frame}: {e}'
                self.tf_drops += 1
                self.get_logger().warn(
                    f'Dropping frames for missing TF ({self.tf_drops} so far), {self.last_drop}',
                    throttle_duration_sec=5.0)
                return None
            timeout = Duration()
            matrix = transform_matrix(tf)
            lo, hi = self.boxes[link]
            camera_in_link = np.linalg.inv(matrix)[:3, 3]
            if np.all(camera_in_link >= lo) and np.all(camera_in_link <= hi):
                if link not in self.warned_inside:
                    self.warned_inside.add(link)
                    self.get_logger().warn(
                        f'Camera is inside the box of {link}; skipping it. Add it to '
                        'exclude_links.')
                continue
            result[link] = corners @ matrix[:3, :3].T + matrix[:3, 3]
        if self.tf_drops:
            self.get_logger().info(f'TF back after {self.tf_drops} dropped frames.')
            self.tf_drops = 0
        return result

    def rebuild_mask(self, left_msg, right_msg, stamp, joints):
        """Build both images' masks from TF and cache them; False (frame dropped) without TF."""
        # Both P matrices take points in the left rectified frame.
        camera_frame = self.info['left'].header.frame_id or left_msg.header.frame_id
        corners = self.link_corners_in_camera(camera_frame, stamp)
        if corners is None:
            return False
        clipped = [clip_box_to_near_plane(c) for c in corners.values()]
        cache = {}
        for side, msg in (('left', left_msg), ('right', right_msg)):
            projection = np.array(self.info[side].p, dtype=np.float64).reshape(3, 4)
            polygons = [project_hull(points, projection, msg.width, msg.height)
                        for points in clipped]
            static = self.static_masks.get(side)
            if static is not None and static.shape != (msg.height, msg.width):
                self.get_logger().error(
                    f'{side}_static_mask is {static.shape[1]}x{static.shape[0]}, image is '
                    f'{msg.width}x{msg.height}; not applied.', throttle_duration_sec=10.0)
                static = None
            mask = build_mask([p for p in polygons if p is not None], msg.width, msg.height,
                              self.dilate_px, static)
            cache[side] = (mask, feather_alpha(mask, self.feather_px))
        self.cache, self.cache_joints = cache, joints
        self.cache_shape = (left_msg.width, left_msg.height, right_msg.width, right_msg.height)
        self.rebuilds += 1
        return True

    def on_images(self, left_msg, right_msg):
        if self.info['left'] is None or self.info['right'] is None or self.boxes is None:
            self.get_logger().warn(
                'Waiting for camera_info and /robot_description.', throttle_duration_sec=5.0)
            return
        self.last_input = time.monotonic()
        stamp = Time.from_msg(left_msg.header.stamp)

        shape = (left_msg.width, left_msg.height, right_msg.width, right_msg.height)
        joints = dict(self.joints)
        # Without /joint_states nothing tells a still arm from a moving one: rebuild every time.
        if self.cache is None or not joints or shape != self.cache_shape or \
                joints_moved(self.cache_joints, joints, self.moving_joints, self.joint_tolerance):
            if not self.rebuild_mask(left_msg, right_msg, stamp, joints):
                return

        debug_images = []
        for side, msg in (('left', left_msg), ('right', right_msg)):
            mask, alpha = self.cache[side]
            info = self.info[side]
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            unmasked = image
            started = time.perf_counter() if self.debug_pub is not None else 0.0
            if self.apply_mask and mask.any():
                filled = fill_image(image, mask, self.fill_mode, self.fill_value,
                                    self.blur_ksize)
                image = blend(image, filled, mask, alpha)
            out = self.bridge.cv2_to_imgmsg(image, encoding=msg.encoding)
            out.header = msg.header
            info_out = CameraInfo()
            info_out.header = msg.header
            for field in ('height', 'width', 'distortion_model', 'd', 'k', 'r', 'p',
                          'binning_x', 'binning_y', 'roi'):
                setattr(info_out, field, getattr(info, field))
            self.image_pubs[side].publish(out)
            self.info_pubs[side].publish(info_out)
            if self.debug_pub is not None:
                elapsed_ms = 1000.0 * (time.perf_counter() - started)
                gray = to_gray(image, msg.encoding)
                near, far = classify_features(gray, mask, self.edge_band)
                removed = robot_features(to_gray(unmasked, msg.encoding), mask)
                mode = self.fill_mode if self.apply_mask else 'NOT MASKED'
                debug_images.append(draw_debug(
                    gray, mask, near, far,
                    f'{side} {mode} {elapsed_ms:.0f}ms rebuilds {self.rebuilds}', removed))
        if len(debug_images) == 2 and debug_images[0].shape[0] == debug_images[1].shape[0]:
            debug_msg = self.bridge.cv2_to_imgmsg(np.hstack(debug_images), encoding='bgr8')
            debug_msg.header = left_msg.header
            self.debug_pub.publish(debug_msg)
        self.last_output = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = RobotSelfMask()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
