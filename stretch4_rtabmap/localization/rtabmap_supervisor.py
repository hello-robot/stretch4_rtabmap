"""
Track whether the robot is localized on the RTAB-Map map, and switch RTAB-Map's mode.

Set a 2D Pose Estimate, send goals as usual, and RTAB-Map confirms the pose from
the closures it finds on the way. The states and mode rules are in localization_policy.py:

    NEED_POSE -> UNVERIFIED -> LOCALIZED -> JOINING -> MAPPING      (LOST: back to localization)

Outputs:

* /localization_status (std_msgs/String, latched): "STATUS | detail | hint".
* /localization_status_overlay: the same, pinned to the top-left of RViz.
* In NO_DATA and NEED_POSE, every active nav2 goal in guarded_actions is cancelled.

Mapping is held while it is dark: the mean brightness of image_topic decides (LightCheck), so
images taken in the dark are never written to the map.

"""

import math
import time

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
from rcl_interfaces.msg import Parameter, ParameterEvent, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rtabmap_msgs.msg import Info, MapGraph
from rviz_2d_overlay_msgs.msg import OverlayText
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Empty
from stretch4_rtabmap.localization.localization_policy import (
    BLOCKED, carry_pose, HINTS, LightCheck, LocalizationPolicy, Status)
from stretch4_rtabmap.localization.rtabmap_loop_monitor import (
    GATE_PARAM, is_true, MODE_PARAM, node_in_graph)
from stretch4_rtabmap.localization.rtabmap_params import mode_params, same
from stretch4_rtabmap.localization.rtabmap_state import (
    MAX_ANG_ERROR_RATIO_KEY, MAX_ERROR_RATIO_KEY, REJECTED_KEY, SessionTracker, State)
import tf2_ros

COLOURS = {   # RGB of the RViz text
    Status.NO_DATA: (0.6, 0.6, 0.6),
    Status.NEED_POSE: (1.0, 0.6, 0.0),
    Status.UNVERIFIED: (1.0, 0.9, 0.0),
    Status.LOCALIZED: (0.2, 0.9, 0.2),
    Status.JOINING: (0.3, 0.8, 1.0),
    Status.MAPPING: (0.0, 0.9, 0.9),
    Status.LOST: (1.0, 0.2, 0.2),
}
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
MAX_ERROR_PARAM = 'RGBD/OptimizeMaxError'   # RTAB-Map rejects a closure above this graph error
MAX_STEP = 0.5   # m of odometry between two ticks; more is an odometry reset, not travel
IMAGE_PERIOD = 1.0   # s between brightness readings; the rest of the frames are skipped
CHANNELS = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}


def to_float(value, default):
    """Return a ParameterValue as a float (rtabmap declares strings), or default if unset."""
    try:
        if value.type == ParameterType.PARAMETER_STRING:
            return float(value.string_value)
        if value.type == ParameterType.PARAMETER_DOUBLE:
            return value.double_value
        if value.type == ParameterType.PARAMETER_INTEGER:
            return float(value.integer_value)
    except ValueError:
        pass
    return default


def brightness(msg, stride=8):
    """Return the mean grayscale value (0-255) of every stride-th pixel, or None."""
    channels = CHANNELS.get(msg.encoding)
    if channels is None or not msg.width or not msg.height:
        return None
    rows = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[::stride]
    pixels = rows[:, :msg.width * channels].reshape(len(rows), msg.width, channels)[:, ::stride]
    if channels == 1:
        return float(pixels.mean())
    weights = (0.299, 0.587, 0.114) if msg.encoding.startswith('rgb') else (0.114, 0.587, 0.299)
    return float((pixels[..., :3].astype(np.float32) @ np.array(weights, np.float32)).mean())


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RtabmapSupervisor(Node):

    def __init__(self):
        super().__init__('rtabmap_supervisor')
        p = self.declare_parameter
        p('rtabmap_node', '/rtabmap/rtabmap')
        p('map_frame', 'map')
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')
        p('manage_mode', True)          # false: report only (slam mode, building a new map)
        p('enable_mapping', True)       # false: never switch to mapping; the map is read-only
        p('gate_in_mapping', True)      # loop-closure gate on in mapping (off in localization)
        p('trust_start_pose', False)    # true: RTAB-Map's start pose counts as given (UNVERIFIED)
        p('localized_after', 2)         # accepted closures before LOCALIZED
        p('lost_after_m', 3.0)          # m without a closure before LOCALIZED drops (tracker)
        p('lost_budget_m', 6.0)         # m without a fix or pose before LOST
        # MAPPING turns LOST after this many updates in mapped space with no match to the old map
        p('mapped_lost_updates', 10)
        p('mapped_radius', 1.0)         # m from an old map node = mapped space
        p('graph_refresh', 60.0)        # s between two looks at the old map's node poses
        p('stale_after', 60.0)          # s without an RTAB-Map update before NO_DATA
        p('mapping_after', 10.0)        # s LOCALIZED before switching to mapping
        p('mapping_after_max', 120.0)   # s; mapping_after doubles after each LOST from mapping
        p('guarded_actions', ['/navigate_to_pose', '/navigate_through_poses'])
        # Mapping only in the light; '' turns the check off (always off without manage_mode).
        p('image_topic', '/cameras_head/stereo_masked/left/image_rect')
        p('dark_below', 40.0)           # mean brightness (0-255) below which it counts as dark
        p('light_above', 70.0)          # ...above which it counts as light
        p('dark_after', 5.0)            # s below dark_below before mapping stops
        p('light_after', 30.0)          # s above light_above before mapping may start
        # Both set: the keys that differ between them follow the mode (see set_mode_params).
        p('mapping_params_file', '')
        p('localization_params_file', '')
        cfg = {name: param.value for name, param in self.get_parameters_by_prefix('').items()}
        self.cfg = cfg
        self.rtabmap_node = cfg['rtabmap_node']
        self.mapping_values, self.localization_values = {}, {}
        if cfg['manage_mode'] and cfg['mapping_params_file'] and cfg['localization_params_file']:
            self.mapping_values, self.localization_values = mode_params(
                cfg['mapping_params_file'], cfg['localization_params_file'])
            self.get_logger().info(f'parameters that follow the mode: {self.localization_values} '
                                   f'in localization, {self.mapping_values} in mapping')
        # One of them, to tell which set RTAB-Map has: True mapping, False localization, None
        # not known yet.
        self.probe = next(iter(self.mapping_values), None)
        self.has_mapping_values = None

        self.tracker = SessionTracker(cfg['localized_after'], cfg['lost_after_m'],
                                      cfg['stale_after'])
        self.policy = LocalizationPolicy(
            manage_mode=cfg['manage_mode'], enable_mapping=cfg['enable_mapping'],
            trust_start_pose=cfg['trust_start_pose'], lost_budget_m=cfg['lost_budget_m'],
            mapped_lost_updates=cfg['mapped_lost_updates'], mapping_after=cfg['mapping_after'],
            mapping_after_max=cfg['mapping_after_max'])
        self.light = None
        self.image_at = -math.inf
        if cfg['manage_mode'] and cfg['image_topic']:
            self.light = LightCheck(cfg['dark_below'], cfg['light_above'], cfg['dark_after'],
                                    cfg['light_after'])
            self.create_subscription(Image, cfg['image_topic'], self.on_image,
                                     qos_profile_sensor_data, raw=True)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/localization_status', latched)
        self.overlay_pub = self.create_publisher(OverlayText, '/localization_status_overlay',
                                                 latched)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        self.create_subscription(Info, '/rtabmap/info', self.on_info, 10)
        self.create_subscription(ParameterEvent, '/parameter_events', self.on_param_event, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose,
                                 10)
        self.get_params = self.create_client(GetParameters, f'{self.rtabmap_node}/get_parameters')
        self.set_params = self.create_client(SetParameters, f'{self.rtabmap_node}/set_parameters')
        # rtabmap names its services after the node.
        self.set_mapping = self.create_client(Empty, f'{self.rtabmap_node}/set_mode_mapping')
        self.set_localization = self.create_client(
            Empty, f'{self.rtabmap_node}/set_mode_localization')

        self.active_goals = {}          # action -> number of accepted/executing goals
        self.cancel_clients = {}
        for action in cfg['guarded_actions']:
            self.active_goals[action] = 0
            self.cancel_clients[action] = self.create_client(
                CancelGoal, f'{action}/_action/cancel_goal')
            self.create_subscription(
                GoalStatusArray, f'{action}/_action/status',
                lambda msg, a=action: self.on_goal_status(a, msg), 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.last_xy = None
        self.info_count = 0             # /rtabmap/info messages received
        self.held_pose = None           # 2D Pose Estimate given while mapping, sent again later
        self.held_odom = None           # odom -> base when it was given
        self.held_from = None           # info_count once RTAB-Map was in localization
        self.held_seed = False          # the held pose is the robot's own, not a person's
        self.seed_stamp = None          # stamp of the last seed sent, to tell it apart on return
        self.old_xy = None              # (N, 2) old map node positions in map_frame
        self.graph_sub = None
        self.graph_received = False
        self.graph_at = -math.inf
        self.params_known = False
        self.max_error = 3.0            # RGBD/OptimizeMaxError, read from RTAB-Map at start
        self.gate_sent_at = -math.inf   # last time the gate was set
        self.mapping_after_gate = None  # monotonic time set_mode_mapping waits on the gate from
        self.decision = None
        self.status_sent = ''
        self.node_checked_at = 0.0
        self.cancelled_at = 0.0
        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.publish_overlay)

    # ---- inputs -------------------------------------------------------------------------------

    def on_info(self, info):
        self.info_count += 1
        self.policy.switching = False   # an update: RTAB-Map runs in its new mode
        stats = dict(zip(info.stats_keys, info.stats_values))
        links = len(self.tracker.old_map_links)
        self.tracker.update(info.ref_id, info.loop_closure_id, info.proximity_detection_id, stats)
        old_link = len(self.tracker.old_map_links) > links
        in_mapped = self.in_mapped_space()
        moved = not stats.get('Memory/Small_movement/', 0.0)
        graph_rejected, blamed = self.graph_rejection(stats)
        self.policy.on_update(bool(self.tracker.mapping), in_mapped, moved, graph_rejected,
                              blamed)
        if info.loop_closure_id > 0 or info.proximity_detection_id > 0:
            # In mapping only a closure to the old map checks the pose against it.
            self.policy.on_fix(to_map=not self.tracker.mapping or old_link)

    def on_image(self, data):
        now = time.monotonic()
        if now - self.image_at < IMAGE_PERIOD:
            return
        self.image_at = now
        msg = deserialize_message(data, Image)
        value = brightness(msg)
        if value is None:
            self.get_logger().error(f'{self.cfg["image_topic"]}: encoding {msg.encoding} not '
                                    'supported', throttle_duration_sec=60.0)
            return
        event = self.light.update(value, now)
        if event:
            self.get_logger().warn(event)

    def graph_rejection(self, stats):
        """
        Return (rejected by graph error, the link blamed) for one update.

        RTAB-Map rejects a closure whose graph optimization leaves a link with an error above
        RGBD/OptimizeMaxError times that link's own uncertainty, and reports the worst link.
        """
        if not stats.get(REJECTED_KEY, 0.0):
            return False, ''
        lin = stats.get(MAX_ERROR_RATIO_KEY, 0.0)
        ang = stats.get(MAX_ANG_ERROR_RATIO_KEY, 0.0)
        if not self.max_error or max(lin, ang) <= self.max_error:
            return False, ''
        parts = []
        for ratio, key in ((lin, 'Loop/Optimization_max_error'),
                           (ang, 'Loop/Optimization_max_ang_error')):
            if ratio > self.max_error:
                parts.append(f'link {int(stats.get(key + "_from_id/", 0))}->'
                             f'{int(stats.get(key + "_to_id/", 0))} x{ratio:.0f}')
        return True, ', '.join(parts)

    def on_param_event(self, event):
        if event.node != self.rtabmap_node:
            return
        for param in list(event.new_parameters) + list(event.changed_parameters):
            if param.name == GATE_PARAM:
                self.tracker.gate = is_true(param.value)
            elif param.name == self.probe:
                self.has_mapping_values = same(param.value.string_value,
                                               self.mapping_values[self.probe])
            elif param.name == MAX_ERROR_PARAM:
                self.max_error = to_float(param.value, self.max_error)
            elif param.name == MODE_PARAM and self.tracker.set_mode(is_true(param.value)):
                # RTAB-Map sets the parameter first and publishes nothing until it has switched.
                self.policy.switching = True

    def on_params(self, future):
        values = future.result().values if future.result() else []
        if len(values) == 2 and not self.params_known:
            self.params_known = True
            self.tracker.gate = is_true(values[1])
            self.tracker.set_mode(is_true(values[0]))
            # Each in its own request: one undeclared name empties the whole answer.
            self.get_params.call_async(GetParameters.Request(
                names=[MAX_ERROR_PARAM])).add_done_callback(self.on_max_error)
            if self.probe:
                self.get_params.call_async(GetParameters.Request(
                    names=[self.probe])).add_done_callback(self.on_probe)

    def on_probe(self, future):
        values = future.result().values if future.result() else []
        if not values:
            self.get_logger().error(f'RTAB-Map has no parameter {self.probe} (from the mapping '
                                    'and localization params files): check its name')
            return
        self.has_mapping_values = same(values[0].string_value, self.mapping_values[self.probe])

    def on_max_error(self, future):
        values = future.result().values if future.result() else []
        if values:
            self.max_error = to_float(values[0], self.max_error)
        self.get_logger().info(f'{MAX_ERROR_PARAM} = {self.max_error}')

    def on_initial_pose(self, msg):
        if self.seed_stamp is not None and msg.header.stamp == self.seed_stamp:
            # Our own seed: it only gives RTAB-Map a local map, it confirms nothing.
            return
        if self.tracker.mapping:
            if not self.cfg['manage_mode']:
                self.get_logger().warn('/initialpose ignored: RTAB-Map only takes it in '
                                       'localization mode')
                return
            # RTAB-Map ignores it in mapping: switch, then send it again (tick).
            self.held_pose, self.held_odom, self.held_from = msg, self.odom_pose(), None
            self.held_seed = False
            self.policy.on_initial_pose_while_mapping()
            self.get_logger().info('initial pose received while mapping; switching to '
                                   'localization to apply it')
            return
        self.tracker.on_initial_pose()
        self.policy.on_initial_pose()
        self.get_logger().info('initial pose received')

    def on_goal_status(self, action, msg):
        self.active_goals[action] = sum(1 for s in msg.status_list if s.status in ACTIVE)

    # ---- tick ---------------------------------------------------------------------------------

    def tick(self):
        now = time.monotonic()
        if not self.params_known and self.get_params.service_is_ready() \
                and now - self.node_checked_at >= 1.0:
            self.get_params.call_async(GetParameters.Request(
                names=[MODE_PARAM, GATE_PARAM])).add_done_callback(self.on_params)
        if now - self.node_checked_at >= 1.0:
            # Gone from the graph: the process exited. Known at once, not after stale_after.
            self.node_checked_at = now
            self.tracker.set_node_alive(node_in_graph(self, self.rtabmap_node))
        self.tracker.check_stale()
        self.update_travel()
        self.resend_held_pose()
        self.sample_graph(now)
        self.manage_gate(now)
        if self.light is not None:
            self.policy.dark = self.light.reason(now)

        decision = self.policy.step(self.tracker, now)
        new_status = self.decision is None or decision.status != self.decision.status
        self.decision = decision
        if decision.event:
            self.get_logger().warn(decision.event)
        # A new status goes out at once; detail changes (metres driven) with the 1 Hz overlay.
        if new_status:
            self.publish_status()
            text = (f'==== {decision.status.value} ==== {decision.detail}'
                    + (f' ({decision.debug})' if decision.debug else ''))
            if decision.status in BLOCKED:
                self.get_logger().warn(f'{text}\n     {HINTS[decision.status]}')
            else:
                self.get_logger().info(f'{text}\n     {HINTS[decision.status]}')
            self.publish_overlay()
        if decision.request_mapping is True and not self.ready_for_mapping():
            # Gate and mapping parameters first, while RTAB-Map is still in localization;
            # manage_gate calls set_mode_mapping once RTAB-Map confirms them.
            self.mapping_after_gate = now
            self.set_mode_params(True, now)
        elif decision.request_mapping is not None:
            if decision.request_mapping is False and self.held_pose is None:
                self.hold_map_pose()
            self.request_mode(decision.request_mapping)
        if self.cfg['manage_mode'] and decision.status in BLOCKED:
            self.cancel_goals(now)

    def odom_pose(self):
        """Return odom -> base as (x, y, yaw), or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.cfg['odom_frame'], self.cfg['base_frame'],
                                                Time()).transform
        except tf2_ros.TransformException:
            return None
        return (t.translation.x, t.translation.y, yaw_of(t.rotation))

    def update_travel(self):
        pose = self.odom_pose()
        if pose is None:
            return
        xy = pose[:2]
        if self.last_xy is not None:
            step = math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
            if step <= MAX_STEP:
                self.policy.add_travel(step)
        self.last_xy = xy

    def in_mapped_space(self):
        """Return True when the robot is within mapped_radius of a node of the old map."""
        if self.old_xy is None or not len(self.old_xy):
            return False
        try:
            t = self.tf_buffer.lookup_transform(self.cfg['map_frame'], self.cfg['base_frame'],
                                                Time()).transform
        except tf2_ros.TransformException:
            return False
        d = np.hypot(self.old_xy[:, 0] - t.translation.x, self.old_xy[:, 1] - t.translation.y)
        return bool(d.min() <= self.cfg['mapped_radius'])

    def sample_graph(self, now):
        """
        Take one mapGraph message every graph_refresh s while LOCALIZED or linked.

        Not in JOINING: until the link, RTAB-Map's optimized graph is the new session alone, so
        the positions from before the switch are kept (the map frame does not move).
        """
        if self.graph_received:
            self.graph_received = False
            self.stop_graph()
        if self.tracker.state not in (State.LOCALIZED, State.MAPPING_LINKED):
            self.stop_graph()
            return
        if self.graph_sub is None and now - self.graph_at >= self.cfg['graph_refresh']:
            self.graph_at = now
            self.graph_sub = self.create_subscription(MapGraph, '/rtabmap/mapGraph',
                                                      self.on_graph, 1)

    def on_graph(self, msg):
        if self.graph_sub is None:
            return
        # In localization every node is the old map; in mapping, the ones before this session.
        start = self.tracker.session_start if self.tracker.mapping else None
        if self.tracker.mapping and start is None:
            return
        old = [(p.position.x, p.position.y)
               for i, p in zip(msg.poses_id, msg.poses) if start is None or i < start]
        self.old_xy = np.array(old, dtype=float).reshape(-1, 2)
        self.get_logger().info(f'old map: {len(old)} nodes', throttle_duration_sec=300.0)
        self.graph_received = True      # unsubscribed in the next tick, not in this callback

    def stop_graph(self):
        if self.graph_sub is not None:
            self.destroy_subscription(self.graph_sub)
            self.graph_sub = None

    def hold_map_pose(self):
        """
        Hold the robot's map pose, sent as the initial pose once RTAB-Map runs in localization.

        Switched to localization without a recent closure, RTAB-Map clears its local map, and
        lidar proximity detection has nothing to match against until it gets a pose.
        """
        try:
            t = self.tf_buffer.lookup_transform(self.cfg['map_frame'], self.cfg['base_frame'],
                                                Time()).transform
        except tf2_ros.TransformException:
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.cfg['map_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = t.translation.x, t.translation.y
        yaw = yaw_of(t.rotation)
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        self.held_pose, self.held_odom, self.held_from = msg, self.odom_pose(), None
        self.held_seed = True

    def resend_held_pose(self):
        """Send a held pose once RTAB-Map runs in localization."""
        if self.held_pose is None or self.tracker.mapping is not False:
            return
        if self.held_from is None:
            # The mode parameter changes before RTAB-Map has finished switching (it publishes
            # nothing meanwhile); two updates after it, the switch is done.
            self.held_from = self.info_count
        elif self.info_count >= self.held_from + 2:
            msg, self.held_pose = self.held_pose, None
            now = self.odom_pose()
            if self.held_odom is not None and now is not None:
                # The robot kept driving during the switch: move the pose along with it.
                p = msg.pose.pose
                x, y, yaw = carry_pose((p.position.x, p.position.y, yaw_of(p.orientation)),
                                       self.held_odom, now)
                p.position.x, p.position.y = x, y
                p.orientation.x = p.orientation.y = 0.0
                p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            msg.header.stamp = self.get_clock().now().to_msg()
            if self.held_seed:
                self.seed_stamp = msg.header.stamp
                self.get_logger().info('localization mode: sending the map pose from before the '
                                       'switch, so RTAB-Map keeps a local map to match against')
            else:
                self.get_logger().info('localization mode: sending the held initial pose again')
            self.held_seed = False
            self.pose_pub.publish(msg)

    def manage_gate(self, now):
        """Keep the loop-closure gate off in localization; finish a switch waiting on it."""
        if not self.cfg['manage_mode'] or self.tracker.gate is None:
            return
        if self.mapping_after_gate is not None:
            if self.ready_for_mapping():
                self.mapping_after_gate = None
                self.request_mode(True)
            elif now - self.mapping_after_gate > 10.0:
                self.get_logger().error('RTAB-Map did not confirm the loop-closure gate or the '
                                        'mapping parameters; not switching to mapping')
                self.mapping_after_gate = None
                self.policy.request_failed()
            elif now - self.gate_sent_at > 3.0:
                self.set_mode_params(True, now)
            return
        if self.tracker.mapping is False and (self.tracker.gate or self.has_mapping_values) \
                and now - self.gate_sent_at > 5.0:
            self.set_mode_params(False, now)

    def ready_for_mapping(self):
        """Return whether the gate and the mapping parameters are set for mapping."""
        return ((not self.cfg['gate_in_mapping'] or self.tracker.gate is True)
                and (self.probe is None or self.has_mapping_values is True))

    def set_mode_params(self, mapping, now):
        """
        Set the gate and the mode's parameters, only ever while RTAB-Map is in localization.

        RTAB-Map applies a set_parameters change at once, and applying in mapping resets its map
        (CoreWrapper::applyParameters). So the mapping values go in before set_mode_mapping, and
        the localization values once it is back in localization.
        """
        if not self.set_params.service_is_ready():
            self.get_logger().error('set_parameters service of RTAB-Map not available',
                                    throttle_duration_sec=5.0)
            return
        self.gate_sent_at = now
        values = dict(self.mapping_values if mapping else self.localization_values)
        if not mapping or self.cfg['gate_in_mapping']:
            values[GATE_PARAM] = 'true' if mapping else 'false'
        self.get_logger().info(
            ('mapping parameters' if mapping else 'localization parameters (gate off: lidar '
             'proximity detection works right after a 2D Pose Estimate)') + f': {values}')
        self.set_params.call_async(SetParameters.Request(parameters=[
            Parameter(name=k, value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                   string_value=v))
            for k, v in values.items()]))

    def request_mode(self, mapping):
        client, name = ((self.set_mapping, 'set_mode_mapping') if mapping
                        else (self.set_localization, 'set_mode_localization'))
        if not client.service_is_ready():
            self.get_logger().error(f'{name} service not available', throttle_duration_sec=5.0)
            self.policy.request_failed()
            return
        self.get_logger().info(f'calling {name}')
        client.call_async(Empty.Request())

    def cancel_goals(self, now):
        if now - self.cancelled_at < 1.0:
            return
        for action, count in self.active_goals.items():
            client = self.cancel_clients[action]
            if count and client.service_is_ready():
                # A zero goal id and stamp cancels every goal on the server.
                client.call_async(CancelGoal.Request())
                self.cancelled_at = now
                self.get_logger().warn(f'{self.decision.status.value}: cancelled {count} goal(s) '
                                       f'on {action}')

    # ---- outputs ------------------------------------------------------------------------------

    def publish_status(self):
        d = self.decision
        text = f'{d.status.value} | {d.detail} | {HINTS[d.status]}'
        if text != self.status_sent:
            self.status_sent = text
            self.status_pub.publish(String(data=text))

    def publish_overlay(self):
        if self.decision is None:
            return
        self.publish_status()
        m = OverlayText()
        m.action = OverlayText.ADD
        m.horizontal_alignment, m.vertical_alignment = OverlayText.LEFT, OverlayText.TOP
        m.horizontal_distance, m.vertical_distance = 10, 10
        m.width, m.height = 520, 60
        m.text_size = 14.0
        m.font = 'DejaVu Sans Mono'
        m.fg_color = ColorRGBA(a=1.0)
        m.fg_color.r, m.fg_color.g, m.fg_color.b = COLOURS[self.decision.status]
        m.bg_color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.6)
        m.text = f'{self.decision.status.value}\n{self.decision.detail}'
        self.overlay_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
