"""
Re-express the lidar IMU in the robot base frame for GLIM.

GLIM is configured with imu_frame_id "base_link", but the IMU sits inside the right Hesai,
about 1.5 m up the mast. This node:

  * converts the Hesai's native units (g, deg/s) to SI (m/s^2, rad/s),
  * rotates the measurements into the base frame, applying the per-robot alignment
    correction from imu_alignment_calibration if one exists,
  * removes the lever-arm terms from the acceleration:

        a_base = a_imu - alpha x r - omega x (omega x r)

    where r is the IMU position in the base frame and alpha is estimated by
    differentiating the gyro. Angular velocity needs no lever-arm correction.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener
import yaml

STANDARD_GRAVITY = 9.80665
ACCEL_DETECT_SAMPLES = 20


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ])


def quat_multiply(a, b):
    """Hamilton product a * b, both as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def default_alignment_path():
    fleet_path = os.environ.get('HELLO_FLEET_PATH', os.path.expanduser('~/stretch_user'))
    fleet_id = os.environ.get('HELLO_FLEET_ID', 'unknown_robot')
    return os.path.join(fleet_path, fleet_id, 'calibration_lidar_imu', 'imu_alignment.yaml')


class ImuFrameTransformer(Node):

    def __init__(self):
        super().__init__('imu_frame_transformer')

        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('gyro_scale', math.pi / 180.0)
        # 0.0 auto-detects g vs m/s^2 from the gravity norm at startup.
        self.declare_parameter('accel_scale', 0.0)
        self.declare_parameter('enable_lever_arm', True)
        self.declare_parameter('angular_accel_cutoff_hz', 10.0)
        # Gaps longer than this reset the angular-acceleration estimate.
        self.declare_parameter('max_dt', 0.05)
        self.declare_parameter('tf_timeout', 0.05)
        # Resolve the mount transform once and drop the TF subscription.
        self.declare_parameter('static_mount', True)
        self.declare_parameter('alignment_file', default_alignment_path())
        self.declare_parameter('apply_alignment', True)
        self.declare_parameter('apply_alignment_scale', True)

        self.target_frame = self.get_parameter('target_frame').value
        self.gyro_scale = self.get_parameter('gyro_scale').value
        self.accel_scale = self.get_parameter('accel_scale').value or None
        self.enable_lever_arm = self.get_parameter('enable_lever_arm').value
        self.alpha_cutoff_hz = self.get_parameter('angular_accel_cutoff_hz').value
        self.max_dt = self.get_parameter('max_dt').value
        self.tf_timeout = Duration(seconds=self.get_parameter('tf_timeout').value)
        self.static_mount = self.get_parameter('static_mount').value
        self.apply_alignment = self.get_parameter('apply_alignment').value
        self.apply_alignment_scale = self.get_parameter('apply_alignment_scale').value
        self.alignment = self._load_alignment(self.get_parameter('alignment_file').value)

        self.tf_buffer = None
        self.tf_listener = None
        self._subscribe_tf()

        self._prev_t = None
        self._prev_omega = None
        self._alpha = np.zeros(3)
        self._accel_norms = []
        self._logged_lever_arm = False
        # (source_frame, rot, lever, q_rot, align_scale) when static_mount is set.
        self._tf_cache = None

        self.subscription = self.create_subscription(
            Imu, '/lidar_imu_right', self.imu_callback, 10)
        self.publisher = self.create_publisher(Imu, 'lidar_imu', qos_profile_sensor_data)

        self.get_logger().info(
            f'target_frame={self.target_frame} '
            f'lever_arm_compensation={"ON" if self.enable_lever_arm else "OFF"} '
            f'static_mount={self.static_mount}')

    def _load_alignment(self, path):
        """Return {frame: (rotation, quaternion, scale)}; empty if unavailable."""
        if not self.apply_alignment:
            self.get_logger().warn('IMU alignment disabled.')
            return {}
        if not path or not os.path.exists(path):
            self.get_logger().warn(f'No IMU alignment file at {path}; running uncorrected.')
            return {}
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
            sensors = (data.get('imu_alignment') or {}).get('sensors') or {}
        except Exception as e:
            self.get_logger().error(f'Failed to read {path}: {e}; running uncorrected.')
            return {}

        alignment = {}
        for frame, cfg in sensors.items():
            try:
                q = cfg['correction_quaternion']
                quat = (float(q['x']), float(q['y']), float(q['z']), float(q['w']))
                scale = float(cfg.get('scale', 1.0)) if self.apply_alignment_scale else 1.0
                alignment[frame] = (quat_to_matrix(*quat), quat, scale)
                self.get_logger().info(
                    f'IMU alignment for {frame}: '
                    f'tilt={cfg.get("tilt_deg", float("nan")):.3f} deg, scale={scale:.5f}')
            except Exception as e:
                self.get_logger().error(f'Invalid alignment entry for {frame}: {e}')
        return alignment

    def _resolve_accel_scale(self, accel):
        """
        Detect the accelerometer unit from the median gravity norm.

        Nothing is published until this succeeds, because GLIM latches its own
        acceleration scale from the first message it receives.
        """
        self._accel_norms.append(float(np.linalg.norm(accel)))
        if len(self._accel_norms) < ACCEL_DETECT_SAMPLES:
            return False

        norm = float(np.median(self._accel_norms))
        self._accel_norms.clear()

        if 7.0 < norm < 12.0:
            self.accel_scale, unit = 1.0, 'm/s^2'
        elif 0.8 < norm < 1.2:
            self.accel_scale, unit = STANDARD_GRAVITY, 'g'
        else:
            self.get_logger().warn(
                f'Cannot identify accelerometer unit (|a|={norm:.3f}). Keep the robot '
                'still or set accel_scale.', throttle_duration_sec=5.0)
            return False

        self.get_logger().info(f'Accelerometer unit: {unit} (|a|={norm:.3f})')
        return True

    def _angular_acceleration(self, omega, t):
        """Low-pass filtered d(omega)/dt; zero after a gap or on the first sample."""
        if self._prev_t is None:
            self._prev_t, self._prev_omega = t, omega
            return self._alpha

        dt = t - self._prev_t
        self._prev_t, prev_omega, self._prev_omega = t, self._prev_omega, omega
        if dt <= 0.0 or dt > self.max_dt:
            self._alpha = np.zeros(3)
            return self._alpha

        raw = (omega - prev_omega) / dt
        tau = 1.0 / (2.0 * math.pi * self.alpha_cutoff_hz)
        self._alpha = self._alpha + (dt / (tau + dt)) * (raw - self._alpha)
        return self._alpha

    def _subscribe_tf(self):
        if self.tf_listener is None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

    def _unsubscribe_tf(self):
        if self.tf_listener is not None:
            self.tf_listener.unregister()
            self.tf_listener = None
            self.tf_buffer = None

    def _transform_for(self, source_frame, stamp):
        """Return (rot, lever, q_rot, align_scale) from source_frame to target, or None."""
        if self._tf_cache is not None:
            if self._tf_cache[0] == source_frame:
                return self._tf_cache[1:]
            self.get_logger().warn(
                f'IMU frame changed {self._tf_cache[0]} -> {source_frame}; re-resolving.')
            self._tf_cache = None

        if source_frame == self.target_frame:
            rot, lever, q_rot = np.eye(3), np.zeros(3), (0.0, 0.0, 0.0, 1.0)
        else:
            self._subscribe_tf()
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, source_frame, Time.from_msg(stamp),
                    timeout=self.tf_timeout)
            except Exception as e:
                self.get_logger().warn(
                    f'No transform {source_frame} -> {self.target_frame}: {e}',
                    throttle_duration_sec=5.0)
                return None

            t = tf.transform.translation
            q = tf.transform.rotation
            lever = np.array([t.x, t.y, t.z])
            q_rot = (q.x, q.y, q.z, q.w)
            rot = quat_to_matrix(*q_rot)

            if not self._logged_lever_arm:
                self._logged_lever_arm = True
                self.get_logger().info(
                    f'Lever arm {source_frame} -> {self.target_frame}: '
                    f'[{lever[0]:.3f}, {lever[1]:.3f}, {lever[2]:.3f}] m')

        align_scale = 1.0
        align = self.alignment.get(source_frame)
        if align is not None:
            correction, q_correction, align_scale = align
            rot = correction @ rot
            q_rot = quat_multiply(q_correction, q_rot)
        elif self.alignment:
            self.get_logger().warn(
                f'No alignment entry for {source_frame}; running uncorrected.',
                throttle_duration_sec=30.0)

        if self.static_mount:
            self._tf_cache = (source_frame, rot, lever, q_rot, align_scale)
            self._unsubscribe_tf()

        return rot, lever, q_rot, align_scale

    def imu_callback(self, msg: Imu):
        omega_src = self.gyro_scale * np.array([
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        accel_src = np.array([
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        if self.accel_scale is None and not self._resolve_accel_scale(accel_src):
            return

        resolved = self._transform_for(msg.header.frame_id, msg.header.stamp)
        if resolved is None:
            return
        rot, lever, q_rot, align_scale = resolved
        accel_scale = self.accel_scale * align_scale

        omega = rot @ omega_src
        accel = rot @ (accel_src * accel_scale)

        if self.enable_lever_arm and np.any(lever):
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            alpha = self._angular_acceleration(omega, t)
            accel = accel - np.cross(alpha, lever) - np.cross(omega, np.cross(omega, lever))

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame
        (out.orientation.x, out.orientation.y,
         out.orientation.z, out.orientation.w) = quat_multiply(
            q_rot, (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
        out.angular_velocity.x, out.angular_velocity.y, out.angular_velocity.z = omega
        out.linear_acceleration.x, out.linear_acceleration.y, out.linear_acceleration.z = accel

        def rotate_cov(cov, scale=1.0):
            c = np.array(cov, dtype=float).reshape(3, 3) * scale
            return (rot @ c @ rot.T).flatten()

        out.orientation_covariance = rotate_cov(msg.orientation_covariance)
        out.angular_velocity_covariance = rotate_cov(
            msg.angular_velocity_covariance, self.gyro_scale ** 2)
        out.linear_acceleration_covariance = rotate_cov(
            msg.linear_acceleration_covariance, accel_scale ** 2)

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImuFrameTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
