"""Adapt FAST-LIO2's full IMU pose and fail safe if it diverges."""

import math
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException
from tf2_ros import TransformListener


def _hamilton_product(left, right):
    """Return the unnormalized Hamilton product of xyzw quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_multiply(left, right):
    """Return the normalized Hamilton product of xyzw quaternions."""
    result = _hamilton_product(left, right)
    norm = math.sqrt(sum(value * value for value in result))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError('invalid quaternion')
    return tuple(value / norm for value in result)


def quaternion_conjugate(quaternion):
    """Return the conjugate of an xyzw quaternion."""
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def rotate_vector(quaternion, vector):
    """Rotate a vector by an xyzw quaternion."""
    pure = (vector[0], vector[1], vector[2], 0.0)
    rotated = _hamilton_product(
        _hamilton_product(quaternion, pure),
        quaternion_conjugate(quaternion),
    )
    return rotated[:3]


def yaw_quaternion(quaternion):
    """Project an xyzw orientation onto gravity-aligned yaw."""
    x, y, z, w = quaternion
    yaw = math.atan2(2.0 * (w * z + x * y),
                     1.0 - 2.0 * (y * y + z * z))
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def quaternion_rpy(quaternion):
    """Return roll, pitch and yaw for an xyzw quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError('invalid quaternion')
    x, y, z, w = (value / norm for value in quaternion)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


class PosePlausibilityGuard:
    """Latch when a ground robot's LIO state becomes physically impossible."""

    def __init__(
        self,
        grace_period=20.0,
        max_planar_speed=0.50,
        max_vertical_speed=0.50,
        max_tilt=math.radians(15.0),
        max_height_change=0.20,
        max_step_speed=0.50,
        violations_to_trip=2,
    ):
        self.grace_period = grace_period
        self.max_planar_speed = max_planar_speed
        self.max_vertical_speed = max_vertical_speed
        self.max_tilt = max_tilt
        self.max_height_change = max_height_change
        self.max_step_speed = max_step_speed
        self.violations_to_trip = violations_to_trip
        self.first_stamp = None
        self.previous_stamp = None
        self.previous_position = None
        self.reference_height = None
        self.violation_count = 0
        self.fault_reason = None

    @property
    def armed(self):
        return (
            self.first_stamp is not None
            and self.previous_stamp - self.first_stamp >= self.grace_period
        )

    def evaluate(self, stamp, position, orientation, linear_velocity):
        """Return a latched fault reason, or None while the pose is safe."""
        if self.fault_reason is not None:
            return self.fault_reason
        values = position + orientation + linear_velocity + (stamp,)
        if not all(math.isfinite(value) for value in values):
            return self._trip('non-finite FAST-LIO state')

        try:
            roll, pitch, _ = quaternion_rpy(orientation)
        except ValueError:
            return self._trip('invalid FAST-LIO orientation')

        if self.first_stamp is None:
            self.first_stamp = stamp
            self.reference_height = position[2]
        elif self.previous_stamp is not None and stamp <= self.previous_stamp:
            return self._trip('non-increasing FAST-LIO timestamp')
        elapsed = stamp - self.first_stamp
        reasons = []
        if elapsed >= self.grace_period:
            planar_speed = math.hypot(linear_velocity[0], linear_velocity[1])
            if planar_speed > self.max_planar_speed:
                reasons.append(f'planar speed {planar_speed:.3f} m/s')
            if abs(linear_velocity[2]) > self.max_vertical_speed:
                reasons.append(
                    f'vertical speed {abs(linear_velocity[2]):.3f} m/s'
                )
            tilt = max(abs(roll), abs(pitch))
            if tilt > self.max_tilt:
                reasons.append(f'tilt {math.degrees(tilt):.1f} deg')
            height_change = abs(position[2] - self.reference_height)
            if height_change > self.max_height_change:
                reasons.append(f'height change {height_change:.3f} m')
            if (
                self.previous_stamp is not None
                and self.previous_position is not None
            ):
                delta_time = stamp - self.previous_stamp
                step_speed = math.hypot(
                    position[0] - self.previous_position[0],
                    position[1] - self.previous_position[1],
                ) / delta_time
                if step_speed > self.max_step_speed:
                    reasons.append(f'pose step {step_speed:.3f} m/s')

        self.previous_stamp = stamp
        self.previous_position = position
        if reasons:
            self.violation_count += 1
            if self.violation_count >= self.violations_to_trip:
                return self._trip('; '.join(reasons))
        else:
            self.violation_count = 0
        return None

    def _trip(self, reason):
        self.fault_reason = reason
        return reason


def compose_pose(position, orientation, child_translation, child_rotation):
    """Compose world->parent and parent->child rigid transforms."""
    rotated = rotate_vector(orientation, child_translation)
    return (
        tuple(position[index] + rotated[index] for index in range(3)),
        quaternion_multiply(orientation, child_rotation),
    )


class FastLioBaseAdapter(Node):
    """Publish full LIO sensor TF and planar odom->base_footprint exactly once."""

    def __init__(self):
        super().__init__('fast_lio_base_adapter')
        self.declare_parameter('input_topic', '/fast_lio/imu_odom')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('lio_frame', 'fast_lio_imu')
        self.declare_parameter('physical_imu_frame', 'imu_link')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter(
            'emergency_stop_topic', '/localization/emergency_stop'
        )
        self.declare_parameter(
            'fault_reason_topic', '/localization/fault_reason'
        )
        self.declare_parameter('startup_grace_period', 20.0)
        self.declare_parameter('max_planar_speed', 0.50)
        self.declare_parameter('max_vertical_speed', 0.50)
        self.declare_parameter('max_tilt_degrees', 15.0)
        self.declare_parameter('max_height_change', 0.20)
        self.declare_parameter('max_step_speed', 0.50)
        # Nav2 activation can briefly contend for CPU on the Jetson. A single
        # observed startup pause slightly exceeded 0.5 s while healthy LIO
        # immediately returned to 10 Hz. Sustained two-second silence remains
        # a sensor/estimator fault; pose and velocity guards stay continuous.
        self.declare_parameter('max_output_silence', 2.0)

        self._world_frame = str(self.get_parameter('world_frame').value)
        self._lio_frame = str(self.get_parameter('lio_frame').value)
        self._physical_imu_frame = str(
            self.get_parameter('physical_imu_frame').value
        )
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._publisher = self.create_publisher(
            Odometry, str(self.get_parameter('output_topic').value), 20
        )
        self._subscription = self.create_subscription(
            Odometry,
            str(self.get_parameter('input_topic').value),
            self._on_odometry,
            20,
        )
        fault_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._emergency_publisher = self.create_publisher(
            Bool,
            str(self.get_parameter('emergency_stop_topic').value),
            fault_qos,
        )
        self._reason_publisher = self.create_publisher(
            String,
            str(self.get_parameter('fault_reason_topic').value),
            fault_qos,
        )
        self._navigation_manager = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self._guard = PosePlausibilityGuard(
            grace_period=float(
                self.get_parameter('startup_grace_period').value
            ),
            max_planar_speed=float(
                self.get_parameter('max_planar_speed').value
            ),
            max_vertical_speed=float(
                self.get_parameter('max_vertical_speed').value
            ),
            max_tilt=math.radians(float(
                self.get_parameter('max_tilt_degrees').value
            )),
            max_height_change=float(
                self.get_parameter('max_height_change').value
            ),
            max_step_speed=float(
                self.get_parameter('max_step_speed').value
            ),
        )
        self._max_output_silence = float(
            self.get_parameter('max_output_silence').value
        )
        self._last_source_wall = None
        # Wall time of the first FAST-LIO message.  The silence watchdog arms
        # on wall time, not on source stamps: a stream that dies during the
        # grace period would otherwise never advance the stamps and never arm.
        self._first_source_wall = None
        self._startup_grace_period = float(
            self.get_parameter('startup_grace_period').value
        )
        self._fault_latched = False
        self._pause_requested = False
        self._watchdog = self.create_timer(0.05, self._watchdog_callback)
        self._base_in_imu = None
        self._waiting_logged = False
        self._emergency_publisher.publish(Bool(data=False))

    @staticmethod
    def _quaternion(message):
        return (message.x, message.y, message.z, message.w)

    @staticmethod
    def _assign_quaternion(message, quaternion):
        message.x, message.y, message.z, message.w = quaternion

    def _lookup_base_in_imu(self):
        if self._base_in_imu is not None:
            return self._base_in_imu
        try:
            transform = self._tf_buffer.lookup_transform(
                self._physical_imu_frame, self._base_frame, Time()
            ).transform
        except TransformException as error:
            if not self._waiting_logged:
                self.get_logger().warning(
                    f'waiting for {self._physical_imu_frame} <- '
                    f'{self._base_frame}: {error}'
                )
                self._waiting_logged = True
            return None
        self._base_in_imu = (
            (transform.translation.x,
             transform.translation.y,
             transform.translation.z),
            self._quaternion(transform.rotation),
        )
        self.get_logger().info(
            f'using calibrated {self._physical_imu_frame} <- '
            f'{self._base_frame} transform'
        )
        return self._base_in_imu

    def _on_odometry(self, source):
        self._last_source_wall = time.monotonic()
        if self._first_source_wall is None:
            self._first_source_wall = self._last_source_wall
        base_in_imu = self._lookup_base_in_imu()
        if base_in_imu is None:
            return
        imu_position = (
            source.pose.pose.position.x,
            source.pose.pose.position.y,
            source.pose.pose.position.z,
        )
        imu_orientation = self._quaternion(source.pose.pose.orientation)
        if not all(math.isfinite(value) for value in
                   imu_position + imu_orientation):
            self._trip('non-finite FAST-LIO2 pose')
            return
        source_stamp = (
            float(source.header.stamp.sec)
            + float(source.header.stamp.nanosec) * 1.0e-9
        )
        linear_velocity = (
            source.twist.twist.linear.x,
            source.twist.twist.linear.y,
            source.twist.twist.linear.z,
        )
        reason = self._guard.evaluate(
            source_stamp,
            imu_position,
            imu_orientation,
            linear_velocity,
        )
        if reason is not None:
            self._trip(reason)
            return
        if self._fault_latched:
            return
        base_position, base_orientation = compose_pose(
            imu_position, imu_orientation, *base_in_imu
        )

        output = Odometry()
        output.header = source.header
        output.header.frame_id = self._world_frame
        output.child_frame_id = self._base_frame
        output.pose.pose.position.x = base_position[0]
        output.pose.pose.position.y = base_position[1]
        output.pose.pose.position.z = 0.0
        self._assign_quaternion(
            output.pose.pose.orientation, yaw_quaternion(base_orientation)
        )
        output.pose.covariance = source.pose.covariance

        imu_to_base_rotation = quaternion_conjugate(base_in_imu[1])
        linear = rotate_vector(
            imu_to_base_rotation,
            (source.twist.twist.linear.x,
             source.twist.twist.linear.y,
             source.twist.twist.linear.z),
        )
        angular = rotate_vector(
            imu_to_base_rotation,
            (source.twist.twist.angular.x,
             source.twist.twist.angular.y,
             source.twist.twist.angular.z),
        )
        output.twist.twist.linear.x = linear[0]
        output.twist.twist.linear.y = linear[1]
        output.twist.twist.linear.z = 0.0
        output.twist.twist.angular.x = 0.0
        output.twist.twist.angular.y = 0.0
        output.twist.twist.angular.z = angular[2]
        output.twist.covariance = source.twist.covariance
        self._publisher.publish(output)

        sensor_tf = TransformStamped()
        sensor_tf.header = source.header
        sensor_tf.header.frame_id = self._world_frame
        sensor_tf.child_frame_id = self._lio_frame
        sensor_tf.transform.translation.x = imu_position[0]
        sensor_tf.transform.translation.y = imu_position[1]
        sensor_tf.transform.translation.z = imu_position[2]
        self._assign_quaternion(
            sensor_tf.transform.rotation, imu_orientation
        )

        base_tf = TransformStamped()
        base_tf.header = output.header
        base_tf.child_frame_id = self._base_frame
        base_tf.transform.translation.x = base_position[0]
        base_tf.transform.translation.y = base_position[1]
        base_tf.transform.translation.z = 0.0
        self._assign_quaternion(
            base_tf.transform.rotation,
            self._quaternion(output.pose.pose.orientation),
        )
        self._tf_broadcaster.sendTransform([sensor_tf, base_tf])

    def _watchdog_callback(self):
        if self._fault_latched:
            self._emergency_publisher.publish(Bool(data=True))
            self._request_navigation_pause()
            return
        if (
            self._silence_watchdog_armed()
            and time.monotonic() - self._last_source_wall
            > self._max_output_silence
        ):
            self._trip(
                'FAST-LIO odometry silent for more than '
                f'{self._max_output_silence:.2f} s'
            )

    def _silence_watchdog_armed(self):
        return (
            self._first_source_wall is not None
            and self._last_source_wall is not None
            and time.monotonic() - self._first_source_wall
            >= self._startup_grace_period
        )

    def _trip(self, reason):
        if self._fault_latched:
            return
        self._fault_latched = True
        self.get_logger().fatal(
            f'LOCALIZATION FAULT: {reason}; motion is locked until restart'
        )
        self._reason_publisher.publish(String(data=reason))
        self._emergency_publisher.publish(Bool(data=True))
        self._request_navigation_pause()

    def _request_navigation_pause(self):
        if self._pause_requested or not self._navigation_manager.service_is_ready():
            return
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.PAUSE
        self._navigation_manager.call_async(request)
        self._pause_requested = True
        self.get_logger().error('requested Nav2 lifecycle pause')


def main(args=None):
    rclpy.init(args=args)
    node = FastLioBaseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
