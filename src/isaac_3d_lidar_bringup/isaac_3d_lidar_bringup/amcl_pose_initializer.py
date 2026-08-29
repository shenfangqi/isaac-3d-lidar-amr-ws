import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def _yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


def _angular_difference(first, second):
    return math.atan2(
        math.sin(first - second),
        math.cos(first - second),
    )


class AmclPoseInitializer(Node):
    """Publish and verify one reproducible AMCL initial pose."""

    def __init__(self):
        super().__init__('amcl_pose_initializer')

        self.declare_parameter('mode', 'odom_identity')
        self.declare_parameter('odom_topic', '/chassis/odom')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('fixed_x', 0.0)
        self.declare_parameter('fixed_y', 0.0)
        self.declare_parameter('fixed_yaw', 0.0)
        self.declare_parameter('covariance_x', 0.25)
        self.declare_parameter('covariance_y', 0.25)
        self.declare_parameter('covariance_yaw', 0.0685389)
        self.declare_parameter('publish_count', 3)
        self.declare_parameter('publish_interval_sec', 0.3)
        self.declare_parameter('stationary_samples', 5)
        self.declare_parameter('max_linear_speed', 0.02)
        self.declare_parameter('max_angular_speed', 0.03)
        self.declare_parameter('confirmation_samples', 2)
        self.declare_parameter('max_confirmation_xy_error', 1.0)
        self.declare_parameter('max_confirmation_yaw_error', 0.75)
        self.declare_parameter('startup_timeout_sec', 75.0)

        self._mode = self.get_parameter('mode').value
        if self._mode not in ('odom_identity', 'fixed'):
            raise ValueError(
                'mode must be odom_identity or fixed; use launch mode manual '
                'to disable this node'
            )

        self._global_frame = self.get_parameter('global_frame').value
        self._publish_count = self.get_parameter('publish_count').value
        self._publish_interval = self.get_parameter(
            'publish_interval_sec'
        ).value
        self._stationary_samples = self.get_parameter(
            'stationary_samples'
        ).value
        self._confirmation_samples = self.get_parameter(
            'confirmation_samples'
        ).value
        self._startup_timeout = self.get_parameter(
            'startup_timeout_sec'
        ).value

        if self._publish_count < 1 or self._confirmation_samples < 1:
            raise ValueError('publish and confirmation counts must be positive')

        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._odom_callback,
            10,
        )
        self._amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10,
        )
        self._state_client = self.create_client(
            GetState,
            '/amcl/get_state',
        )
        self._timer = self.create_timer(0.1, self._timer_callback)

        self._start_time = time.monotonic()
        self._state_future = None
        self._amcl_active = False
        self._latest_odom = None
        self._stationary_count = 0
        self._initial_pose = None
        self._next_publish_time = None
        self._published_count = 0
        self._confirmed_count = 0
        self.done = False
        self.exit_code = 1

        if self._mode == 'odom_identity':
            self.get_logger().warning(
                'Using odom_identity: valid only when map and odom share the '
                'same coordinates, as in the current Isaac v3 simulation'
            )
        else:
            self.get_logger().info('Using configured fixed AMCL initial pose')

    def _odom_callback(self, msg):
        self._latest_odom = msg
        linear = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )
        angular = abs(msg.twist.twist.angular.z)
        if (
            linear <= self.get_parameter('max_linear_speed').value
            and angular <= self.get_parameter('max_angular_speed').value
        ):
            self._stationary_count += 1
        else:
            self._stationary_count = 0

    def _make_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self._global_frame

        if self._mode == 'odom_identity':
            pose = self._latest_odom.pose.pose
            x = pose.position.x
            y = pose.position.y
            yaw = _yaw_from_quaternion(pose.orientation)
        else:
            x = self.get_parameter('fixed_x').value
            y = self.get_parameter('fixed_y').value
            yaw = self.get_parameter('fixed_yaw').value

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = self.get_parameter('covariance_x').value
        msg.pose.covariance[7] = self.get_parameter('covariance_y').value
        msg.pose.covariance[35] = self.get_parameter(
            'covariance_yaw'
        ).value
        return msg

    def _request_amcl_state(self):
        if self._amcl_active:
            return
        if self._state_future is None:
            if self._state_client.service_is_ready():
                self._state_future = self._state_client.call_async(
                    GetState.Request()
                )
            return
        if not self._state_future.done():
            return
        try:
            response = self._state_future.result()
            self._amcl_active = response.current_state.label == 'active'
        except Exception as exc:
            self.get_logger().warning(f'AMCL state request failed: {exc}')
        self._state_future = None

    def _ready_to_publish(self):
        if not self._amcl_active:
            return False
        if self._publisher.get_subscription_count() < 1:
            return False
        if self._mode == 'fixed':
            return True
        return (
            self._latest_odom is not None
            and self._stationary_count >= self._stationary_samples
        )

    def _publish_initial_pose(self):
        self._initial_pose.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(self._initial_pose)
        self._published_count += 1
        self._next_publish_time = time.monotonic() + self._publish_interval
        pose = self._initial_pose.pose.pose
        self.get_logger().info(
            'Published Initial Pose '
            f'{self._published_count}/{self._publish_count}: '
            f'x={pose.position.x:.3f}, y={pose.position.y:.3f}, '
            f'yaw={_yaw_from_quaternion(pose.orientation):.3f}'
        )

    def _amcl_pose_callback(self, msg):
        if self._initial_pose is None or self.done:
            return
        if msg.header.frame_id and msg.header.frame_id != self._global_frame:
            return

        expected = self._initial_pose.pose.pose
        actual = msg.pose.pose
        xy_error = math.hypot(
            actual.position.x - expected.position.x,
            actual.position.y - expected.position.y,
        )
        yaw_error = abs(
            _angular_difference(
                _yaw_from_quaternion(actual.orientation),
                _yaw_from_quaternion(expected.orientation),
            )
        )
        if (
            xy_error
            <= self.get_parameter('max_confirmation_xy_error').value
            and yaw_error
            <= self.get_parameter('max_confirmation_yaw_error').value
        ):
            self._confirmed_count += 1
        else:
            self._confirmed_count = 0
            self.get_logger().warning(
                'AMCL response is too far from Initial Pose: '
                f'xy={xy_error:.3f} m, yaw={yaw_error:.3f} rad',
                throttle_duration_sec=2.0,
            )
            return

        if (
            self._published_count >= self._publish_count
            and self._confirmed_count >= self._confirmation_samples
        ):
            self.get_logger().info(
                'AMCL Initial Pose confirmed: '
                f'xy_error={xy_error:.3f} m, '
                f'yaw_error={yaw_error:.3f} rad'
            )
            self.done = True
            self.exit_code = 0

    def _timer_callback(self):
        if self.done:
            return
        if time.monotonic() - self._start_time > self._startup_timeout:
            self.get_logger().error(
                'Timed out waiting for AMCL Initial Pose confirmation'
            )
            self.done = True
            self.exit_code = 2
            return

        self._request_amcl_state()
        if self._initial_pose is None:
            if not self._ready_to_publish():
                return
            self._initial_pose = self._make_initial_pose()
            self._next_publish_time = time.monotonic()

        if (
            self._published_count < self._publish_count
            and time.monotonic() >= self._next_publish_time
        ):
            self._publish_initial_pose()


def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseInitializer()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        exit_code = node.exit_code
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
