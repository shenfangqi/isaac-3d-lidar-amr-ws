"""Anchor FAST-LIO odometry to a manually selected saved-map pose."""

import math

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException
from tf2_ros import TransformListener


def yaw_from_quaternion(quaternion):
    """Return planar yaw from a geometry quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def map_to_odom_from_poses(map_base, odom_base):
    """Return planar map->odom (x, y, yaw) from two base poses."""
    map_x, map_y, map_yaw = map_base
    odom_x, odom_y, odom_yaw = odom_base
    yaw = math.atan2(
        math.sin(map_yaw - odom_yaw),
        math.cos(map_yaw - odom_yaw),
    )
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    rotated_odom_x = cosine * odom_x - sine * odom_y
    rotated_odom_y = sine * odom_x + cosine * odom_y
    return (
        map_x - rotated_odom_x,
        map_y - rotated_odom_y,
        yaw,
    )


class ManualMapLocalizer(Node):
    """Publish map->odom from RViz Initial Pose and stable FAST-LIO odom."""

    def __init__(self):
        super().__init__('manual_map_localizer')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 20.0)

        self._global_frame = str(self.get_parameter('global_frame').value)
        self._odom_frame = str(self.get_parameter('odom_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be positive')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._on_initial_pose,
            10,
        )
        self._timer = self.create_timer(1.0 / publish_rate, self._on_timer)
        self._pending_pose = None
        self._map_to_odom = None
        self._waiting_logged = False

        self.get_logger().warning(
            'Waiting for RViz 2D Pose Estimate; saved-map alignment will use '
            'FAST-LIO odometry without AMCL scan corrections'
        )

    def _on_initial_pose(self, message):
        if message.header.frame_id not in ('', self._global_frame):
            self.get_logger().error(
                f'Rejecting Initial Pose in {message.header.frame_id}; '
                f'expected {self._global_frame}'
            )
            return
        values = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.orientation.x,
            message.pose.pose.orientation.y,
            message.pose.pose.orientation.z,
            message.pose.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('Rejecting non-finite Initial Pose')
            return
        self._pending_pose = message.pose.pose
        self._waiting_logged = False
        self._try_set_alignment()

    def _try_set_alignment(self):
        if self._pending_pose is None:
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self._odom_frame, self._base_frame, Time()
            ).transform
        except TransformException as error:
            if not self._waiting_logged:
                self.get_logger().warning(
                    f'Waiting for {self._odom_frame} <- {self._base_frame}: '
                    f'{error}'
                )
                self._waiting_logged = True
            return

        requested = self._pending_pose
        map_base = (
            requested.position.x,
            requested.position.y,
            yaw_from_quaternion(requested.orientation),
        )
        odom_base = (
            transform.translation.x,
            transform.translation.y,
            yaw_from_quaternion(transform.rotation),
        )
        self._map_to_odom = map_to_odom_from_poses(map_base, odom_base)
        self._pending_pose = None
        self.get_logger().info(
            'Saved-map alignment set: base=(%.3f, %.3f, %.3f), '
            'map->odom=(%.3f, %.3f, %.3f)' % (
                *map_base, *self._map_to_odom
            )
        )

    def _on_timer(self):
        self._try_set_alignment()
        if self._map_to_odom is None:
            return
        x, y, yaw = self._map_to_odom
        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._global_frame
        message.child_frame_id = self._odom_frame
        message.transform.translation.x = x
        message.transform.translation.y = y
        message.transform.rotation.z = math.sin(0.5 * yaw)
        message.transform.rotation.w = math.cos(0.5 * yaw)
        self._tf_broadcaster.sendTransform(message)


def main(args=None):
    rclpy.init(args=args)
    node = ManualMapLocalizer()
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
