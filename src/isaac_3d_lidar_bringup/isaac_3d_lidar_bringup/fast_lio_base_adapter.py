"""Adapt FAST-LIO2's full IMU pose to the planar Carbot navigation frame."""

import math

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.time import Time
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
        self._base_in_imu = None
        self._waiting_logged = False

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
            self.get_logger().error('rejecting non-finite FAST-LIO2 pose')
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
