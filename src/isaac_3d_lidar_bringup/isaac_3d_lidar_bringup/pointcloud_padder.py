import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudPadder(Node):
    """Pad a sparse PointCloud2 with NaNs for nvblox's fixed LiDAR model."""

    def __init__(self):
        super().__init__('pointcloud_padder')

        self.declare_parameter('input_topic', '/front_3d_lidar/lidar_points')
        self.declare_parameter(
            'output_topic', '/front_3d_lidar/lidar_points_nvblox'
        )
        self.declare_parameter('target_width', 1800)
        self.declare_parameter('target_height', 31)

        self._target_width = self.get_parameter('target_width').value
        self._target_height = self.get_parameter('target_height').value
        self._target_points = self._target_width * self._target_height
        if self._target_points <= 0:
            raise ValueError('target_width * target_height must be positive')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        output_topic = self.get_parameter('output_topic').value
        input_topic = self.get_parameter('input_topic').value
        self._publisher = self.create_publisher(PointCloud2, output_topic, qos)
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._pointcloud_callback, qos
        )
        self._logged_first_message = False

        self.get_logger().info(
            f'Padding {input_topic} to {self._target_width}x'
            f'{self._target_height} on {output_topic}'
        )

    @staticmethod
    def _xyz_offsets(msg):
        offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z'):
                if field.datatype != PointField.FLOAT32 or field.count != 1:
                    raise ValueError(
                        f'{field.name} must be a single FLOAT32 field'
                    )
                offsets[field.name] = field.offset
        if offsets.keys() != {'x', 'y', 'z'}:
            raise ValueError('PointCloud2 must contain FLOAT32 x, y, and z fields')
        return offsets

    def _pointcloud_callback(self, msg):
        input_points = msg.width * msg.height
        if input_points > self._target_points:
            self.get_logger().error(
                f'Dropping cloud with {input_points} points; fixed nvblox model '
                f'only accepts {self._target_points}',
                throttle_duration_sec=2.0,
            )
            return

        expected_size = input_points * msg.point_step
        if msg.height != 1 or msg.row_step != msg.width * msg.point_step:
            self.get_logger().error(
                'Dropping PointCloud2 with row padding or multiple input rows; '
                'the Isaac Sim adapter expects a packed 1-row cloud',
                throttle_duration_sec=2.0,
            )
            return
        if len(msg.data) < expected_size:
            self.get_logger().error(
                'Dropping truncated PointCloud2 data buffer',
                throttle_duration_sec=2.0,
            )
            return

        try:
            offsets = self._xyz_offsets(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)
            return

        endian = '>' if msg.is_bigendian else '<'
        nan_point = bytearray(msg.point_step)
        for field_name in ('x', 'y', 'z'):
            struct.pack_into(endian + 'f', nan_point, offsets[field_name], float('nan'))

        padding_points = self._target_points - input_points
        output = PointCloud2()
        output.header = msg.header
        output.height = self._target_height
        output.width = self._target_width
        output.fields = msg.fields
        output.is_bigendian = msg.is_bigendian
        output.point_step = msg.point_step
        output.row_step = self._target_width * msg.point_step
        output.data = bytes(msg.data[:expected_size]) + bytes(nan_point) * padding_points
        output.is_dense = False
        self._publisher.publish(output)

        if not self._logged_first_message:
            self.get_logger().info(
                f'First cloud padded from {input_points} to '
                f'{self._target_points} points'
            )
            self._logged_first_message = True


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudPadder()
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
