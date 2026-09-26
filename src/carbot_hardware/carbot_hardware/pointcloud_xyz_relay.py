"""Publish a bandwidth-efficient XYZ-only copy of the MID-360 point cloud."""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


XYZ_NAMES = ('x', 'y', 'z')
LIVOX_FULL_FIELD_NAMES = (
    'x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'
)


def missing_pointcloud_fields(msg, required_fields=LIVOX_FULL_FIELD_NAMES):
    """Return required PointCloud2 fields absent from a source cloud."""
    present = {field.name for field in msg.fields}
    return tuple(name for name in required_fields if name not in present)


def should_publish_frame(frame_index, frame_stride):
    """Return whether an input frame belongs to the output cadence."""
    if frame_index < 0:
        raise ValueError('frame_index must be non-negative')
    if frame_stride < 1:
        raise ValueError('frame_stride must be at least 1')
    return frame_index % frame_stride == 0


def compact_xyz_cloud(msg, point_stride=1, phase=0):
    """Return a packed XYZ-only PointCloud2 while preserving header and shape."""
    if point_stride < 1:
        raise ValueError('point_stride must be at least 1')
    if phase < 0 or phase >= point_stride:
        raise ValueError('phase must be within point_stride')
    point_count = msg.width * msg.height
    if point_count <= 0:
        raise ValueError('PointCloud2 must contain at least one point')
    if msg.row_step != msg.width * msg.point_step:
        raise ValueError('PointCloud2 rows must be tightly packed')
    if len(msg.data) < point_count * msg.point_step:
        raise ValueError('PointCloud2 data buffer is truncated')

    fields = {field.name: field for field in msg.fields}
    for name in XYZ_NAMES:
        field = fields.get(name)
        if field is None:
            raise ValueError(f'PointCloud2 is missing {name}')
        if field.datatype != PointField.FLOAT32 or field.count != 1:
            raise ValueError(f'{name} must be a single FLOAT32 field')
        if field.offset + 4 > msg.point_step:
            raise ValueError(f'{name} lies outside point_step')

    dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
    xyz = np.empty((point_count, 3), dtype=dtype)
    for column, name in enumerate(XYZ_NAMES):
        xyz[:, column] = np.ndarray(
            shape=(point_count,),
            dtype=dtype,
            buffer=msg.data,
            offset=fields[name].offset,
            strides=(msg.point_step,),
        )

    xyz = np.ascontiguousarray(xyz[phase::point_stride])

    output = PointCloud2()
    output.header = msg.header
    output.height = 1
    output.width = xyz.shape[0]
    output.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    output.is_bigendian = msg.is_bigendian
    output.point_step = 12
    output.row_step = output.width * output.point_step
    output.data = xyz.tobytes(order='C')
    output.is_dense = msg.is_dense
    return output


class PointcloudXyzRelay(Node):
    """Strip non-XYZ fields before transmitting MID-360 scans over Wi-Fi."""

    def __init__(self):
        super().__init__('mid360_pointcloud_xyz_relay')
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/mid360/points_xyz')
        self.declare_parameter('point_stride', 4)
        self.declare_parameter('frame_stride', 1)
        self.declare_parameter(
            'required_input_fields_csv', ','.join(LIVOX_FULL_FIELD_NAMES)
        )

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._point_stride = self.get_parameter('point_stride').value
        self._frame_stride = self.get_parameter('frame_stride').value
        self._required_input_fields = tuple(
            name.strip()
            for name in self.get_parameter('required_input_fields_csv')
            .value.split(',')
            if name.strip()
        )
        if not self._required_input_fields:
            raise ValueError('required_input_fields_csv must not be empty')
        if self._point_stride < 1:
            raise ValueError('point_stride must be at least 1')
        if self._frame_stride < 1:
            raise ValueError('frame_stride must be at least 1')
        self._phase = 0
        self._frame_index = 0
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data
        )
        self._subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self._pointcloud_callback,
            qos_profile_sensor_data,
        )
        self._logged_first_message = False
        self.get_logger().info(
            f'Publishing XYZ-only clouds from {input_topic} on {output_topic} '
            f'with point stride {self._point_stride} and frame stride '
            f'{self._frame_stride}'
        )

    def _pointcloud_callback(self, msg):
        frame_index = self._frame_index
        self._frame_index += 1
        if not should_publish_frame(frame_index, self._frame_stride):
            return
        missing = missing_pointcloud_fields(msg, self._required_input_fields)
        if missing:
            self.get_logger().error(
                'Refusing to compact source cloud missing required fields: '
                + ', '.join(missing),
                throttle_duration_sec=2.0,
            )
            return
        try:
            output = compact_xyz_cloud(
                msg, point_stride=self._point_stride, phase=self._phase
            )
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)
            return
        self._phase = (self._phase + 1) % self._point_stride
        self._publisher.publish(output)
        if not self._logged_first_message:
            self.get_logger().info(
                f'First cloud compacted from {msg.width * msg.height} points '
                f'at {msg.point_step} bytes to {output.width} points at '
                f'{output.point_step} bytes'
            )
            self._logged_first_message = True


def main(args=None):
    rclpy.init(args=args)
    node = PointcloudXyzRelay()
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
