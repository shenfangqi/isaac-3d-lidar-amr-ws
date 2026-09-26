"""Convert the incremental nvblox mesh into a bounded voxel snapshot."""

from copy import deepcopy
import math

import numpy as np

from geometry_msgs.msg import Point

from nvblox_msgs.msg import Mesh

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.qos import ReliabilityPolicy

from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from visualization_msgs.msg import Marker


def update_mesh_cache(cache, message):
    """Apply one full or incremental nvblox Mesh message to a block cache."""
    if len(message.block_indices) != len(message.blocks):
        raise ValueError('mesh block indices and blocks must have equal length')
    if message.clear:
        cache.clear()
    for index, block in zip(message.block_indices, message.blocks):
        key = (index.x, index.y, index.z)
        if block.vertices:
            cache[key] = deepcopy(block)
        else:
            cache.pop(key, None)


def mesh_cache_to_voxel_marker(
    cache,
    header,
    block_size_m,
    max_points,
    cube_size_m,
):
    """Build a uniformly thinned CUBE_LIST snapshot from cached mesh blocks."""
    if max_points < 1:
        raise ValueError('max_points must be at least 1')
    if cube_size_m <= 0.0:
        raise ValueError('cube_size_m must be positive')

    total_vertices = sum(len(block.vertices) for block in cache.values())
    stride = max(1, (total_vertices + max_points - 1) // max_points)

    marker = Marker()
    marker.header = header
    # Reuse nvblox's previous namespace so this complete snapshot replaces
    # any stale partial marker that RViz retained across relay restarts.
    marker.ns = 'layer'
    marker.id = 0
    marker.type = Marker.CUBE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = cube_size_m
    marker.scale.y = cube_size_m
    marker.scale.z = cube_size_m
    marker.color.r = 0.10
    marker.color.g = 0.75
    marker.color.b = 1.00
    marker.color.a = 0.65

    source_index = 0
    for block_index in sorted(cache):
        block = cache[block_index]
        origin_x = block_index[0] * block_size_m
        origin_y = block_index[1] * block_size_m
        origin_z = block_index[2] * block_size_m
        for vertex in block.vertices:
            if source_index % stride == 0 and len(marker.points) < max_points:
                marker.points.append(
                    Point(
                        x=origin_x + vertex.x,
                        y=origin_y + vertex.y,
                        z=origin_z + vertex.z,
                    )
                )
            source_index += 1
    return marker, total_vertices


def compact_cube_list_marker(message, max_points):
    """Uniformly thin a TSDF CUBE_LIST marker and preserve point colors."""
    if max_points < 1:
        raise ValueError('max_points must be at least 1')
    output = deepcopy(message)
    point_count = len(message.points)
    if message.type != Marker.CUBE_LIST or point_count <= max_points:
        return output

    stride = (point_count + max_points - 1) // max_points
    indices = range(0, point_count, stride)
    output.points = [message.points[index] for index in indices]
    if len(message.colors) == point_count:
        output.colors = [message.colors[index] for index in indices]
    return output


class MeshVoxelRelay(Node):
    """Cache nvblox mesh blocks and publish a low-bandwidth full snapshot."""

    def __init__(self):
        """Create the mesh cache, publisher, subscription, and timer."""
        super().__init__('nvblox_mesh_voxel_relay')
        self.declare_parameter('input_topic', '/nvblox_node/mesh')
        self.declare_parameter(
            'fallback_input_topic', '/nvblox_node/tsdf_layer_marker'
        )
        self.declare_parameter(
            'output_topic', '/carbot_jetson/nvblox_mesh_voxels'
        )
        self.declare_parameter('publish_period_sec', 2.0)
        self.declare_parameter('max_points', 6000)
        self.declare_parameter('cube_size_m', 0.05)
        self.declare_parameter(
            'cloud_fallback_topic', '/fast_lio/cloud_registered'
        )
        self.declare_parameter('fixed_frame', 'map')
        self.declare_parameter('cloud_voxel_size_m', 0.10)
        self.declare_parameter('cloud_sample_period_sec', 0.5)
        self.declare_parameter('cloud_max_cached_voxels', 120000)
        self.declare_parameter('enable_nvblox_inputs', True)

        input_topic = self.get_parameter('input_topic').value
        fallback_input_topic = self.get_parameter(
            'fallback_input_topic'
        ).value
        output_topic = self.get_parameter('output_topic').value
        period = float(self.get_parameter('publish_period_sec').value)
        self._max_points = int(self.get_parameter('max_points').value)
        self._cube_size_m = float(self.get_parameter('cube_size_m').value)
        cloud_fallback_topic = self.get_parameter(
            'cloud_fallback_topic'
        ).value
        self._fixed_frame = str(self.get_parameter('fixed_frame').value)
        self._cloud_voxel_size_m = float(
            self.get_parameter('cloud_voxel_size_m').value
        )
        self._cloud_sample_period_ns = int(
            float(self.get_parameter('cloud_sample_period_sec').value) * 1e9
        )
        self._cloud_max_cached_voxels = int(
            self.get_parameter('cloud_max_cached_voxels').value
        )
        enable_nvblox_inputs = bool(
            self.get_parameter('enable_nvblox_inputs').value
        )
        if period <= 0.0:
            raise ValueError('publish_period_sec must be positive')
        if self._max_points < 1:
            raise ValueError('max_points must be at least 1')
        if self._cube_size_m <= 0.0:
            raise ValueError('cube_size_m must be positive')
        if self._cloud_voxel_size_m <= 0.0:
            raise ValueError('cloud_voxel_size_m must be positive')
        if self._cloud_sample_period_ns <= 0:
            raise ValueError('cloud_sample_period_sec must be positive')
        if self._cloud_max_cached_voxels < self._max_points:
            raise ValueError(
                'cloud_max_cached_voxels must not be smaller than max_points'
            )

        mesh_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._blocks = {}
        self._header = None
        self._block_size_m = 0.0
        self._dirty = False
        self._latest_layer_marker = None
        self._cloud_voxels = set()
        self._cloud_header = None
        self._last_cloud_stamp_ns = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._logged_first_snapshot = False
        self._publisher = self.create_publisher(
            Marker, output_topic, marker_qos
        )
        self._subscription = None
        self._fallback_subscription = None
        if enable_nvblox_inputs:
            self._subscription = self.create_subscription(
                Mesh, input_topic, self._mesh_callback, mesh_qos
            )
            self._fallback_subscription = self.create_subscription(
                Marker,
                fallback_input_topic,
                self._layer_marker_callback,
                mesh_qos,
            )
        self._cloud_subscription = self.create_subscription(
            PointCloud2,
            cloud_fallback_topic,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self._timer = self.create_timer(period, self._publish_snapshot)
        source_description = (
            f'{input_topic}, {fallback_input_topic}, and '
            f'{cloud_fallback_topic}'
            if enable_nvblox_inputs else
            f'accumulated {cloud_fallback_topic}'
        )
        self.get_logger().info(
            f'Publishing {source_description} at most {1.0 / period:.1f} Hz '
            f'on {output_topic} with {self._max_points} points maximum'
        )

    def _mesh_callback(self, message):
        try:
            update_mesh_cache(self._blocks, message)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)
            return
        self._header = message.header
        self._block_size_m = float(message.block_size_m)
        self._dirty = True

    def _layer_marker_callback(self, message):
        self._latest_layer_marker = message

    @staticmethod
    def _rotation_matrix(quaternion):
        x, y, z, w = (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        return np.asarray([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
             2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
             2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w),
             1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)

    def _cloud_callback(self, message):
        stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        if stamp_ns - self._last_cloud_stamp_ns < self._cloud_sample_period_ns:
            return
        transform = None
        if message.header.frame_id != self._fixed_frame:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._fixed_frame,
                    message.header.frame_id,
                    Time.from_msg(message.header.stamp),
                )
            except TransformException:
                return

        offsets = {field.name: field.offset for field in message.fields}
        if not {'x', 'y', 'z'}.issubset(offsets):
            return
        endian = '>' if message.is_bigendian else '<'
        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [endian + 'f4'] * 3,
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': message.point_step,
        })
        count = message.width * message.height
        cloud = np.frombuffer(message.data, dtype=dtype, count=count)
        points = np.column_stack((cloud['x'], cloud['y'], cloud['z']))
        ranges_squared = np.einsum('ij,ij->i', points, points)
        valid = np.isfinite(points).all(axis=1)
        valid &= ranges_squared >= 0.25
        valid &= ranges_squared <= 100.0
        points = points[valid]
        if not len(points):
            return

        if transform is not None:
            tf = transform.transform
            translation = np.asarray(
                [tf.translation.x, tf.translation.y, tf.translation.z],
                dtype=np.float32,
            )
            points = (
                points @ self._rotation_matrix(tf.rotation).T + translation
            )
        keys = np.floor(points / self._cloud_voxel_size_m).astype(np.int32)
        self._cloud_voxels.update(map(tuple, np.unique(keys, axis=0)))
        if len(self._cloud_voxels) > self._cloud_max_cached_voxels:
            stride = math.ceil(
                len(self._cloud_voxels) / self._cloud_max_cached_voxels
            )
            self._cloud_voxels = set(sorted(self._cloud_voxels)[::stride])
        self._cloud_header = deepcopy(message.header)
        self._cloud_header.frame_id = self._fixed_frame
        self._last_cloud_stamp_ns = stamp_ns

    def _cloud_voxel_marker(self):
        marker = Marker()
        marker.header = self._cloud_header
        marker.ns = 'layer'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._cloud_voxel_size_m
        marker.scale.y = self._cloud_voxel_size_m
        marker.scale.z = self._cloud_voxel_size_m
        marker.color.r = 0.10
        marker.color.g = 0.75
        marker.color.b = 1.00
        marker.color.a = 0.65
        keys = sorted(self._cloud_voxels)
        stride = max(1, math.ceil(len(keys) / self._max_points))
        half = 0.5 * self._cloud_voxel_size_m
        for x, y, z in keys[::stride][:self._max_points]:
            marker.points.append(Point(
                x=x * self._cloud_voxel_size_m + half,
                y=y * self._cloud_voxel_size_m + half,
                z=z * self._cloud_voxel_size_m + half,
            ))
        return marker

    def _publish_snapshot(self):
        if self._header is not None and self._blocks:
            marker, source_vertices = mesh_cache_to_voxel_marker(
                self._blocks,
                self._header,
                self._block_size_m,
                self._max_points,
                self._cube_size_m,
            )
            source_description = (
                f'{source_vertices} mesh vertices from '
                f'{len(self._blocks)} blocks'
            )
        elif self._cloud_voxels:
            source_vertices = len(self._cloud_voxels)
            marker = self._cloud_voxel_marker()
            source_description = f'{source_vertices} accumulated cloud voxels'
        elif self._latest_layer_marker is not None:
            source_vertices = len(self._latest_layer_marker.points)
            marker = compact_cube_list_marker(
                self._latest_layer_marker, self._max_points
            )
            marker.ns = 'layer'
            marker.id = 0
            source_description = f'{source_vertices} TSDF marker points'
        else:
            return
        self._dirty = False
        self._publisher.publish(marker)
        if not self._logged_first_snapshot:
            self.get_logger().info(
                f'First full voxel snapshot contains {len(marker.points)} of '
                f'{source_description}'
            )
            self._logged_first_snapshot = True


def main(args=None):
    """Run the low-bandwidth mesh-to-voxel relay."""
    rclpy.init(args=args)
    node = MeshVoxelRelay()
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
