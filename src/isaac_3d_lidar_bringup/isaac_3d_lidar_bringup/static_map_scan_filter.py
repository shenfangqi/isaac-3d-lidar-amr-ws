import copy
import math

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from isaac_3d_lidar_bringup.tf_wait_queue import TfWaitQueue


def _yaw(quaternion):
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


def build_static_mask(message, margin_m, occupied_threshold):
    """Return map cells lying on, or close to, a static occupied cell."""
    radius = math.ceil(margin_m / message.info.resolution)
    occupied = {
        (index % message.info.width, index // message.info.width)
        for index, value in enumerate(message.data)
        if value >= occupied_threshold
    }
    masked = set()
    for column, row in occupied:
        for delta_row in range(-radius, radius + 1):
            for delta_column in range(-radius, radius + 1):
                squared_distance = (
                    delta_column * delta_column + delta_row * delta_row
                )
                if squared_distance > radius * radius:
                    continue
                candidate_column = column + delta_column
                candidate_row = row + delta_row
                if (
                    0 <= candidate_column < message.info.width
                    and 0 <= candidate_row < message.info.height
                ):
                    masked.add((candidate_column, candidate_row))
    return masked


def filter_static_returns(scan, map_message, static_mask, transform):
    """Replace scan returns explained by the static map with infinity."""
    output = copy.deepcopy(scan)
    transform_yaw = _yaw(transform.rotation)
    transform_cos = math.cos(transform_yaw)
    transform_sin = math.sin(transform_yaw)
    origin = map_message.info.origin
    origin_yaw = _yaw(origin.orientation)
    origin_cos = math.cos(origin_yaw)
    origin_sin = math.sin(origin_yaw)
    resolution = map_message.info.resolution

    filtered = 0
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance):
            continue
        if distance < scan.range_min or distance > scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        scan_x = distance * math.cos(angle)
        scan_y = distance * math.sin(angle)
        map_x = (
            transform.translation.x
            + transform_cos * scan_x
            - transform_sin * scan_y
        )
        map_y = (
            transform.translation.y
            + transform_sin * scan_x
            + transform_cos * scan_y
        )
        relative_x = map_x - origin.position.x
        relative_y = map_y - origin.position.y
        grid_x = origin_cos * relative_x + origin_sin * relative_y
        grid_y = -origin_sin * relative_x + origin_cos * relative_y
        cell = (
            math.floor(grid_x / resolution),
            math.floor(grid_y / resolution),
        )
        if cell in static_mask:
            output.ranges[index] = float('inf')
            filtered += 1
    return output, filtered


class StaticMapScanFilter(Node):
    """Remove static-map wall returns from the global-planning scan only."""

    def __init__(self):
        super().__init__('static_map_scan_filter')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_global_static_filtered')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('static_margin_m', 0.12)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('transform_wait_sec', 0.5)

        self._map_frame = self.get_parameter('map_frame').value
        self._margin = self.get_parameter('static_margin_m').value
        self._occupied_threshold = self.get_parameter(
            'occupied_threshold'
        ).value
        if self._margin < 0.0:
            raise ValueError('static_margin_m must not be negative')

        self._map = None
        self._static_mask = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._waiting = TfWaitQueue(
            max_wait_s=self.get_parameter('transform_wait_sec').value)
        self._reported_drops = 0

        scan_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        map_topic = self.get_parameter('map_topic').value
        self._publisher = self.create_publisher(
            LaserScan, output_topic, scan_qos
        )
        self._map_subscription = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, map_qos
        )
        self._scan_subscription = self.create_subscription(
            LaserScan, input_topic, self._scan_callback, scan_qos
        )
        self._logged_first_scan = False
        # Scans waiting for TF are also retried between scan arrivals.
        self.create_timer(0.05, self._process_waiting)
        self.get_logger().info(
            f'Filtering static-map returns from {input_topic} onto '
            f'{output_topic} with margin {self._margin:.2f} m'
        )

    def _map_callback(self, message):
        self._map = message
        self._static_mask = build_static_mask(
            message, self._margin, self._occupied_threshold
        )
        self.get_logger().info(
            f'Built static scan mask with {len(self._static_mask)} cells '
            f'from {message.info.width}x{message.info.height} map',
            once=True,
        )

    def _scan_callback(self, message):
        if self._map is None or self._static_mask is None:
            return
        self._waiting.push(
            Time.from_msg(message.header.stamp).nanoseconds * 1e-9,
            message.header.frame_id, message)
        self._process_waiting()

    def _lookup(self, frame_id, stamp_s):
        return self._tf_buffer.lookup_transform(
            self._map_frame, frame_id,
            Time(nanoseconds=int(round(stamp_s * 1e9)),
                 clock_type=self.get_clock().clock_type)).transform

    def _process_waiting(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        for message, transform in self._waiting.pop_ready(
                self._lookup, now_s):
            self._filter(message, transform)
        if self._waiting.dropped != self._reported_drops:
            self.get_logger().warning(
                f'Dropped {self._waiting.dropped} global scans whose TF '
                'never arrived', throttle_duration_sec=2.0)
            self._reported_drops = self._waiting.dropped

    def _filter(self, message, transform):
        output, filtered = filter_static_returns(
            message, self._map, self._static_mask, transform
        )
        self._publisher.publish(output)
        if not self._logged_first_scan:
            self.get_logger().info(
                f'First global scan removed {filtered}/'
                f'{len(message.ranges)} static-map returns'
            )
            self._logged_first_scan = True


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapScanFilter()
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
