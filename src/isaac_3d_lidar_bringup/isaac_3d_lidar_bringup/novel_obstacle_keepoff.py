"""
Keep global detours far enough from novel obstacles for the LiDAR to see them.

Issue #1.  The MID-360 /scan has a 0.5 m minimum range and sees only the
near face of an obstacle that is absent from the saved map.  A global path
that hugs that face cuts through the unseen depth, which then stays inside
the blind zone while the robot passes.  This node republishes the saved map
with a keep-off added, for a second global-costmap StaticLayer that
overwrites the master grid:

* a novel return (static-map returns already removed upstream) becomes
  evidence after ``confirm_hits`` scans and is extended ``assumed_depth_m``
  along its beam because the far side is unobserved;
* every confirmed or assumed cell, plus ``keepoff_radius_m`` around it, is
  lethal, so a detour keeps the obstacle outside the blind zone or fails;
* a disk of ``robot_clear_radius_m`` around the robot stays free (outside the
  evidence itself) so the stopped robot can plan its way out of the ring;
* the raw /scan clears evidence its beams pass through.  Returns inside the
  minimum range are inf and clear nothing, so an obstacle the robot stopped
  at stays marked;
* scans are used only while the robot is not turning: an in-place turn
  smears a 0.1 s scan by several degrees, and the five-frame median keeps
  those frames for another half second, so wall returns land beyond the
  static-map mask and would be taken for novel obstacles.
"""

import math

import numpy as np
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def disk_offsets(radius_cells):
    """Return (row, column) offsets inside a disk of the given radius."""
    span = np.arange(-radius_cells, radius_cells + 1)
    rows, columns = np.meshgrid(span, span, indexing='ij')
    inside = rows * rows + columns * columns <= radius_cells * radius_cells
    return list(zip(rows[inside].tolist(), columns[inside].tolist()))


def dilate(mask, offsets):
    """Return the boolean dilation of mask by the given offsets."""
    height, width = mask.shape
    out = np.zeros_like(mask)
    for row, column in offsets:
        source = mask[max(0, -row):height - max(0, row),
                      max(0, -column):width - max(0, column)]
        out[max(0, row):max(0, row) + source.shape[0],
            max(0, column):max(0, column) + source.shape[1]] |= source
    return out


class KeepoffGrid:
    """Novel-obstacle evidence on the saved map's grid."""

    def __init__(self, width, height, resolution, origin_x, origin_y,
                 assumed_depth_m=0.4, keepoff_radius_m=0.5,
                 robot_clear_radius_m=0.3, confirm_hits=3, max_hits=10,
                 max_range_m=10.0, mark_max_range_m=4.0):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.assumed_depth_m = assumed_depth_m
        self.confirm_hits = confirm_hits
        self.max_hits = max_hits
        self.max_range_m = max_range_m
        # Far wall returns miss the static mask by small angular errors and
        # would read as novel; detours only concern nearby obstacles.
        self.mark_max_range_m = min(mark_max_range_m, max_range_m)
        self.hits = np.zeros((height, width), dtype=np.int16)
        # Latest beam direction of each hit cell.
        self.direction = np.zeros((height, width, 2), dtype=np.float32)
        # Cells a raw beam has passed through since they were last hit.
        self.seen_free = np.zeros((height, width), dtype=bool)
        self.keepoff_offsets = disk_offsets(
            int(math.ceil(keepoff_radius_m / resolution)))
        self.robot_clear_cells = robot_clear_radius_m / resolution

    def _cells(self, xs, ys):
        columns = np.floor((xs - self.origin_x) / self.resolution).astype(int)
        rows = np.floor((ys - self.origin_y) / self.resolution).astype(int)
        valid = ((columns >= 0) & (columns < self.width)
                 & (rows >= 0) & (rows < self.height))
        return rows[valid], columns[valid], valid

    def _sample_step(self):
        return self.resolution / 4.0

    def mark(self, origin, yaw, angles, ranges):
        """Count finite novel returns and remember their beam direction."""
        ranges = np.asarray(ranges, dtype=float)
        finite = np.isfinite(ranges) & (ranges <= self.mark_max_range_m)
        if not finite.any():
            return
        world = yaw + np.asarray(angles, dtype=float)[finite]
        cos, sin = np.cos(world), np.sin(world)
        x0, y0 = origin
        hit = ranges[finite]
        rows, columns, valid = self._cells(x0 + hit * cos, y0 + hit * sin)
        cells, first_index = np.unique(
            np.stack([rows, columns]), axis=1, return_index=True)
        rows, columns = cells
        self.hits[rows, columns] = np.minimum(
            self.hits[rows, columns] + 1, self.max_hits)
        self.seen_free[rows, columns] = False
        self.direction[rows, columns, 0] = cos[valid][first_index]
        self.direction[rows, columns, 1] = sin[valid][first_index]

    def clear(self, origin, yaw, angles, ranges):
        """Clear evidence that finite raw beams pass through."""
        ranges = np.asarray(ranges, dtype=float)
        finite = np.isfinite(ranges)
        if not finite.any():
            return
        world = yaw + np.asarray(angles, dtype=float)[finite]
        cos, sin = np.cos(world), np.sin(world)
        x0, y0 = origin
        # Stop one cell short of the return so the surface itself survives.
        reach = np.minimum(ranges[finite], self.max_range_m) - self.resolution
        steps = np.arange(0.0, self.max_range_m, self._sample_step())
        inside = steps[None, :] < reach[:, None]
        distance = np.broadcast_to(steps, inside.shape)[inside]
        beam = np.nonzero(inside)[0]
        rows, columns, _ = self._cells(
            x0 + distance * cos[beam], y0 + distance * sin[beam])
        self.hits[rows, columns] = 0
        self.seen_free[rows, columns] = True

    def confirmed(self):
        """Return returns seen in at least ``confirm_hits`` scans."""
        return self.hits >= self.confirm_hits

    def evidence(self):
        """Return confirmed returns plus their assumed depth behind them."""
        confirmed = self.confirmed()
        evidence = confirmed.copy()
        rows, columns = np.nonzero(confirmed)
        if rows.size == 0:
            return evidence
        x = self.origin_x + (columns + 0.5) * self.resolution
        y = self.origin_y + (rows + 0.5) * self.resolution
        cos = self.direction[rows, columns, 0]
        sin = self.direction[rows, columns, 1]
        steps = np.arange(self._sample_step(), self.assumed_depth_m + 1e-9,
                          self._sample_step())
        depth_rows, depth_columns, _ = self._cells(
            (x[:, None] + steps[None, :] * cos[:, None]).ravel(),
            (y[:, None] + steps[None, :] * sin[:, None]).ravel())
        keep = ~self.seen_free[depth_rows, depth_columns]
        evidence[depth_rows[keep], depth_columns[keep]] = True
        return evidence

    def render(self, robot_xy=None, base=None):
        """
        Return OccupancyGrid data (row-major) with the keep-off lethal.

        With ``base`` (the saved map's data) every other cell keeps its map
        value, so a second StaticLayer can overwrite the master grid with
        the saved map plus the keep-off.  Humble's use_maximum is
        costmap-wide and would stop the first StaticLayer from writing the
        map's unknown cells.  Without ``base`` other cells are unknown.
        """
        evidence = self.evidence()
        lethal = dilate(evidence, self.keepoff_offsets)
        if robot_xy is not None:
            rows, columns = np.ogrid[0:self.height, 0:self.width]
            robot_column = (robot_xy[0] - self.origin_x) / self.resolution
            robot_row = (robot_xy[1] - self.origin_y) / self.resolution
            near_robot = ((columns + 0.5 - robot_column) ** 2
                          + (rows + 0.5 - robot_row) ** 2
                          <= self.robot_clear_cells ** 2)
            lethal &= ~(near_robot & ~evidence)
        if base is None:
            base = np.full(lethal.shape, -1, dtype=np.int8)
        return np.where(lethal, 100, base).astype(np.int8)


class TurnGate:
    """Accept scans only after the robot has stopped turning for a while."""

    def __init__(self, max_yaw_rate, settle_s):
        self.max_yaw_rate = max_yaw_rate
        self.settle_s = settle_s
        self.last_turning_s = None

    def observe(self, time_s, yaw_rate):
        if abs(yaw_rate) > self.max_yaw_rate:
            self.last_turning_s = time_s

    def accepts(self, time_s):
        return (self.last_turning_s is not None
                and time_s - self.last_turning_s >= self.settle_s) or (
                    self.last_turning_s is None)


class NovelObstacleKeepoff(Node):
    """Publish the keep-off grid for the global costmap."""

    def __init__(self):
        super().__init__('novel_obstacle_keepoff')
        self.declare_parameter('marking_scan_topic', '/scan_global_filtered')
        self.declare_parameter('clearing_scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', '/novel_obstacle_keepoff')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('assumed_depth_m', 0.4)
        self.declare_parameter('keepoff_radius_m', 0.5)
        self.declare_parameter('robot_clear_radius_m', 0.3)
        self.declare_parameter('confirm_hits', 3)
        self.declare_parameter('mark_max_range_m', 4.0)
        self.declare_parameter('max_range_m', 10.0)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('transform_timeout_sec', 0.2)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('max_yaw_rate_rad_s', 0.1)
        self.declare_parameter('turn_settle_s', 0.6)
        self._turn_gate = TurnGate(
            self.get_parameter('max_yaw_rate_rad_s').value,
            self.get_parameter('turn_settle_s').value)
        self._odom_seen = False

        self._grid = None
        self._map_header = None
        self._map_info = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            OccupancyGrid, self.get_parameter('output_topic').value, latched)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self._map_callback, latched)
        self.create_subscription(
            LaserScan, self.get_parameter('marking_scan_topic').value,
            lambda message: self._scan_callback(message, mark=True),
            scan_qos)
        self.create_subscription(
            LaserScan, self.get_parameter('clearing_scan_topic').value,
            lambda message: self._scan_callback(message, mark=False),
            scan_qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._odom_callback, 10)
        self.create_timer(
            1.0 / self.get_parameter('publish_rate_hz').value, self._publish)

    def _odom_callback(self, message):
        self._odom_seen = True
        self._turn_gate.observe(
            Time.from_msg(message.header.stamp).nanoseconds * 1e-9,
            message.twist.twist.angular.z)

    def _map_callback(self, message):
        info = message.info
        self._grid = KeepoffGrid(
            info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y,
            assumed_depth_m=self.get_parameter('assumed_depth_m').value,
            keepoff_radius_m=self.get_parameter('keepoff_radius_m').value,
            robot_clear_radius_m=self.get_parameter(
                'robot_clear_radius_m').value,
            confirm_hits=self.get_parameter('confirm_hits').value,
            max_range_m=self.get_parameter('max_range_m').value,
            mark_max_range_m=self.get_parameter('mark_max_range_m').value,
        )
        self._map_header = message.header
        self._map_info = info
        self._map_data = np.asarray(message.data, dtype=np.int8).reshape(
            info.height, info.width)
        self.get_logger().info(
            f'Keep-off grid {info.width}x{info.height} at '
            f'{info.resolution:.3f} m', once=True)

    def _lookup(self, frame, stamp):
        return self._tf_buffer.lookup_transform(
            self._map_frame, frame, stamp,
            timeout=Duration(seconds=self.get_parameter(
                'transform_timeout_sec').value)).transform

    def _scan_callback(self, message, mark):
        if self._grid is None or not self._odom_seen:
            return
        if not self._turn_gate.accepts(
                Time.from_msg(message.header.stamp).nanoseconds * 1e-9):
            return
        try:
            transform = self._lookup(
                message.header.frame_id, Time.from_msg(message.header.stamp))
        except TransformException as exc:
            self.get_logger().warning(
                f'Keep-off scan dropped without TF: {exc}',
                throttle_duration_sec=2.0)
            return
        angles = (message.angle_min
                  + np.arange(len(message.ranges)) * message.angle_increment)
        origin = (transform.translation.x, transform.translation.y)
        yaw = _yaw(transform.rotation)
        if mark:
            self._grid.mark(origin, yaw, angles, message.ranges)
        else:
            self._grid.clear(origin, yaw, angles, message.ranges)

    def _publish(self):
        if self._grid is None:
            return
        robot_xy = None
        try:
            translation = self._lookup(self._base_frame, Time()).translation
            robot_xy = (translation.x, translation.y)
        except TransformException:
            pass
        message = OccupancyGrid()
        message.header.frame_id = self._map_header.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.info = self._map_info
        message.data = self._grid.render(
            robot_xy, self._map_data).ravel().tolist()
        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = NovelObstacleKeepoff()
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
