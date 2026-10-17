"""
Per-beam temporal median of a LaserScan.

Issue #1 on the robot.  The simulation's global scan branch uses
laser_filters' MultiChannelMedianFilterFloat over 5 scans, but the Jetson
navigation image has no laser_filters.  This node does the same: each output
range is the median of that beam over the last ``window`` scans, so a return
must persist for most of the window before novel_obstacle_keepoff sees it.
"""

import collections

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class TemporalMedian:
    """Median of each beam over the last ``window`` equally long scans."""

    def __init__(self, window=5):
        self._scans = collections.deque(maxlen=window)

    def update(self, ranges):
        """Add one scan; return the median ranges once the window is full."""
        ranges = np.asarray(ranges, dtype=np.float32)
        if self._scans and self._scans[0].shape != ranges.shape:
            self._scans.clear()
        self._scans.append(ranges)
        if len(self._scans) < self._scans.maxlen:
            return None
        return np.median(np.stack(self._scans), axis=0)


class ScanTemporalMedian(Node):
    """Publish the temporal median of the input scan."""

    def __init__(self):
        super().__init__('scan_temporal_median')
        self.declare_parameter('input_topic', '/scan_global_static_filtered')
        self.declare_parameter('output_topic', '/scan_global_filtered')
        self.declare_parameter('window', 5)
        self._median = TemporalMedian(self.get_parameter('window').value)
        self._publisher = self.create_publisher(
            LaserScan, self.get_parameter('output_topic').value,
            qos_profile_sensor_data)
        self.create_subscription(
            LaserScan, self.get_parameter('input_topic').value,
            self._scan_callback, qos_profile_sensor_data)

    def _scan_callback(self, message):
        ranges = self._median.update(message.ranges)
        if ranges is None:
            return
        output = LaserScan()
        output.header = message.header
        output.angle_min = message.angle_min
        output.angle_max = message.angle_max
        output.angle_increment = message.angle_increment
        output.time_increment = message.time_increment
        output.scan_time = message.scan_time
        output.range_min = message.range_min
        output.range_max = message.range_max
        output.ranges = ranges.astype(float).tolist()
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ScanTemporalMedian()
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
