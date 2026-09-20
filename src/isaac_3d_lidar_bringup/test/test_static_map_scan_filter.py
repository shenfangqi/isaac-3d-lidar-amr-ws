import math

from geometry_msgs.msg import Transform
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from isaac_3d_lidar_bringup.static_map_scan_filter import (
    build_static_mask,
    filter_static_returns,
)


def _map():
    message = OccupancyGrid()
    message.info.resolution = 0.1
    message.info.width = 10
    message.info.height = 10
    message.info.origin.orientation.w = 1.0
    message.data = [0] * 100
    message.data[5 * 10 + 5] = 100
    return message


def test_static_mask_expands_by_metric_margin():
    mask = build_static_mask(_map(), 0.11, 65)
    assert (5, 5) in mask
    assert (7, 5) in mask
    assert (7, 7) not in mask


def test_filter_removes_static_wall_but_keeps_novel_obstacle():
    map_message = _map()
    mask = build_static_mask(map_message, 0.0, 65)
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = math.pi / 2.0
    scan.range_min = 0.1
    scan.range_max = 10.0
    scan.ranges = [0.55, 0.25]
    transform = Transform()
    transform.translation.y = 0.55
    transform.rotation.w = 1.0

    output, filtered = filter_static_returns(
        scan, map_message, mask, transform
    )

    assert filtered == 1
    assert math.isinf(output.ranges[0])
    assert output.ranges[1] == scan.ranges[1]
