"""Conservative finite-ray cell coverage; blind zones remain unknown.

Cells are cleared only if all four corners lie between a pair of adjacent
finite returns and beyond range_min. No interpolation across invalid beams,
infinite ranges or large angular gaps. This is a 2D evidence model; physical
acceptance must establish that the scan covers the robot collision envelope.
"""

import math
import time

from .swept_footprint import ObservedFreeSpaceSnapshot


def scan_visibility(
        scan, sensor_pose, grid, received, *, deadline=None,
        world_bounds=None):
    values = (scan.angle_min, scan.angle_increment, scan.range_min,
              scan.range_max, sensor_pose.x, sensor_pose.y, sensor_pose.yaw)
    if (not all(math.isfinite(v) for v in values)
            or not 0 < scan.angle_increment <= 0.035
            or not 0 < scan.range_min < scan.range_max
            or not 2 <= len(scan.ranges) <= 20000):
        raise ValueError('unsupported scan geometry')
    stamp = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
    cosine, sine = math.cos(sensor_pose.yaw), math.sin(sensor_pose.yaw)
    if world_bounds is None:
        min_mx, min_my, max_mx, max_my = 0, 0, grid.width, grid.height
    else:
        if (len(world_bounds) != 4
                or not all(math.isfinite(v) for v in world_bounds)):
            raise ValueError('invalid visibility bounds')
        min_x, min_y, max_x, max_y = world_bounds
        if min_x > max_x or min_y > max_y:
            raise ValueError('invalid visibility bounds')
        min_mx = max(0, math.floor((min_x-grid.origin_x)/grid.resolution))
        min_my = max(0, math.floor((min_y-grid.origin_y)/grid.resolution))
        max_mx = min(grid.width, math.ceil((max_x-grid.origin_x)/grid.resolution))
        max_my = min(grid.height, math.ceil((max_y-grid.origin_y)/grid.resolution))
    # Cells outside the requested recovery window remain explicitly unknown.
    result = [False] * (grid.width * grid.height)
    for my in range(min_my, max_my):
        if deadline is not None and time.monotonic() > deadline:
            raise ValueError('visibility computation deadline')
        for mx in range(min_mx, max_mx):
            left = grid.origin_x + mx * grid.resolution - sensor_pose.x
            bottom = grid.origin_y + my * grid.resolution - sensor_pose.y
            right, top = left + grid.resolution, bottom + grid.resolution
            near_x = max(left, min(0.0, right))
            near_y = max(bottom, min(0.0, top))
            if math.hypot(near_x, near_y) < scan.range_min:
                continue
            indices, radii = [], []
            for dx, dy in ((left, bottom), (left, top),
                           (right, bottom), (right, top)):
                x, y = dx * cosine + dy * sine, -dx * sine + dy * cosine
                angle = math.atan2(y, x)
                angle += math.ceil((scan.angle_min-angle)/(2*math.pi))*2*math.pi
                indices.append((angle-scan.angle_min)/scan.angle_increment)
                radii.append(math.hypot(x, y))
            first, last = math.floor(min(indices)), math.ceil(max(indices))
            clear = (0 <= first < last < len(scan.ranges)
                     and (last-first)*scan.angle_increment < math.pi)
            if clear:
                # Check ALL intermediate beams. Four clear corners alone can
                # enclose an invalid beam or a shorter obstacle return.
                ranges = scan.ranges[first:last+1]
                clear = all(math.isfinite(r) and scan.range_min <= r <= scan.range_max
                            for r in ranges)
                if clear:
                    limit = min(ranges)*math.cos(scan.angle_increment/2)
                    limit -= grid.resolution*math.sqrt(2)/2
                    clear = max(radii) < limit
            result[my*grid.width+mx] = clear
    return ObservedFreeSpaceSnapshot(
        grid.width, grid.height, grid.resolution, grid.origin_x, grid.origin_y,
        tuple(result), grid.frame_id, stamp, received)
