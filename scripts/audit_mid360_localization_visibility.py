#!/usr/bin/env python3
"""Read-only MID-360 near-field visibility audit for Issue #13 (PR0).

Two independent parts:

* ``geometry``: analytic coverage of the in-place rotation sweep from the
  canonical robot parameters, the MID-360 vertical field of view and every
  configured range/height filter.  Needs no bag and no ROS.
* ``bag``: per-message statistics from a recorded rosbag2 (needs rosbag2_py,
  so run it in the ROS container).  It reports the actual message types, the
  fraction of returns removed by range/height/self filters, source ages, TF
  failures at source time, and UNKNOWN / OBSERVED_OCCUPIED / OBSERVED_FREE
  counts for the rotation sweep annulus.

Nothing here commands motion.  The absence of near returns is never turned
into free space: infinite ranges, beams below range_min and gaps between
sparse 3D points all stay UNKNOWN.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PARAMETERS = ROOT / 'src/carbot_description/config/carbot_parameters.yaml'
FAST_LIO_CONFIG = (
    ROOT / 'src/isaac_3d_lidar_bringup/config/state_estimation/'
    'fast_lio_mid360.yaml')

UNKNOWN = 'UNKNOWN'
OBSERVED_OCCUPIED = 'OBSERVED_OCCUPIED'
OBSERVED_FREE = 'OBSERVED_FREE'
SELF = 'SELF'

# Filters configured in carbot_navigation_real.launch.py at the PR0 baseline.
# The bag mode prefers parameter dumps stored beside the bag and reports the
# source it used.
LAUNCH_SCAN_FILTERS = {
    '/scan': {'min_height': 0.10, 'max_height': 0.35,
              'range_min': 0.5, 'range_max': 20.0,
              'target_frame': 'base_footprint'},
    '/scan_localization': {'min_height': 0.22, 'max_height': 0.35,
                           'range_min': 0.5, 'range_max': 20.0,
                           'target_frame': 'base_footprint'},
}
NEAR_FIELD_BINS_M = tuple(round(0.05 * i, 2) for i in range(0, 21))


def load_robot_geometry(parameters_path=PARAMETERS):
    """Return the geometry needed by the audit from canonical parameters."""
    parameters = yaml.safe_load(Path(parameters_path).read_text('utf-8'))
    geometry = parameters['geometry']
    mid360 = parameters['sensors']['mid360']
    specs = mid360['manufacturer_specs']
    # Same expression as carbot.urdf.xacro: housing bottom + origin offset.
    origin_z = (mid360['housing_top_height_from_ground_m']
                - mid360['housing_height_m']
                + mid360['coordinate_origin_height_from_housing_bottom_m'])
    body = geometry['body_collision']
    body_bottom = (geometry['base_link_height_m']
                   + body['position_from_base_link_m'][2]
                   - body['size_m'][2] / 2.0)
    return {
        'footprint': [tuple(point) for point in geometry['footprint_m']],
        'body_z_band_m': (max(0.0, body_bottom),
                          geometry['overall_size_m'][2]),
        'sensor_origin_m': (
            mid360['mount_translation_xy_from_base_link_m'][0],
            mid360['mount_translation_xy_from_base_link_m'][1],
            origin_z),
        'vertical_fov_deg': tuple(specs['vertical_fov_deg']),
        'minimum_detection_range_m': specs['minimum_detection_range_m'],
        'declared_pointcloud_type': mid360['pointcloud_type'],
    }


def load_fast_lio_preprocess(config_path=FAST_LIO_CONFIG):
    """Return the FAST-LIO preprocessing filter that feeds derived scans."""
    config = yaml.safe_load(Path(config_path).read_text('utf-8'))
    parameters = config['/**']['ros__parameters']
    lidar_type = parameters['preprocess']['lidar_type']
    return {
        'lid_topic': parameters['common']['lid_topic'],
        'lidar_type': lidar_type,
        # FAST-LIO enum: 1=AVIA (livox_ros_driver2 CustomMsg subscriber).
        'expected_message_type': (
            'livox_ros_driver2/msg/CustomMsg' if lidar_type == 1
            else 'sensor_msgs/msg/PointCloud2'),
        'blind_m': parameters['preprocess']['blind'],
        'point_filter_num': parameters['point_filter_num'],
        'body_frame': parameters['common']['body_frame'],
    }


def footprint_boundary_distance(footprint, angle):
    """Distance from the base origin to the footprint edge along ``angle``."""
    dx, dy = math.cos(angle), math.sin(angle)
    best = math.inf
    for index, (ax, ay) in enumerate(footprint):
        bx, by = footprint[(index + 1) % len(footprint)]
        ex, ey = bx - ax, by - ay
        denominator = dx * ey - dy * ex
        if abs(denominator) < 1e-12:
            continue
        t = (ax * ey - ay * ex) / denominator
        s = (ax * dy - ay * dx) / denominator
        if t >= 0.0 and -1e-12 <= s <= 1.0 + 1e-12:
            best = min(best, t)
    if not math.isfinite(best):
        raise ValueError('base origin is outside the footprint')
    return best


def rotation_envelope_radius(footprint, padding_m=0.0):
    """Outer radius swept by the padded footprint over a full turn."""
    if padding_m < 0.0:
        raise ValueError('padding must be non-negative')
    return max(math.hypot(x, y) for x, y in footprint) + padding_m


def observable_height_interval(sensor_z, horizontal_m, vertical_fov_deg):
    """Heights visible at a horizontal distance from the sensor axis."""
    if horizontal_m < 0.0:
        raise ValueError('horizontal distance must be non-negative')
    low, high = vertical_fov_deg
    return (sensor_z + horizontal_m * math.tan(math.radians(low)),
            sensor_z + horizontal_m * math.tan(math.radians(high)))


def observable_band(sensor_z, horizontal_m, vertical_fov_deg, min_range_m,
                    z_min=-math.inf, z_max=math.inf):
    """Intersect FOV, minimum 3D range and a height filter at one distance.

    Returns a list of disjoint ``(low, high)`` height intervals.  A 3D range
    limit removes the heights closest to the sensor, which can split the
    field-of-view interval in two.
    """
    low, high = observable_height_interval(
        sensor_z, horizontal_m, vertical_fov_deg)
    low, high = max(low, z_min), min(high, z_max)
    if low >= high:
        return []
    if min_range_m <= horizontal_m:
        return [(low, high)]
    hidden = math.sqrt(min_range_m ** 2 - horizontal_m ** 2)
    intervals = []
    if low < sensor_z - hidden:
        intervals.append((low, min(high, sensor_z - hidden)))
    if high > sensor_z + hidden:
        intervals.append((max(low, sensor_z + hidden), high))
    return [(a, b) for a, b in intervals if a < b]


def unobservable_fraction(intervals, band):
    """Fraction of ``band`` not covered by the observable intervals."""
    low, high = band
    covered = sum(max(0.0, min(b, high) - max(a, low))
                  for a, b in intervals)
    return max(0.0, 1.0 - covered / (high - low))


def analytic_sweep_coverage(geometry, fast_lio, padding_m=0.05,
                            samples=6, scan_filters=None):
    """Coverage of the collision height band across the sweep annulus.

    The 2D scans measure range from base_footprint in the projected plane,
    after FAST-LIO already removed returns within ``blind`` of the sensor in
    3D.  The raw channel keeps only the sensor's own physical limits.
    """
    scan_filters = scan_filters or LAUNCH_SCAN_FILTERS
    footprint = geometry['footprint']
    sensor_x, sensor_y, sensor_z = geometry['sensor_origin_m']
    sensor_offset = math.hypot(sensor_x, sensor_y)
    inner = min(footprint_boundary_distance(footprint, 2.0 * math.pi * k / 72)
                for k in range(72))
    outer = rotation_envelope_radius(footprint, padding_m)
    band = geometry['body_z_band_m']
    radii = [inner + (outer - inner) * i / max(1, samples - 1)
             for i in range(samples)]
    channels = {
        'raw_mid360': {
            'min_range_m': geometry['minimum_detection_range_m'],
            'z_min': -math.inf, 'z_max': math.inf, 'planar_range_min': 0.0,
        },
        'fast_lio_cloud_registered_body': {
            'min_range_m': max(geometry['minimum_detection_range_m'],
                               fast_lio['blind_m']),
            'z_min': -math.inf, 'z_max': math.inf, 'planar_range_min': 0.0,
        },
    }
    for topic, config in scan_filters.items():
        channels[topic] = {
            'min_range_m': max(geometry['minimum_detection_range_m'],
                               fast_lio['blind_m']),
            'z_min': config['min_height'], 'z_max': config['max_height'],
            'planar_range_min': config['range_min'],
        }
    report = {
        'sweep_inner_radius_m': round(inner, 4),
        'sweep_outer_radius_m': round(outer, 4),
        'padding_m': padding_m,
        'collision_z_band_m': [round(v, 4) for v in band],
        'sensor_origin_m': [round(v, 4) for v in geometry['sensor_origin_m']],
        'vertical_fov_deg': list(geometry['vertical_fov_deg']),
        'channels': {},
    }
    for name, channel in channels.items():
        rows = []
        for radius in radii:
            # Worst case over azimuth: the sensor is offset from the centre.
            worst = None
            for horizontal in (max(0.0, radius - sensor_offset),
                               radius + sensor_offset):
                if radius < channel['planar_range_min']:
                    intervals = []
                else:
                    intervals = observable_band(
                        sensor_z, horizontal, geometry['vertical_fov_deg'],
                        channel['min_range_m'], channel['z_min'],
                        channel['z_max'])
                missing = unobservable_fraction(intervals, band)
                if worst is None or missing > worst[0]:
                    worst = (missing, intervals)
            rows.append({
                'radius_m': round(radius, 4),
                'observable_z_m': [[round(a, 4), round(b, 4)]
                                   for a, b in worst[1]],
                'unobservable_fraction_of_band': round(worst[0], 4),
            })
        complete = all(row['unobservable_fraction_of_band'] == 0.0
                       for row in rows)
        report['channels'][name] = {
            'filters': {k: (None if not math.isfinite(v) else v)
                        for k, v in channel.items()},
            'radii': rows,
            # Only full band coverage at every radius could ever support a
            # free-space claim; partial coverage still leaves UNKNOWN cells.
            'can_prove_sweep_free': complete,
        }
    return report


def classify_scan_sweep(ranges, angle_min, angle_increment, range_min,
                        range_max, footprint, outer_radius):
    """Classify every beam's sweep-annulus segment.

    The scan origin must be base_footprint (the launch target_frame).  A
    beam supports OBSERVED_FREE only when it is valid from inside the body
    edge to beyond the sweep: a finite return farther than ``outer_radius``
    and a ``range_min`` not larger than the body edge on that bearing.
    """
    counts = {UNKNOWN: 0, OBSERVED_OCCUPIED: 0, OBSERVED_FREE: 0, SELF: 0}
    reasons = {'nonfinite_or_out_of_range': 0, 'range_min_hides_sweep': 0}
    for index, value in enumerate(ranges):
        angle = angle_min + index * angle_increment
        inner = footprint_boundary_distance(footprint, angle)
        valid = math.isfinite(value) and range_min <= value <= range_max
        if not valid:
            counts[UNKNOWN] += 1
            reasons['nonfinite_or_out_of_range'] += 1
        elif value < inner:
            counts[SELF] += 1
        elif value <= outer_radius:
            counts[OBSERVED_OCCUPIED] += 1
        elif range_min <= inner:
            counts[OBSERVED_FREE] += 1
        else:
            counts[UNKNOWN] += 1
            reasons['range_min_hides_sweep'] += 1
    return counts, reasons


def point_in_polygon(x, y, polygon):
    """Even-odd point-in-polygon test."""
    inside = False
    for index, (ax, ay) in enumerate(polygon):
        bx, by = polygon[(index + 1) % len(polygon)]
        if (ay > y) != (by > y):
            if x < ax + (y - ay) * (bx - ax) / (by - ay):
                inside = not inside
    return inside


def classify_cloud_points(points_base, footprint, outer_radius, z_band):
    """Count 3D returns that fall in the sweep volume.

    Point clouds only provide obstacle presence; empty space between sparse
    points is never reported as free.
    """
    counts = {OBSERVED_OCCUPIED: 0, SELF: 0, 'outside_sweep': 0}
    for x, y, z in points_base:
        if not z_band[0] <= z <= z_band[1]:
            counts['outside_sweep'] += 1
        elif point_in_polygon(x, y, footprint):
            counts[SELF] += 1
        elif math.hypot(x, y) <= outer_radius:
            counts[OBSERVED_OCCUPIED] += 1
        else:
            counts['outside_sweep'] += 1
    return counts


def histogram(values, edges):
    """Counts per [edge_i, edge_i+1) bin plus an overflow bin."""
    counts = [0] * len(edges)
    for value in values:
        for index in range(len(edges) - 1):
            if edges[index] <= value < edges[index + 1]:
                counts[index] += 1
                break
        else:
            if value >= edges[-1]:
                counts[-1] += 1
    return {f'{edges[i]:.2f}': counts[i] for i in range(len(edges))}


def percentile(values, fraction):
    """Nearest-rank percentile or None for no data."""
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, round((len(ordered) - 1) * fraction))]


def summarize(values):
    """Compact distribution summary."""
    if not values:
        return {'count': 0}
    return {'count': len(values), 'min': min(values),
            'p50': percentile(values, 0.5), 'p95': percentile(values, 0.95),
            'p99': percentile(values, 0.99), 'max': max(values)}


def sidecar_parameter_dumps(bag_path):
    """Return ROS parameter dumps saved next to an evidence bag."""
    directory = Path(bag_path)
    directory = directory.parent if directory.name == 'bag' else directory
    dumps = {}
    for path in sorted(directory.glob('*.yaml')):
        if path.name == 'metadata.yaml':
            continue
        try:
            data = yaml.safe_load(path.read_text('utf-8'))
        except yaml.YAMLError:
            continue
        if isinstance(data, dict) and any(
                isinstance(v, dict) and 'ros__parameters' in v
                for v in data.values()):
            dumps[path.name] = {
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'parameters': data,
            }
    return dumps


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _cloud_xyz(message):
    """Yield (x, y, z, extras) from a PointCloud2 without numpy."""
    import struct
    fields = {field.name: field for field in message.fields}
    endian = '>' if message.is_bigendian else '<'
    offsets = [fields[name].offset for name in ('x', 'y', 'z')]
    tag = fields.get('tag')
    count = message.width * message.height
    data = bytes(message.data)
    for index in range(count):
        base = index * message.point_step
        x, y, z = (struct.unpack_from(endian + 'f', data, base + o)[0]
                   for o in offsets)
        extra = data[base + tag.offset] if tag is not None else None
        yield x, y, z, extra


def _custom_xyz(message):
    for point in message.points:
        yield point.x, point.y, point.z, point.tag


def _transform_point(transform, point):
    """Apply a geometry_msgs/Transform to a 3D point."""
    t, q = transform.translation, transform.rotation
    x, y, z = point
    # Rotate by quaternion: v' = v + 2w(q x v) + 2 q x (q x v)
    cx = q.y * z - q.z * y
    cy = q.z * x - q.x * z
    cz = q.x * y - q.y * x
    dx = q.y * cz - q.z * cy
    dy = q.z * cx - q.x * cz
    dz = q.x * cy - q.y * cx
    return (x + 2.0 * (q.w * cx + dx) + t.x,
            y + 2.0 * (q.w * cy + dy) + t.y,
            z + 2.0 * (q.w * cz + dz) + t.z)


def audit_bag(bag_path, geometry, fast_lio, padding_m, cloud_stride,
              base_frame='base_footprint'):
    """Stream a bag once for TF, then again for sensor statistics."""
    import rosbag2_py
    from rclpy.duration import Duration
    from rclpy.serialization import deserialize_message
    from rclpy.time import Time
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer, TransformException

    metadata = yaml.safe_load(
        (Path(bag_path) / 'metadata.yaml').read_text('utf-8'))
    storage_id = metadata['rosbag2_bagfile_information'][
        'storage_identifier']

    def open_reader():
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path),
                                      storage_id=storage_id),
            rosbag2_py.ConverterOptions('cdr', 'cdr'))
        return reader

    reader = open_reader()
    declared = {item.name: item.type
                for item in reader.get_all_topics_and_types()}
    types = {}
    unavailable = {}
    for topic, type_name in declared.items():
        try:
            types[topic] = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as error:
            unavailable[topic] = f'{type_name}: {error}'

    first_ns = last_ns = None
    buffer = Buffer(cache_time=Duration(seconds=24 * 3600))
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        first_ns = bag_ns if first_ns is None else first_ns
        last_ns = bag_ns
        if topic in ('/tf', '/tf_static') and topic in types:
            message = deserialize_message(data, types[topic])
            for transform in message.transforms:
                if topic == '/tf_static':
                    buffer.set_transform_static(transform, 'bag')
                else:
                    buffer.set_transform(transform, 'bag')
    del reader

    footprint = geometry['footprint']
    outer = rotation_envelope_radius(footprint, padding_m)
    band = geometry['body_z_band_m']
    scans = {}
    clouds = {}
    cloud_topics = ('/livox/lidar', '/fast_lio/cloud_registered_body')
    cloud_index = {}
    reader = open_reader()
    while reader.has_next():
        topic, data, bag_ns = reader.read_next()
        if topic not in types:
            continue
        type_name = declared[topic]
        if type_name == 'sensor_msgs/msg/LaserScan':
            message = deserialize_message(data, types[topic])
            entry = scans.setdefault(topic, {
                'type': type_name, 'frames': set(), 'messages': 0,
                'receipt_minus_source_sec': [], 'tf_failures': 0,
                'range_classes': {'finite_in_range': 0, 'below_range_min': 0,
                                  'positive_infinity': 0, 'nan': 0,
                                  'above_range_max': 0},
                'finite_ranges': [],
                'sweep': {UNKNOWN: 0, OBSERVED_OCCUPIED: 0,
                          OBSERVED_FREE: 0, SELF: 0},
                'sweep_unknown_reasons': {'nonfinite_or_out_of_range': 0,
                                          'range_min_hides_sweep': 0},
                'scan_range_min': set(), 'scan_range_max': set(),
            })
            entry['messages'] += 1
            entry['frames'].add(message.header.frame_id)
            entry['scan_range_min'].add(round(message.range_min, 4))
            entry['scan_range_max'].add(round(message.range_max, 4))
            source_ns = _stamp_ns(message.header.stamp)
            entry['receipt_minus_source_sec'].append(
                (bag_ns - source_ns) / 1e9)
            try:
                buffer.lookup_transform(
                    'odom', message.header.frame_id,
                    Time(nanoseconds=source_ns))
            except TransformException:
                entry['tf_failures'] += 1
            classes = entry['range_classes']
            for value in message.ranges:
                if math.isnan(value):
                    classes['nan'] += 1
                elif value == math.inf:
                    classes['positive_infinity'] += 1
                elif value < message.range_min:
                    classes['below_range_min'] += 1
                elif value > message.range_max:
                    classes['above_range_max'] += 1
                else:
                    classes['finite_in_range'] += 1
                    if value < 1.0:
                        entry['finite_ranges'].append(value)
            if message.header.frame_id == base_frame:
                counts, reasons = classify_scan_sweep(
                    message.ranges, message.angle_min,
                    message.angle_increment, message.range_min,
                    message.range_max, footprint, outer)
                for key, value in counts.items():
                    entry['sweep'][key] += value
                for key, value in reasons.items():
                    entry['sweep_unknown_reasons'][key] += value
        elif topic in cloud_topics:
            cloud_index[topic] = cloud_index.get(topic, 0) + 1
            entry = clouds.setdefault(topic, {
                'type': type_name, 'frames': set(), 'messages': 0,
                'sampled_messages': 0, 'receipt_minus_source_sec': [],
                'tf_attempts': 0, 'tf_failures': 0, 'points': 0, 'zero_points': 0,
                'sensor_range_lt_0p1': 0, 'sensor_range_lt_blind': 0,
                'near_horizontal_ranges': [], 'near_heights': [],
                'tags': {}, 'sweep': {OBSERVED_OCCUPIED: 0, SELF: 0,
                                      'outside_sweep': 0},
            })
            entry['messages'] += 1
            message = deserialize_message(data, types[topic])
            entry['frames'].add(message.header.frame_id)
            source_ns = _stamp_ns(message.header.stamp)
            entry['receipt_minus_source_sec'].append(
                (bag_ns - source_ns) / 1e9)
            if (cloud_index[topic] - 1) % cloud_stride:
                continue
            entry['tf_attempts'] += 1
            try:
                transform = buffer.lookup_transform(
                    base_frame, message.header.frame_id,
                    Time(nanoseconds=source_ns)).transform
            except TransformException:
                entry['tf_failures'] += 1
                continue
            entry['sampled_messages'] += 1
            points = (_custom_xyz(message)
                      if type_name.endswith('/CustomMsg')
                      else _cloud_xyz(message))
            base_points = []
            for x, y, z, tag in points:
                entry['points'] += 1
                if tag is not None:
                    entry['tags'][str(tag)] = entry['tags'].get(str(tag), 0) + 1
                sensor_range = math.sqrt(x * x + y * y + z * z)
                if sensor_range == 0.0:
                    entry['zero_points'] += 1
                    continue
                if sensor_range < 0.1:
                    entry['sensor_range_lt_0p1'] += 1
                if sensor_range < fast_lio['blind_m']:
                    entry['sensor_range_lt_blind'] += 1
                point = _transform_point(transform, (x, y, z))
                horizontal = math.hypot(point[0], point[1])
                if horizontal < 1.0:
                    entry['near_horizontal_ranges'].append(horizontal)
                    entry['near_heights'].append(point[2])
                base_points.append(point)
            counts = classify_cloud_points(base_points, footprint, outer, band)
            for key, value in counts.items():
                entry['sweep'][key] += value

    for entry in scans.values():
        entry['frames'] = sorted(entry['frames'])
        entry['scan_range_min'] = sorted(entry['scan_range_min'])
        entry['scan_range_max'] = sorted(entry['scan_range_max'])
        entry['receipt_minus_source_sec'] = summarize(
            entry['receipt_minus_source_sec'])
        entry['near_range_histogram_m'] = histogram(
            entry.pop('finite_ranges'), NEAR_FIELD_BINS_M)
        entry['tf_failure_rate'] = (
            entry['tf_failures'] / entry['messages']
            if entry['messages'] else None)
    for entry in clouds.values():
        entry['frames'] = sorted(entry['frames'])
        entry['receipt_minus_source_sec'] = summarize(
            entry['receipt_minus_source_sec'])
        entry['near_horizontal_histogram_m'] = histogram(
            entry.pop('near_horizontal_ranges'), NEAR_FIELD_BINS_M)
        entry['near_height_summary_m'] = summarize(entry.pop('near_heights'))
        entry['tf_failure_rate'] = (
            entry['tf_failures'] / entry['tf_attempts']
            if entry['tf_attempts'] else None)
        # Never derive free space from sparse 3D returns.
        entry['sweep'][OBSERVED_FREE] = None

    duration = None if first_ns is None else (last_ns - first_ns) / 1e9
    return {
        'bag': str(bag_path),
        'duration_sec': duration,
        'declared_types': declared,
        'undeserializable_types': unavailable,
        'raw_lidar_type': declared.get('/livox/lidar'),
        'raw_lidar_type_matches_fast_lio': (
            None if '/livox/lidar' not in declared
            else declared['/livox/lidar'] == fast_lio['expected_message_type']),
        'missing_channels': sorted(
            (set(LAUNCH_SCAN_FILTERS) | set(cloud_topics)) - set(declared)),
        'parameter_dumps': sidecar_parameter_dumps(bag_path),
        'scans': scans,
        'clouds': clouds,
        'sweep_outer_radius_m': outer,
        'cloud_stride': cloud_stride,
    }


def conclusions(geometry_report, bag_reports):
    """Fail-closed statements derived from the measured data."""
    lines = []
    for name, channel in geometry_report['channels'].items():
        worst = max(row['unobservable_fraction_of_band']
                    for row in channel['radii'])
        lines.append(
            f'{name}: can_prove_sweep_free={channel["can_prove_sweep_free"]} '
            f'(worst unobservable fraction of the collision band {worst:.2f})')
    for report in bag_reports:
        for topic, entry in report['scans'].items():
            sweep = entry['sweep']
            if sweep[OBSERVED_FREE]:
                lines.append(
                    f'{report["bag"]} {topic}: {sweep[OBSERVED_FREE]} beams '
                    'claim free sweep evidence; review range_min/frame')
        if report['missing_channels']:
            lines.append(f'{report["bag"]}: missing '
                         f'{", ".join(report["missing_channels"])}')
        if report['raw_lidar_type_matches_fast_lio'] is False:
            lines.append(
                f'{report["bag"]}: /livox/lidar recorded as '
                f'{report["raw_lidar_type"]}, FAST-LIO config expects a '
                'different type')
    return lines


def markdown(report):
    """Human-readable summary of the JSON report."""
    out = ['# MID-360 定位近场可见性审计（只读）', '']
    geometry = report['geometry']
    out.append(
        f'- 扫掠环：内半径 {geometry["sweep_inner_radius_m"]} m，外半径 '
        f'{geometry["sweep_outer_radius_m"]} m（padding {geometry["padding_m"]} m）')
    out.append(f'- 碰撞高度带：{geometry["collision_z_band_m"]} m；'
               f'传感器原点 {geometry["sensor_origin_m"]} m')
    out += ['', '| 通道 | 可证明扫掠自由 | 各半径最差不可观测比例 |',
            '| --- | --- | --- |']
    for name, channel in geometry['channels'].items():
        fractions = ', '.join(
            f'{row["radius_m"]:.3f}m:{row["unobservable_fraction_of_band"]:.2f}'
            for row in channel['radii'])
        out.append(f'| {name} | {channel["can_prove_sweep_free"]} | '
                   f'{fractions} |')
    for bag in report['bags']:
        out += ['', f'## {bag["bag"]}', '',
                f'- 时长 {bag["duration_sec"]} s；/livox/lidar 类型 '
                f'{bag["raw_lidar_type"]}；缺失通道 {bag["missing_channels"]}']
        for topic, entry in bag['scans'].items():
            out.append(
                f'- {topic} ({entry["type"]}, frame {entry["frames"]}): '
                f'扫掠 {entry["sweep"]}；未知原因 '
                f'{entry["sweep_unknown_reasons"]}；TF 失败率 '
                f'{entry["tf_failure_rate"]}')
        for topic, entry in bag['clouds'].items():
            out.append(
                f'- {topic} ({entry["type"]}): 采样 {entry["sampled_messages"]}'
                f' 帧，扫掠体积内 {entry["sweep"]}；<0.1 m '
                f'{entry["sensor_range_lt_0p1"]}，<blind '
                f'{entry["sensor_range_lt_blind"]}')
    out += ['', '## 结论（fail-closed）', '']
    out += [f'- {line}' for line in report['conclusions']]
    return '\n'.join(out) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('bags', nargs='*', type=Path,
                        help='rosbag2 directories; omit for geometry only')
    parser.add_argument('--parameters', type=Path, default=PARAMETERS)
    parser.add_argument('--fast-lio-config', type=Path,
                        default=FAST_LIO_CONFIG)
    parser.add_argument('--padding', type=float, default=0.05)
    parser.add_argument('--cloud-stride', type=int, default=10)
    parser.add_argument('--output', type=Path,
                        help='write JSON here and Markdown beside it')
    args = parser.parse_args()
    if args.cloud_stride < 1:
        parser.error('--cloud-stride must be >= 1')

    geometry = load_robot_geometry(args.parameters)
    fast_lio = load_fast_lio_preprocess(args.fast_lio_config)
    geometry_report = analytic_sweep_coverage(geometry, fast_lio, args.padding)
    bags = [audit_bag(path, geometry, fast_lio, args.padding,
                      args.cloud_stride) for path in args.bags]
    report = {
        'schema_version': 1,
        'motion_commanded': False,
        'fast_lio_preprocess': fast_lio,
        'declared_pointcloud_type': geometry['declared_pointcloud_type'],
        'geometry': geometry_report,
        'bags': bags,
        'conclusions': conclusions(geometry_report, bags),
    }
    encoded = json.dumps(report, indent=2, sort_keys=True, default=str)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + '\n', 'utf-8')
        args.output.with_suffix('.md').write_text(markdown(report), 'utf-8')
    else:
        print(encoded)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
