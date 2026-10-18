"""
Near-field LaserScan from the raw LiDAR cloud for the local costmap.

/scan is projected from FAST-LIO's body cloud, which drops every point
within ``blind`` (0.5 m), and /scan itself has ``range_min`` 0.5 m.  An
obstacle that comes closer than 0.5 m therefore leaves the local costmap;
on 2026-10-17 the robot crept into a handbag that the raw MID-360 cloud had
shown 0.1-0.5 m ahead for 15 s.  The project rule is not to lower the
global /scan range_min; this node is the dedicated near-field channel.

For every azimuth bin around base_footprint:

* the nearest raw point within ``max_range_m`` whose height lies in the
  /scan band and which is outside the footprint (plus a margin) is a hit;
* otherwise, if a beam whose elevation keeps it inside the height band
  across the whole near field returned beyond ``max_range_m``, the near
  field along that bearing is observed free and the bin reports a range
  just beyond ``max_range_m`` (clearing only; the costmap's
  obstacle_max_range keeps it from marking);
* otherwise the bin is NaN: nothing is marked or cleared.

Obstacles lower than the lowest beam (about 0.16 m at 0.3 m for a MID-360
mounted 0.21 m high with a -7 deg lower field of view) are invisible here;
they are for the planned ToF sensors.
"""

import math

import numpy as np


def _inside_polygon(x, y, polygon):
    """Vectorised even-odd test of points against a closed polygon."""
    inside = np.zeros(x.shape, dtype=bool)
    count = len(polygon)
    for index in range(count):
        x1, y1 = polygon[index]
        x2, y2 = polygon[(index + 1) % count]
        crosses = (y1 > y) != (y2 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            x_cross = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        inside ^= crosses & (x < x_cross)
    return inside


def grow_polygon(polygon, margin):
    """Grow an axis-aligned rectangular footprint by ``margin`` metres."""
    xs = [x for x, _ in polygon]
    ys = [y for _, y in polygon]
    x_min, x_max = min(xs) - margin, max(xs) + margin
    y_min, y_max = min(ys) - margin, max(ys) + margin
    return [(x_max, y_max), (x_max, y_min), (x_min, y_min), (x_min, y_max)]


def near_field_ranges(points, lidar_origin, footprint, bins=360,
                      max_range_m=0.6, min_height_m=0.10, max_height_m=0.35,
                      footprint_margin_m=0.05, min_lidar_range_m=0.1,
                      clear_elevation_deg=(-8.0, 13.0), clear_pad_m=0.05):
    """
    Return per-bin ranges (NaN, hit distance, or max_range + clear_pad).

    ``points`` is an (N, 3) array in base_footprint (z up from the floor);
    ``lidar_origin`` is the LiDAR origin in the same frame.  Bin ``i``
    covers azimuths [-pi + i * 2 pi / bins, -pi + (i + 1) * 2 pi / bins).
    """
    ranges = np.full(bins, np.nan)
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if points.size == 0:
        return ranges
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    offset = points - np.asarray(lidar_origin, dtype=float)
    lidar_range = np.linalg.norm(offset, axis=1)
    # Livox fills no-return slots with zeros, i.e. the LiDAR origin.
    points = points[lidar_range >= min_lidar_range_m]
    offset = offset[lidar_range >= min_lidar_range_m]
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    distance = np.hypot(x, y)
    azimuth = np.arctan2(y, x)
    index = np.clip(((azimuth + math.pi) / (2.0 * math.pi) * bins).astype(int),
                    0, bins - 1)

    self_mask = _inside_polygon(
        x, y, grow_polygon(footprint, footprint_margin_m))
    hit = ((distance <= max_range_m) & (z >= min_height_m)
           & (z <= max_height_m) & ~self_mask)
    if hit.any():
        nearest = np.full(bins, np.inf)
        np.minimum.at(nearest, index[hit], distance[hit])
        found = np.isfinite(nearest)
        ranges[found] = nearest[found]

    elevation = np.degrees(np.arctan2(
        offset[:, 2], np.hypot(offset[:, 0], offset[:, 1])))
    low, high = clear_elevation_deg
    through = ((distance > max_range_m) & (elevation >= low)
               & (elevation <= high))
    free = np.zeros(bins, dtype=bool)
    free[np.unique(index[through])] = True
    clear = free & np.isnan(ranges)
    ranges[clear] = max_range_m + clear_pad_m
    return ranges


def _align(position, size):
    return (position + size - 1) // size * size


# livox_ros_driver2/CustomPoint in CDR: uint32 offset_time, float32 x y z,
# uint8 reflectivity tag line; the next point's uint32 aligns it to 20 bytes.
_LIVOX_POINT = np.dtype([('offset_time', '<u4'), ('x', '<f4'), ('y', '<f4'),
                         ('z', '<f4'), ('reflectivity', 'u1'), ('tag', 'u1'),
                         ('line', 'u1'), ('pad', 'u1')])


def livox_custom_xyz(cdr):
    """
    Return (frame_id, sec, nanosec, (N, 3) xyz) from a serialized CustomMsg.

    Deserializing the 20 000 points of a MID-360 frame into Python objects
    takes longer than a scan period, so the points are read straight from
    the little-endian CDR buffer (4-byte encapsulation header first).
    """
    data = memoryview(cdr)[4:]
    if bytes(cdr[:2]) != b'\x00\x01':
        raise ValueError('expected little-endian CDR')
    sec, nanosec, length = np.frombuffer(data[:12], dtype='<i4,<u4,<u4')[0]
    frame_id = bytes(data[12:12 + length - 1]).decode()
    position = _align(12 + length, 8) + 8              # header, uint64 timebase
    position = _align(position, 4) + 4 + 1 + 3          # point_num, lidar_id, rsvd
    position = _align(position, 4)
    count = int(np.frombuffer(data[position:position + 4], dtype='<u4')[0])
    position += 4
    raw = bytes(data[position:position + count * 20]).ljust(count * 20, b'\0')
    points = np.frombuffer(raw, dtype=_LIVOX_POINT, count=count)
    xyz = np.stack([points['x'], points['y'], points['z']], axis=1)
    return frame_id, int(sec), int(nanosec), xyz


def _footprint_from_string(text):
    import yaml

    return [tuple(map(float, point)) for point in yaml.safe_load(text)]


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan, PointCloud2
    from tf2_ros import Buffer, TransformException, TransformListener

    class NearFieldScan(Node):
        """Publish /scan_near from the raw LiDAR cloud."""

        def __init__(self):
            super().__init__('near_field_scan')
            self.declare_parameter('input_topic', '/livox/lidar')
            # livox_custom (robot CustomMsg) or pointcloud2 (simulation).
            self.declare_parameter('input_type', 'livox_custom')
            self.declare_parameter('output_topic', '/scan_near')
            self.declare_parameter('base_frame', 'base_footprint')
            self.declare_parameter(
                'footprint',
                '[[0.155, 0.133], [0.155, -0.133], [-0.130, -0.133], '
                '[-0.130, 0.133]]')
            self.declare_parameter('bins', 360)
            self.declare_parameter('max_range_m', 0.6)
            self.declare_parameter('min_height_m', 0.10)
            self.declare_parameter('max_height_m', 0.35)
            self.declare_parameter('footprint_margin_m', 0.05)
            self.declare_parameter('min_lidar_range_m', 0.1)
            self.declare_parameter('clear_elevation_min_deg', -8.0)
            self.declare_parameter('clear_elevation_max_deg', 13.0)
            # A 20 000-point MID-360 frame is about 360 kB; best-effort
            # delivery loses whole frames when a fragment is lost (59 % of the
            # frames arrived in a loopback bag replay, 94 % with reliable).
            # Best effort is the default: a reliable point-cloud subscriber on
            # the Jetson has stalled FAST-LIO before, and /livox/lidar feeds
            # FAST-LIO.  About 6 Hz is 1.7 cm per update at 0.1 m/s.
            self.declare_parameter('input_reliable', False)
            self._base_frame = self.get_parameter('base_frame').value
            self._footprint = _footprint_from_string(
                self.get_parameter('footprint').value)
            self._options = {
                'bins': int(self.get_parameter('bins').value),
                'max_range_m': self.get_parameter('max_range_m').value,
                'min_height_m': self.get_parameter('min_height_m').value,
                'max_height_m': self.get_parameter('max_height_m').value,
                'footprint_margin_m': self.get_parameter(
                    'footprint_margin_m').value,
                'min_lidar_range_m': self.get_parameter(
                    'min_lidar_range_m').value,
                'clear_elevation_deg': (
                    self.get_parameter('clear_elevation_min_deg').value,
                    self.get_parameter('clear_elevation_max_deg').value),
            }
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self._transform = None
            self._publisher = self.create_publisher(
                LaserScan, self.get_parameter('output_topic').value,
                qos_profile_sensor_data)
            input_type = self.get_parameter('input_type').value
            topic = self.get_parameter('input_topic').value
            input_qos = QoSProfile(
                depth=1,
                history=HistoryPolicy.KEEP_LAST,
                reliability=(
                    ReliabilityPolicy.RELIABLE
                    if self.get_parameter('input_reliable').value
                    else ReliabilityPolicy.BEST_EFFORT),
                durability=DurabilityPolicy.VOLATILE)
            if input_type == 'livox_custom':
                from livox_ros_driver2.msg import CustomMsg
                self.create_subscription(
                    CustomMsg, topic, self._livox_callback, input_qos,
                    raw=True)
            elif input_type == 'pointcloud2':
                self.create_subscription(
                    PointCloud2, topic, self._cloud_callback, input_qos)
            else:
                raise ValueError(f'unsupported input_type {input_type}')

        def _livox_callback(self, cdr):
            frame_id, sec, nanosec, xyz = livox_custom_xyz(cdr)
            self._process(frame_id, sec, nanosec, xyz)

        def _cloud_callback(self, message):
            from sensor_msgs_py import point_cloud2

            data = point_cloud2.read_points_numpy(
                message, field_names=('x', 'y', 'z'), skip_nans=True)
            self._process(
                message.header.frame_id, message.header.stamp.sec,
                message.header.stamp.nanosec,
                np.asarray(data, dtype=np.float32).reshape(-1, 3))

        def _sensor_to_base(self, frame_id):
            # The LiDAR mount is static; look it up once.
            if self._transform is None:
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._base_frame, frame_id, Time()).transform
                except TransformException as error:
                    self.get_logger().warning(
                        f'near-field scan waits for {frame_id} -> '
                        f'{self._base_frame}: {error}',
                        throttle_duration_sec=5.0)
                    return None
                q = transform.rotation
                rotation = np.array([
                    [1 - 2 * (q.y * q.y + q.z * q.z),
                     2 * (q.x * q.y - q.z * q.w),
                     2 * (q.x * q.z + q.y * q.w)],
                    [2 * (q.x * q.y + q.z * q.w),
                     1 - 2 * (q.x * q.x + q.z * q.z),
                     2 * (q.y * q.z - q.x * q.w)],
                    [2 * (q.x * q.z - q.y * q.w),
                     2 * (q.y * q.z + q.x * q.w),
                     1 - 2 * (q.x * q.x + q.y * q.y)]])
                t = transform.translation
                self._transform = (rotation, np.array([t.x, t.y, t.z]))
            return self._transform

        def _process(self, frame_id, sec, nanosec, xyz):
            transform = self._sensor_to_base(frame_id)
            if transform is None:
                return
            rotation, origin = transform
            points = xyz.astype(float) @ rotation.T + origin
            ranges = near_field_ranges(
                points, origin, self._footprint, **self._options)
            bins = self._options['bins']
            scan = LaserScan()
            scan.header.stamp.sec = sec
            scan.header.stamp.nanosec = nanosec
            scan.header.frame_id = self._base_frame
            scan.angle_increment = 2.0 * math.pi / bins
            # Bin centres, so the published angles match the binning.
            scan.angle_min = -math.pi + scan.angle_increment / 2.0
            scan.angle_max = scan.angle_min + (bins - 1) * scan.angle_increment
            scan.scan_time = 0.1
            scan.range_min = 0.0
            scan.range_max = float(self._options['max_range_m']) + 0.1
            scan.ranges = [float(value) for value in ranges]
            self._publisher.publish(scan)

    rclpy.init(args=args)
    node = NearFieldScan()
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
