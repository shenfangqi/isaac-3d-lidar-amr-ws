"""Near-field scan: raw LiDAR returns inside the /scan blind zone."""

import math

import numpy as np

from isaac_3d_lidar_bringup.near_field_scan import near_field_ranges


FOOTPRINT = [(0.155, 0.133), (0.155, -0.133), (-0.130, -0.133),
             (-0.130, 0.133)]
LIDAR = (0.017, 0.0, 0.209)
BINS = 360


def _bin(degrees):
    return int((math.radians(degrees) + math.pi) / (2 * math.pi) * BINS)


def _bin_of(x, y):
    return _bin(math.degrees(math.atan2(y, x)))


def _ranges(points, **overrides):
    return near_field_ranges(np.array(points, dtype=float), LIDAR, FOOTPRINT,
                             bins=BINS, **overrides)


def test_an_obstacle_inside_the_scan_blind_zone_is_a_hit():
    # A bag face 0.30 m ahead, 0.15-0.30 m high.
    face = [(0.30, y, z) for y in (-0.05, 0.0, 0.05) for z in (0.15, 0.30)]
    ranges = _ranges(face)
    assert math.isclose(ranges[_bin(0.5)], 0.30, abs_tol=1e-4)


def test_the_nearest_return_of_a_bin_wins():
    ranges = _ranges([(0.45, 0.001, 0.2), (0.25, 0.001, 0.2)])
    assert math.isclose(ranges[_bin(0.2)], 0.25, abs_tol=1e-4)


def test_floor_roof_and_out_of_band_returns_are_ignored():
    ranges = _ranges([(0.3, 0.0, 0.02), (0.3, 0.0, 0.5)])
    assert np.isnan(ranges).all()


def test_returns_on_the_robot_itself_are_ignored():
    # Inside the footprint plus the 5 cm margin.
    ranges = _ranges([(0.19, 0.0, 0.2), (0.0, 0.17, 0.2)])
    assert np.isnan(ranges).all()


def test_livox_zero_points_at_the_origin_are_ignored():
    ranges = _ranges([LIDAR, (0.0, 0.0, 0.0)])
    assert np.isnan(ranges).all()


def test_a_low_beam_reaching_past_the_near_field_clears_its_bin():
    # -5 deg beam returning at 2 m along 90 deg: observed free near field.
    distance = 2.0
    z = LIDAR[2] - distance * math.tan(math.radians(5.0))
    ranges = _ranges([(LIDAR[0], distance, z)])
    assert math.isclose(ranges[_bin_of(LIDAR[0], distance)], 0.65)
    assert np.isnan(ranges[_bin_of(LIDAR[0], -distance)])


def test_a_steep_beam_does_not_clear_the_near_field():
    # A 30 deg beam passes above the height band within the near field.
    distance = 2.0
    z = LIDAR[2] + distance * math.tan(math.radians(30.0))
    assert np.isnan(_ranges([(LIDAR[0], distance, z)])).all()


def test_a_hit_is_never_replaced_by_free_space():
    far = (2.0, 0.001, LIDAR[2])
    ranges = _ranges([far, (0.35, 0.001, 0.2)])
    assert math.isclose(ranges[_bin(0.2)], 0.35, abs_tol=1e-4)


def test_clearing_can_be_disabled_by_an_empty_elevation_band():
    far = (2.0, 0.001, LIDAR[2])
    ranges = _ranges([far], clear_elevation_deg=(1.0, -1.0))
    assert np.isnan(ranges).all()


def _livox_cdr(frame_id, points, sec=7, nanosec=9):
    """Serialize a livox_ros_driver2/CustomMsg by hand (little-endian CDR)."""
    import struct

    def pad(buffer, size):
        return buffer + b'\0' * (-len(buffer) % size)

    body = struct.pack('<iI', sec, nanosec)
    name = frame_id.encode() + b'\0'
    body += struct.pack('<I', len(name)) + name
    body = pad(body, 8) + struct.pack('<Q', 123)             # timebase
    body = pad(body, 4) + struct.pack('<I', len(points))     # point_num
    body += struct.pack('<B3B', 1, 0, 0, 0)                  # lidar_id, rsvd
    body = pad(body, 4) + struct.pack('<I', len(points))
    for index, (x, y, z) in enumerate(points):
        body = pad(body, 4) + struct.pack('<I3f3B', index, x, y, z, 5, 0, 1)
    return b'\x00\x01\x00\x00' + body


def test_livox_cdr_points_are_read_for_any_frame_id_length():
    from isaac_3d_lidar_bringup.near_field_scan import livox_custom_xyz

    points = [(0.25, -0.5, 0.125), (1.0, 2.0, -0.25), (3.5, 0.0, 0.0)]
    for frame_id in ('', 'ab', 'abc', 'livox_frame', 'a_longer_frame_name'):
        frame, sec, nanosec, xyz = livox_custom_xyz(
            _livox_cdr(frame_id, points))
        assert (frame, sec, nanosec) == (frame_id, 7, 9)
        assert xyz.tolist() == [list(point) for point in points]


def test_an_empty_livox_frame_has_no_points():
    from isaac_3d_lidar_bringup.near_field_scan import livox_custom_xyz

    assert livox_custom_xyz(_livox_cdr('livox_frame', []))[3].shape == (0, 3)
