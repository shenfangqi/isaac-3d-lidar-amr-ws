"""Issue #1: temporal median used on the robot instead of laser_filters."""

import math

from isaac_3d_lidar_bringup.scan_temporal_median import TemporalMedian


INF = float('inf')


def test_nothing_is_published_until_the_window_is_full():
    median = TemporalMedian(window=5)
    for _ in range(4):
        assert median.update([1.0, 2.0]) is None
    assert median.update([1.0, 2.0]).tolist() == [1.0, 2.0]


def test_a_return_must_persist_for_most_of_the_window():
    median = TemporalMedian(window=5)
    # Beam 0: a return in 2 of 5 scans stays inf.  Beam 1: 3 of 5 shows.
    scans = [[INF, INF], [INF, 1.5], [1.0, INF], [1.0, 1.5], [INF, 1.5]]
    for ranges in scans[:-1]:
        median.update(ranges)
    result = median.update(scans[-1])
    assert math.isinf(result[0])
    assert result[1] == 1.5


def test_a_scan_of_another_size_restarts_the_window():
    median = TemporalMedian(window=3)
    median.update([1.0, 1.0])
    median.update([1.0, 1.0])
    assert median.update([2.0]) is None
