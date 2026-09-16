import math

import pytest

from carbot_hardware.odometry_math import Pose2D, integrate_tick_delta


METERS_PER_TICK = 2.0 * math.pi * 0.02175 / 1560.0
TRACK_SEPARATION_M = 0.254


def test_equal_positive_ticks_drive_straight_forward():
    result = integrate_tick_delta(
        Pose2D(), 1560, 1560, METERS_PER_TICK, TRACK_SEPARATION_M
    )
    assert result.pose.x == pytest.approx(2.0 * math.pi * 0.02175)
    assert result.pose.y == pytest.approx(0.0)
    assert result.pose.yaw == pytest.approx(0.0)


def test_opposite_ticks_turn_in_place():
    result = integrate_tick_delta(
        Pose2D(), -100, 100, METERS_PER_TICK, TRACK_SEPARATION_M
    )
    assert result.pose.x == pytest.approx(0.0)
    assert result.pose.y == pytest.approx(0.0)
    assert result.pose.yaw > 0.0
    assert result.yaw_rad == pytest.approx(
        200 * METERS_PER_TICK / TRACK_SEPARATION_M
    )


def test_curved_motion_uses_exact_arc():
    result = integrate_tick_delta(
        Pose2D(), 100, 200, METERS_PER_TICK, TRACK_SEPARATION_M
    )
    assert result.pose.x > 0.0
    assert result.pose.y > 0.0
    assert result.pose.yaw > 0.0


@pytest.mark.parametrize("meters_per_tick,track", [(0.0, 0.254), (0.001, 0.0)])
def test_invalid_geometry_is_rejected(meters_per_tick, track):
    with pytest.raises(ValueError):
        integrate_tick_delta(Pose2D(), 1, 1, meters_per_tick, track)
