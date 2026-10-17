import math

import pytest

from isaac_3d_lidar_bringup.fast_lio_base_adapter import compose_pose
from isaac_3d_lidar_bringup.fast_lio_base_adapter import PosePlausibilityGuard
from isaac_3d_lidar_bringup.fast_lio_base_adapter import rotate_vector
from isaac_3d_lidar_bringup.fast_lio_base_adapter import yaw_quaternion


def test_compose_pose_rotates_child_translation():
    yaw_90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    position, orientation = compose_pose(
        (1.0, 2.0, 0.0), yaw_90, (1.0, 0.0, 0.0), (0, 0, 0, 1)
    )
    assert position == pytest.approx((1.0, 3.0, 0.0))
    assert orientation == pytest.approx(yaw_90)


def test_yaw_projection_removes_roll_and_pitch():
    roll_30 = (math.sin(math.pi / 12), 0.0, 0.0, math.cos(math.pi / 12))
    projected = yaw_quaternion(roll_30)
    assert projected == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_rotate_vector_preserves_length():
    yaw_90 = (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    rotated = rotate_vector(yaw_90, (2.0, 0.0, 0.0))
    assert rotated == pytest.approx((0.0, 2.0, 0.0))


def test_guard_ignores_fast_lio_initialization_transient():
    guard = PosePlausibilityGuard(grace_period=20.0)
    identity = (0.0, 0.0, 0.0, 1.0)
    assert guard.evaluate(
        0.0, (0.0, 0.0, 0.0), identity, (0.0, 0.0, 0.0)
    ) is None
    assert guard.evaluate(
        10.0, (3.0, 0.0, 1.0), identity, (3.0, 0.0, 1.0)
    ) is None


IDENTITY = (0.0, 0.0, 0.0, 1.0)
STILL = (0.0, 0.0, 0.0)


def _drive(guard, start, end, speed=(0.1, 0.0, 0.0), origin=(0.0, 0.0, 0.0),
           step=0.1):
    """Feed poses moving at ``speed`` from ``start`` to ``end`` seconds."""
    reason = None
    stamp = start
    while stamp <= end + 1e-9 and reason is None:
        elapsed = stamp - start
        position = tuple(o + v * elapsed for o, v in zip(origin, speed))
        reason = guard.evaluate(stamp, position, IDENTITY, speed)
        stamp = round(stamp + step, 6)
    return reason


def test_guard_ignores_filter_velocity_glitches():
    # Robot 2026-10-17: FAST-LIO's filter velocity reached 0.64 m/s while the
    # pose moved at the commanded 0.1 m/s.
    guard = PosePlausibilityGuard(grace_period=0.0)
    assert _drive(guard, 1.0, 2.0) is None
    for index in range(5):
        stamp = 2.1 + 0.1 * index
        position = (0.1 + 0.01 * (index + 1), 0.0, 0.0)
        assert guard.evaluate(
            stamp, position, IDENTITY, (0.0, 0.0, 0.64)) is None


def test_guard_ignores_stalled_then_caught_up_poses():
    # The pose repeats for three frames, then catches up 4 cm in one frame:
    # 0.4 m/s for that frame, 0.1 m/s over the window.
    guard = PosePlausibilityGuard(grace_period=0.0)
    assert _drive(guard, 1.0, 2.0) is None
    x = 0.1
    for repeat in range(4):
        for frame in range(3):
            assert guard.evaluate(
                2.1 + 0.4 * repeat + 0.1 * frame, (x, 0.0, 0.0), IDENTITY,
                STILL) is None
        x += 0.04
        assert guard.evaluate(
            2.4 + 0.4 * repeat, (x, 0.0, 0.0), IDENTITY, STILL) is None


def test_guard_latches_a_sustained_planar_drift():
    guard = PosePlausibilityGuard(grace_period=0.0)
    reason = _drive(guard, 1.0, 3.0, speed=(0.6, 0.0, 0.0))
    assert 'planar speed' in reason
    assert guard.evaluate(5.0, (0.0, 0.0, 0.0), IDENTITY, STILL) == reason


def test_guard_latches_a_sustained_vertical_drift():
    guard = PosePlausibilityGuard(grace_period=0.0)
    reason = _drive(guard, 1.0, 3.0, speed=(0.0, 0.0, 0.6))
    assert 'vertical speed' in reason or 'height change' in reason


def test_guard_trips_at_once_on_a_large_single_jump():
    guard = PosePlausibilityGuard(grace_period=0.0)
    assert _drive(guard, 1.0, 2.0) is None
    reason = guard.evaluate(2.1, (0.4, 0.0, 0.0), IDENTITY, STILL)
    assert 'pose jump' in reason


def test_guard_keeps_recent_history_for_the_fault_log():
    guard = PosePlausibilityGuard(grace_period=0.0, history_s=1.0)
    _drive(guard, 1.0, 3.0)
    text = guard.history_text()
    assert len(text.splitlines()) <= 11
    assert text.splitlines()[-1].startswith(' +0.00 s')


def test_guard_immediately_rejects_timestamp_rollback():
    guard = PosePlausibilityGuard(grace_period=20.0)
    identity = (0.0, 0.0, 0.0, 1.0)
    assert guard.evaluate(
        100.0, (0.0, 0.0, 0.0), identity, (0.0, 0.0, 0.0)
    ) is None
    assert guard.evaluate(
        99.0, (0.0, 0.0, 0.0), identity, (0.0, 0.0, 0.0)
    ) == 'non-increasing FAST-LIO timestamp'
