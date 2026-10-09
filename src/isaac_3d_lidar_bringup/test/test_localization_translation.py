"""Offline geometry and fail-closed linear motion tests; no robot connection."""
from dataclasses import replace
import math

import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError, SE2
from isaac_3d_lidar_bringup.localization_motion_guard import OdomSample
from isaac_3d_lidar_bringup.localization_translation_policy import (
    preview_translation, choose_translation)
from isaac_3d_lidar_bringup.localization_translation_guard import (
    LinearProfile, LinearProbeGuard)
from test_localization_rotation_policy import _grid, FOOTPRINT, ORIGIN


def preview(grid=None, start=ORIGIN, distance=.4, **kwargs):
    return preview_translation(grid or _grid(half=2, resolution=.05),
                               FOOTPRINT, start, distance,
                               stop_extension_m=.12, now_ns=1000, **kwargs)


@pytest.mark.parametrize('point', [(.3, 0), (.3, .18), (.65, 0), (0, 0)])
def test_whole_path_sides_stop_tail_and_body_obstacles(point):
    result = preview(_grid(half=2, resolution=.02, occupied_points=(point,)))
    assert not result.geometry_clear
    assert result.reason == 'OBSTACLE_IN_SWEEP'


def test_unknown_blind_ring_cannot_be_assumed_clear():
    assert preview().geometry_clear
    result = preview(_grid(half=2, resolution=.02,
                           unknown=lambda x, y: .15 < x < .22))
    assert result.reason == 'UNKNOWN_SWEEP'


def test_only_wholly_inside_current_body_is_self_masked():
    result = preview(_grid(half=2, resolution=.02,
                           unknown=lambda x, y: abs(x) < .05 and abs(y) < .05))
    assert result.geometry_clear


def test_rotated_odom_path_and_obstacle():
    pose = SE2(0, 0, math.pi / 2)
    assert preview(start=pose).target.y == pytest.approx(.4)
    assert preview(_grid(half=2, resolution=.02, occupied_points=((0, .3),)),
                   start=pose).reason == 'OBSTACLE_IN_SWEEP'


@pytest.mark.parametrize('now', [1, 1_000_001_000])
def test_source_time_age_or_future_rejected(now):
    result = preview_translation(_grid(), FOOTPRINT, ORIGIN, .4,
                                 stop_extension_m=.12, now_ns=now)
    assert result.reason == 'SENSOR_STALE'


def test_bounded_work_and_mixed_preview_rejection():
    assert preview(deadline=0).reason == 'PREVIEW_BUDGET_EXHAUSTED'
    a, b = preview(distance=.2), preview(distance=.4)
    assert choose_translation((a, b)) == b
    assert choose_translation((replace(a, geometry_clear=False),)) is None
    with pytest.raises(ContractError):
        choose_translation((a, replace(b, snapshot_stamp_ns=2)))


def test_concave_self_mask_rejected():
    with pytest.raises(ContractError):
        preview_translation(_grid(), ((-.1, -.1), (.1, -.1), (0, 0), (.1, .1), (-.1, .1)),
                            ORIGIN, .2, stop_extension_m=.1, now_ns=1000)


HASHES = ('a' * 64, 'b' * 64, 'c' * 64)


def profile(status='ACCEPTED'):
    return LinearProfile(*HASHES, ('synthetic-test-only',), .05, .1, .02, .5, .04,
                         status=status)


def odom(t, x=0, y=0, yaw=0):
    return OdomSample(round((t + 10) * 1e9), x, y, yaw, 0., 0., t)


def ready(p=None):
    guard = LinearProbeGuard(p or profile(), HASHES)
    guard.request('STOP', 's', 0, 0.)
    guard.on_emergency(False, 0.)
    for i in range(13):
        t = i / 10
        guard.on_odom(odom(t))
        guard.on_chassis(True, False, t)
        guard.tick(t)
    assert guard.state == 'STOPPED'
    return guard


def move(g, t=1.2, distance=.4):
    g.request('MOVE', 's', 1, t, distance_m=distance, speed_mps=.05,
              profile_hash=g.profile.digest)


def observation(g, t, x=0, y=0, yaw=0):
    sample = odom(t, x, y, yaw)
    g.on_odom(sample)
    g.on_chassis(True, False, t)
    grid = replace(_grid(half=2, resolution=.05), stamp_ns=sample.stamp_ns)
    result = preview_translation(grid, FOOTPRINT, SE2(x, y, yaw),
                                 max(.01, g.target_m - g.progress_m),
                                 stop_extension_m=g.stop_extension_m,
                                 now_ns=sample.stamp_ns)
    g.on_preview(result, t)


@pytest.mark.parametrize('status', ['ESTIMATED', 'REVIEWED'])
def test_unaccepted_calibration_cannot_move(status):
    g = ready(profile(status))
    move(g)
    assert g.tick(1.3) == 0
    assert g.reason == 'LINEAR_PROFILE_INVALID'


def test_default_forbidden_and_stop_handshake_required():
    g = LinearProbeGuard()
    g.request('MOVE', 's', 0, 0., distance_m=.3, speed_mps=.03)
    assert g.state == 'FAULT' and g.tick(0.) == 0


def test_normal_move_renews_without_reset_and_stops_early():
    g = ready()
    move(g)
    for i in range(1, 62):
        t, x = 1.2 + i * .1, i * .005
        observation(g, t, x)
        move(g, t)
        command = g.tick(t)
        assert 0 <= command <= .05
        if g.state == 'STOPPING':
            break
    assert g.state == 'STOPPING' and command == 0
    assert .28 < g.progress_m < .32
    assert g.segments == 1
    for j in range(1, 15):
        observation(g, t + j * .1, x)
        assert g.tick(t + j * .1) == 0
    assert g.state == 'STOPPED'
    g.request('RELEASE', 's', 2, t + 1.4)
    assert g.tick(t + 1.4) == 0 and g.state == 'RELEASED'


def test_expired_lease_latches_zero_even_after_renewal():
    g = ready()
    move(g)
    observation(g, 1.6)
    assert g.tick(1.6) == 0 and g.reason == 'LEASE_EXPIRED'
    move(g, 1.6)
    assert g.tick(1.7) == 0 and g.state == 'FAULT'


@pytest.mark.parametrize('fault', [
    'conflict', 'emergency', 'chassis', 'lateral', 'yaw', 'jump', 'sweep'])
def test_faults_stop(fault):
    g = ready()
    move(g)
    observation(g, 1.3)
    assert g.tick(1.3) > 0
    if fault == 'conflict':
        move(g, 1.4, .3)
    elif fault == 'emergency':
        g.on_emergency(True, 1.4)
    elif fault == 'chassis':
        g.on_chassis(False, False, 1.4)
    elif fault in ('lateral', 'yaw', 'jump'):
        g.on_odom(odom(1.4, x=.3 if fault == 'jump' else 0,
                       y=.05 if fault == 'lateral' else 0,
                       yaw=.1 if fault == 'yaw' else 0))
    else:
        g.on_preview(replace(g._preview[0], geometry_clear=False, reason='UNKNOWN_SWEEP'), 1.4)
    assert g.tick(1.4) == 0 and g.state == 'FAULT'


def test_stationary_stall_and_current_pose_resweep_required():
    g = ready()
    move(g)
    for i in range(1, 34):
        t = 1.2 + i * .1
        observation(g, t)
        move(g, t)
        g.tick(t)
    assert g.reason == 'NO_PROGRESS'
    g = ready()
    move(g)
    observation(g, 1.3)
    g.on_odom(odom(1.4, x=.01))
    assert g.tick(1.4) == 0 and g.reason == 'SWEEP_POSE_MISMATCH'


@pytest.mark.parametrize('fault', ['odom', 'preview', 'short', 'identity', 'speed', 'tiny'])
def test_additional_fail_closed_boundaries(fault):
    g = ready()
    if fault == 'identity':
        g.permitted = False
    move(g, distance=.1 if fault == 'tiny' else .4)
    if fault == 'tiny':
        assert g.reason == 'INSUFFICIENT_PROBE_DISTANCE'
        return
    if fault == 'identity':
        assert g.reason == 'LINEAR_PROFILE_INVALID'
        return
    observation(g, 1.3)
    if fault == 'odom':
        move(g, 2.)
        assert g.tick(2.) == 0 and g.reason == 'SENSOR_STALE'
    elif fault == 'preview':
        g.on_preview(replace(g._preview[0], snapshot_stamp_ns=1), 1.3)
        assert g.tick(1.3) == 0 and g.reason == 'SWEEP_STALE'
    elif fault == 'short':
        g.on_preview(replace(g._preview[0], distance_m=.1), 1.3)
        assert g.tick(1.3) == 0 and g.reason == 'SWEEP_TOO_SHORT'
    else:
        for i in range(1, 7):
            g.on_odom(odom(1.3 + i * .1, x=i * .012))
        assert g.reason == 'OVERSPEED' and g.tick(1.9) == 0


def test_nan_preview_cannot_bypass_distance_gate():
    with pytest.raises(ContractError):
        replace(preview(), distance_m=math.nan)


def test_total_budget_counts_real_travel_and_limits_segments():
    g = ready()
    g.total_travel_m = 1.1
    move(g)
    assert g.reason == 'MOTION_BUDGET_EXHAUSTED'
    g = ready()
    g.segments = g.config.max_segments
    move(g)
    assert g.reason == 'MOTION_BUDGET_EXHAUSTED'
    g = ready()
    move(g)
    g.on_odom(odom(1.3, x=.005))
    g.on_odom(odom(1.4, x=.003))
    assert g.progress_m == pytest.approx(.003)
    assert g.total_travel_m == pytest.approx(.007)
