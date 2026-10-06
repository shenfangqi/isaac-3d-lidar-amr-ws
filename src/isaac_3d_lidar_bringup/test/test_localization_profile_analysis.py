"""Issue #13 PR3 rotation profile analysis on synthetic stops."""

import math

import pytest

from isaac_3d_lidar_bringup.localization_contracts import ProfileStatus
from isaac_3d_lidar_bringup.localization_profile_analysis import (
    analyze_segment,
    build_profile,
    command_segments,
    OdomPoint,
    section_hash,
    summarize,
    TwistSample,
)


def _stop(rate, start=0.0, on=1.0, off=3.0, start_latency=0.2,
          stop_latency=0.15, decel=1.0, drift=0.0, dt=0.05):
    """Commands and odometry for one rotation with known timings."""
    commands = [TwistSample(start, 0.0, 0.0), TwistSample(on, 0.0, rate),
                TwistSample(off, 0.0, 0.0)]
    odometry, yaw, speed, t = [], 0.0, 0.0, start
    while t <= off + 3.0:
        if on + start_latency <= t < off + stop_latency:
            speed = rate
        elif t >= off + stop_latency:
            speed = math.copysign(max(0.0, abs(speed) - decel * dt), rate)
        else:
            speed = 0.0
        yaw += speed * dt
        moved = drift if t >= on + start_latency else 0.0
        odometry.append(OdomPoint(t, moved, 0.0,
                                  math.atan2(math.sin(yaw), math.cos(yaw)),
                                  speed))
        t += dt
    return commands, odometry


def test_segments_ignore_translation_and_split_on_zero():
    commands = [TwistSample(0, 0, 0), TwistSample(1, 0, 0.4),
                TwistSample(2, 0, 0), TwistSample(3, 0.05, 0.4),
                TwistSample(4, 0, -0.4), TwistSample(5, 0, 0)]
    assert command_segments(commands) == [(1, 2, 0.4), (4, 5, -0.4)]


def test_latency_and_tail_are_measured_separately():
    commands, odometry = _stop(0.4, drift=0.01)
    (segment,) = command_segments(commands)
    sample = analyze_segment(segment, odometry)
    assert sample.valid
    assert sample.start_latency_s == pytest.approx(0.2, abs=0.06)
    assert sample.stop_latency_s == pytest.approx(0.2, abs=0.06)
    # 0.4 rad/s decelerating at 1 rad/s^2: about 0.08 rad after onset.
    assert sample.stop_tail_rad == pytest.approx(0.08, abs=0.03)
    assert sample.center_drift_m == pytest.approx(0.01)


@pytest.mark.parametrize('damage, note', [
    ('gap', 'odometry gap'),
    ('still', 'no motion'),
])
def test_bad_data_is_marked_invalid(damage, note):
    commands, odometry = _stop(0.4)
    if damage == 'gap':
        odometry = [p for p in odometry if not 2.0 < p.t < 2.5]
    else:
        odometry = [OdomPoint(p.t, 0.0, 0.0, 0.0, 0.0) for p in odometry]
    (segment,) = command_segments(commands)
    sample = analyze_segment(segment, odometry)
    assert not sample.valid and sample.note == note


def _samples(left, right):
    output = []
    for rate, count in ((0.4, left), (-0.4, right)):
        for _ in range(count):
            commands, odometry = _stop(rate)
            output.append(analyze_segment(command_segments(commands)[0],
                                          odometry))
    return output


def test_three_valid_stops_each_way_give_estimated_profile_only():
    summary = summarize(_samples(3, 3))
    profile = build_profile(summary, 'a' * 64, 'b' * 64, 'c' * 64,
                            ['bag-1'])
    assert profile.status == ProfileStatus.ESTIMATED
    assert profile.externally_reviewed is False
    assert profile.stop_tail_rad == summary['stop_tail_rad']['max']
    assert profile.latency_s == summary['stop_latency_s']['max']


def test_too_few_stops_give_insufficient_with_nulls():
    profile = build_profile(summarize(_samples(3, 2)), 'a' * 64, 'b' * 64,
                            'c' * 64, ['bag-1'])
    assert profile.status == ProfileStatus.INSUFFICIENT
    assert profile.stop_tail_rad is None and profile.latency_s is None


def test_section_hash_changes_with_any_value():
    assert section_hash({'a': 1}) == section_hash({'a': 1})
    assert section_hash({'a': 1}) != section_hash({'a': 2})
