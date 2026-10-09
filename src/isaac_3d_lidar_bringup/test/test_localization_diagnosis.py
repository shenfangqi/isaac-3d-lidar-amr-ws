"""Read-only replay integrity and exact worker input trace regressions."""

from dataclasses import replace
import json
import math
from pathlib import Path
import struct
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts'))
from diagnose_localization_bag import (  # noqa: E402
    frames_from_trace, interpolate_pose, load_map, state_intervals,
)
from isaac_3d_lidar_bringup.localization_contracts import FrameRole, SE2  # noqa: E402
from isaac_3d_lidar_bringup.localization_hypotheses import diagnostic_snapshot  # noqa: E402
from test_localization_hypotheses import _frames, ROOM, IDENTITY  # noqa: E402


def test_interpolation_wraps_yaw_and_refuses_extrapolation():
    samples = [(1_000_000_000, SE2(0., 0., math.radians(179))),
               (1_100_000_000, SE2(.01, 0., math.radians(-179)))]
    stamps = [t for t, _ in samples]
    pose = interpolate_pose(samples, stamps, 1_050_000_000)
    assert pose.x == pytest.approx(.005)
    assert abs(pose.yaw) == pytest.approx(math.pi)
    with pytest.raises(ValueError, match='outside'):
        interpolate_pose(samples, stamps, 1_200_000_000)


def test_interpolation_refuses_gaps_and_jumps():
    with pytest.raises(ValueError, match='gap'):
        interpolate_pose([(1, IDENTITY), (1_000_000_001, IDENTITY)],
                         [1, 1_000_000_001], 500_000_001)
    with pytest.raises(ValueError, match='discontinuity'):
        interpolate_pose([(1, IDENTITY), (11, SE2(1., 0., 0.))], [1, 11], 5)


def test_map_unknown_flip_and_wire_resolution(tmp_path):
    from PIL import Image
    image = Image.new('L', (2, 2))
    image.putdata([0, 254, 205, 0])
    image.save(tmp_path / 'map.pgm')
    path = tmp_path / 'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.05\norigin: [1, 2, 0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    grid = load_map(path)
    assert list(grid.data) == [-1, 100, 100, 0]
    assert grid.info.resolution == struct.unpack('<f', struct.pack('<f', .05))[0]


def test_status_intervals_end_at_next_transition():
    rows = [(10, {'state': 'A'}), (20, {'state': 'A'}), (30, {'state': 'B'})]
    assert state_intervals(rows)[0] == {'state': 'A', 'start_ns': 10, 'end_ns': 30}


def test_trace_records_stamps_and_transforms_without_ranges():
    frames = _frames(ROOM, SE2(1., 1., 0.), FrameRole.TRAIN)
    holdout = tuple(replace(f, role=FrameRole.HOLDOUT, id=f.id + 10,
                            stamp_ns=f.stamp_ns + 10) for f in frames)
    trace = diagnostic_snapshot(None, frames, holdout)
    assert trace['train'][0]['stamp_ns'] == frames[0].stamp_ns
    assert trace['train'][0]['T_odom_base'] == {'x': 0., 'y': 0., 'yaw': 0.}
    assert trace['holdout'][0]['role'] == 'HOLDOUT'
    encoded = json.dumps(trace, allow_nan=False)
    assert 'ranges' not in encoded


def test_recorded_frames_recover_exact_transforms_and_reject_missing_scan():
    frames = _frames(ROOM, SE2(1., 1., 0.), FrameRole.TRAIN)
    trace = diagnostic_snapshot(None, frames, ())
    scans = [(12345, f.scan) for f in frames]
    recovered = frames_from_trace(trace['train'], scans, 's1', FrameRole.TRAIN)
    assert [f.T_odom_base for f in recovered] == [f.T_odom_base for f in frames]
    assert [f.scan for f in recovered] == [f.scan for f in frames]
    with pytest.raises(ValueError, match='scan absent'):
        frames_from_trace(trace['train'], scans[:1], 's1', FrameRole.TRAIN)
