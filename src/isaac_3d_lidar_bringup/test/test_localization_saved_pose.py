"""Issue #13 saved-pose prior: strict file contract and loading rules."""

import json
import math

import pytest

from isaac_3d_lidar_bringup.localization_contracts import ContractError
from isaac_3d_lidar_bringup.localization_saved_pose import (
    decode_saved_pose,
    encode_saved_pose,
    load_pose,
    save_pose,
    SavedPose,
)


MAP = 'ab' * 32
POSE = SavedPose(MAP, -0.77, 0.10, -1.62, 1_000_000.0, 0.02, 0.016)


def test_round_trip():
    assert decode_saved_pose(encode_saved_pose(POSE)) == POSE


@pytest.mark.parametrize('change', [
    {'schema_version': 2},
    {'extra': 1},
    {'map_hash': 'xyz'},
    {'x': float('nan')},
    {'yaw': 'north'},
    {'xy_std': -0.1},
    {'saved_unix': True},
])
def test_decode_is_strict(change):
    data = json.loads(encode_saved_pose(POSE))
    data.update(change)
    with pytest.raises(ContractError):
        decode_saved_pose(json.dumps(data))


def test_decode_rejects_missing_fields_and_garbage():
    data = json.loads(encode_saved_pose(POSE))
    del data['yaw']
    with pytest.raises(ContractError):
        decode_saved_pose(json.dumps(data))
    with pytest.raises(ContractError):
        decode_saved_pose('not json')


def test_save_is_atomic_and_load_checks_map_and_age(tmp_path):
    path = tmp_path / 'state' / 'pose.json'
    save_pose(str(path), POSE)
    assert [p.name for p in path.parent.iterdir()] == ['pose.json']
    pose, reason = load_pose(str(path), MAP, POSE.saved_unix + 60.0, 3600.0)
    assert pose == POSE and reason == ''
    assert load_pose(str(path), 'cd' * 32, POSE.saved_unix, 3600.0)[1] == (
        'saved pose belongs to another map')
    assert load_pose(str(path), MAP, POSE.saved_unix + 7200.0,
                     3600.0)[0] is None
    # A pose from the future (clock change) is not trusted either.
    assert load_pose(str(path), MAP, POSE.saved_unix - 10.0, 3600.0)[0] is None


def test_load_reports_missing_and_unreadable(tmp_path):
    assert load_pose(str(tmp_path / 'none.json'), MAP, 0.0, 1.0) == (
        None, 'no saved pose')
    bad = tmp_path / 'bad.json'
    bad.write_text('{}')
    pose, reason = load_pose(str(bad), MAP, 0.0, 1.0)
    assert pose is None and reason.startswith('saved pose unreadable')


def test_overwrite_replaces_the_pose(tmp_path):
    path = str(tmp_path / 'pose.json')
    save_pose(path, POSE)
    moved = SavedPose(MAP, 2.7, 2.5, -1.0, POSE.saved_unix + 5.0, 0.02, 0.02)
    save_pose(path, moved)
    assert load_pose(path, MAP, moved.saved_unix, 60.0)[0] == moved
    assert math.isclose(moved.x, 2.7)
