"""
Issue #13 PR5 startup closure: profile gate and guarded-rotation plumbing.

The profile checker runs on the workstation without ROS; the shell scripts
are checked statically (they need the Jetson to run).
"""

import importlib.util
import json
from pathlib import Path
import subprocess

import pytest
import yaml

from isaac_3d_lidar_bringup.localization_contracts import (
    encode_motion_profile,
    MotionProfile,
)


PROJECT_DIR = Path(__file__).resolve().parents[3]
SCRIPTS = PROJECT_DIR / 'scripts'
PARAMETERS = yaml.safe_load((
    PROJECT_DIR / 'src/carbot_description/config/carbot_parameters.yaml'
).read_text(encoding='utf-8'))


def _checker():
    spec = importlib.util.spec_from_file_location(
        'check_motion_profile', SCRIPTS / 'check_motion_profile.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _profile(status='ACCEPTED', reviewed=True, **changes):
    checker = _checker()
    geometry, extrinsics, control = checker.canonical_hashes(PARAMETERS, 0.05)
    values = dict(geometry_hash=geometry, extrinsics_hash=extrinsics,
                  control_chain_hash=control)
    values.update(changes)
    return encode_motion_profile(MotionProfile(
        1, values['geometry_hash'], values['extrinsics_hash'],
        values['control_chain_hash'], ('bag-1', 'review:x'), 0.001, 0.015,
        0.171, reviewed, status))


def test_accepted_matching_profile_passes_and_reports_hashes():
    result = _checker().check(_profile(), PARAMETERS)
    assert set(result) == {'geometry_hash', 'extrinsics_hash',
                           'control_chain_hash', 'profile_hash'}


@pytest.mark.parametrize('text', [
    'not json',
    _profile('ESTIMATED', reviewed=False),
    _profile('REVIEWED'),
    _profile(control_chain_hash='0' * 64),
    _profile(geometry_hash='1' * 64),
])
def test_only_accepted_profiles_for_this_robot_pass(text):
    with pytest.raises(ValueError):
        _checker().check(text, PARAMETERS)


def test_repository_accepted_profile_is_still_valid():
    # Guards against silently changing geometry/extrinsics/control without
    # re-reviewing the motion profile.
    profile = (PROJECT_DIR / 'docs/evidence/'
               'issue13_motion_profile_accepted_2026-10-06.json')
    result = subprocess.run(
        ['python3', str(SCRIPTS / 'check_motion_profile.py'), str(profile)],
        capture_output=True, text=True, check=False)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['profile_hash']


def test_start_script_gates_guarded_rotation():
    source = (SCRIPTS / 'start_real_robot_navigation_rviz.sh').read_text()
    for flag in ('--localization-motion)', '--motion-profile)',
                 '--operator-rotation-clear)', '--operator-present)'):
        assert flag in source
    assert 'requires --localization-strategy segmented_rotation' in source
    assert 'if [[ "${operator_present}" != "true" ]]; then' in source
    assert 'check_motion_profile.py' in source
    # The profile check happens before anything starts on the robot.
    assert source.index('check_motion_profile.py') < source.index(
        'echo "[1/8] Disabling the conflicting web teleop publisher..."')
    assert 'remote_sum="$(remote sha256sum' in source
    assert 'verify_motion_guard || return 1' in source
    assert "grep -q 'motion guard: permitted=True'" in source
    assert 'A motion guard is running although motion is forbidden.' in source


def test_lost_start_response_falls_back_to_manager_state():
    # 2026-10-07: the start request took effect but its response was lost,
    # and the launcher tore down a localization already in progress.
    source = (SCRIPTS / 'start_real_robot_navigation_rviz.sh').read_text()
    assert 'response was lost, but the manager is already in' in source
    assert '"${state}" == "WAIT_FOR_START"' in source


def test_force_probe_once_is_plumbed_as_a_guarded_validation_test():
    start = (SCRIPTS / 'start_real_robot_navigation_rviz.sh').read_text()
    container = (SCRIPTS / 'jetson_nvblox_container.sh').read_text()
    assert '--force-probe-once is a validation-only test' in start
    assert 'force_probe_once was requested but the manager' in start
    assert "force-probe-once is a validation-only test; use initialization 'auto'." in container
    assert 'motion_launch_argument+=" force_probe_once:=true"' in container
    launch = (PROJECT_DIR / 'src/isaac_3d_lidar_bringup/launch/'
              'carbot_navigation_real.launch.py').read_text()
    block = launch.split("'force_probe_once',", 1)[1]
    assert block.lstrip().startswith("default_value='false'")


def test_container_script_passes_guarded_arguments_only_when_valid():
    source = (SCRIPTS / 'jetson_nvblox_container.sh').read_text()
    assert 'motion_policy guarded is valid only for segmented_rotation.' in (
        source)
    assert ('operator_rotation_clear and force-probe-once require '
            'motion_policy guarded.') in source
    assert 'motion_policy:=guarded motion_profile_path:=' in source
    assert 'Motion profile must be inside ${workspace}' in source
    assert '^[0-9a-f]{64}$' in source
    # Forbid (the default) adds no motion launch arguments.
    assert 'motion_policy="${7:-forbid}"' in source


@pytest.mark.parametrize('script', [
    'start_real_robot_navigation_rviz.sh', 'jetson_nvblox_container.sh'])
def test_scripts_parse(script):
    subprocess.run(['bash', '-n', str(SCRIPTS / script)], check=True)


def test_robot_not_moved_is_plumbed_as_a_per_launch_attestation():
    start = (SCRIPTS / 'start_real_robot_navigation_rviz.sh').read_text()
    container = (SCRIPTS / 'jetson_nvblox_container.sh').read_text()
    launch = (PROJECT_DIR / 'src/isaac_3d_lidar_bringup/launch/'
              'carbot_navigation_real.launch.py').read_text()
    assert '--robot-not-moved)' in start
    assert ('--robot-not-moved requires --localization-strategy '
            'stationary_only or segmented_rotation.') in start
    assert 'robot_not_moved was requested but the manager' in start
    assert '"${robot_not_moved}" "${surface_recheck}" >/dev/null' in start
    assert 'robot_not_moved="${13:-false}"' in container
    assert 'auto_launch_argument+=" robot_not_moved:=true"' in container
    block = launch.split("'robot_not_moved',", 1)[1]
    assert block.lstrip().startswith("default_value='false'")
    assert '/workspaces/isaac_ros-dev/state/carbot_last_pose.json' in launch


def test_surface_recheck_is_plumbed_with_record_as_the_default():
    start = (SCRIPTS / 'start_real_robot_navigation_rviz.sh').read_text()
    container = (SCRIPTS / 'jetson_nvblox_container.sh').read_text()
    launch = (PROJECT_DIR / 'src/isaac_3d_lidar_bringup/launch/'
              'carbot_navigation_real.launch.py').read_text()
    assert 'surface_recheck=record' in start
    assert '--surface-recheck)' in start
    assert 'surface_recheck decide was requested but the manager reports' in start
    assert '"${robot_not_moved}" "${surface_recheck}" >/dev/null' in start
    assert 'surface_recheck="${14:-record}"' in container
    assert 'auto_launch_argument+=" surface_recheck_policy:=${surface_recheck}"' in container
    block = launch.split("'surface_recheck_policy',", 1)[1]
    assert block.lstrip().startswith("default_value='record'")
    assert "'.rsplit('.', 1)[0] + '.ply'" in launch
