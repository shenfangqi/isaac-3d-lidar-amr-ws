"""Regression tests for the deliberately conservative real-robot Nav2 stack."""

import ast
from pathlib import Path

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SOURCE_DIR = PACKAGE_DIR.parent
PROJECT_DIR = SOURCE_DIR.parent


def _load_yaml(path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def test_real_nav_limits_are_conservative():
    nav = _load_yaml(
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml')

    controller = nav['controller_server']['ros__parameters']['FollowPath']
    assert controller['desired_linear_vel'] <= 0.10
    assert controller['rotate_to_heading_angular_vel'] == 0.50
    # At 20 Hz the requested turn must cross the measured 0.40 rad/s track
    # deadband in one RPP cycle.  The final smoother, not RPP's odometry-based
    # clamp, owns the physical output ramp.
    assert controller['max_angular_accel'] == 10.00
    assert controller['use_collision_detection'] is True

    smoother = nav['velocity_smoother']['ros__parameters']
    assert smoother['max_velocity'] == [0.10, 0.0, 0.50]
    assert smoother['min_velocity'] == [-0.10, 0.0, -0.50]
    # The final smoother must cross the turn deadband promptly and follow the
    # stop command in either direction; the former +/-0.50 limits added 5--8
    # degrees of measured Spin overshoot after the action had succeeded.
    assert smoother['max_accel'] == [0.20, 0.0, 2.00]
    assert smoother['max_decel'] == [-0.20, 0.0, -2.00]
    behavior = nav['behavior_server']['ros__parameters']
    assert behavior['min_rotational_vel'] == 0.40
    assert behavior['max_rotational_vel'] == 0.50
    assert behavior['rotational_acc_lim'] == 0.50
    assert smoother['velocity_timeout'] <= 0.5


def test_real_nav_footprint_matches_canonical_parameters():
    canonical_path = (
        SOURCE_DIR / 'carbot_description/config/carbot_parameters.yaml')
    if not canonical_path.is_file():
        pytest.skip('partial deployment has no carbot_description source tree')

    nav = _load_yaml(
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml')
    canonical = _load_yaml(canonical_path)
    expected_footprint = canonical['geometry']['footprint_m']
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    global_ = nav['global_costmap']['global_costmap']['ros__parameters']
    assert ast.literal_eval(local['footprint']) == expected_footprint
    assert ast.literal_eval(global_['footprint']) == expected_footprint


def test_real_navigation_launch_is_inactive_and_has_one_final_velocity_path():
    launch_source = (
        PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py'
    ).read_text(encoding='utf-8')

    assert "default_value='false'" in launch_source
    assert launch_source.count("('cmd_vel', 'cmd_vel_nav')") == 3
    assert launch_source.count(
        "('cmd_vel_smoothed', cmd_vel_output)") == 1
    assert "default_value='/cmd_vel_command'" in launch_source
    assert 'Use /cmd_vel_diagnostic' in launch_source
    assert "'autostart': autostart" in launch_source


def test_jetson_startup_orders_localization_pose_then_navigation():
    start_path = PROJECT_DIR / 'scripts/jetson_navigation_start.sh'
    initializer_path = (
        PROJECT_DIR / 'scripts/jetson_navigation_initialize.py')
    if not start_path.is_file() or not initializer_path.is_file():
        pytest.skip('partial deployment has no repository-level Jetson scripts')

    start_source = start_path.read_text(encoding='utf-8')
    initializer_source = initializer_path.read_text(encoding='utf-8')
    assert 'jetson_nav_preflight.sh' in start_source
    assert 'jetson_navigation_initialize.py' in start_source
    assert 'no goal was sent' in start_source
    assert initializer_source.index('call_manager(node, LOCALIZATION_MANAGER') < (
        initializer_source.index("message.header.frame_id = 'map'"))
    assert initializer_source.index("message.header.frame_id = 'map'") < (
        initializer_source.index('call_manager(node, NAVIGATION_MANAGER'))
    assert "'map', 'odom'" in initializer_source
    assert 'LIFECYCLE_NODES' in initializer_source
