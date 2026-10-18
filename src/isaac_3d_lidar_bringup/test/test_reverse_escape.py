"""Reverse escape: collision by cell centres, ignoring cells under the start pose."""

import math
from pathlib import Path
import xml.etree.ElementTree as ElementTree

from isaac_3d_lidar_bringup.reverse_escape import (
    first_blocked_distance, footprint_cells, LETHAL)
import numpy as np
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
FOOTPRINT = [(0.155, 0.133), (0.155, -0.133), (-0.130, -0.133),
             (-0.130, 0.133)]
# A 4 m x 4 m local costmap at 5 cm, robot at the centre.
GRID = {'resolution': 0.05, 'origin_x': -2.0, 'origin_y': -2.0,
        'width': 80, 'height': 80, 'frame': 'map'}


def _costs():
    return np.zeros((GRID['height'], GRID['width']), dtype=np.uint8)


def _mark(costs, x, y):
    row = int(math.floor((y - GRID['origin_y']) / GRID['resolution']))
    col = int(math.floor((x - GRID['origin_x']) / GRID['resolution']))
    costs[row, col] = LETHAL


def test_footprint_cells_follow_the_rotated_polygon():
    straight = footprint_cells(GRID, (0.0, 0.0, 0.0), FOOTPRINT)
    # 0.285 m x 0.266 m: 6 x 6 cell centres inside, or 5 where an edge
    # falls short of a centre.
    assert 25 <= len(straight) <= 36
    turned = footprint_cells(GRID, (0.0, 0.0, math.pi / 2), FOOTPRINT)
    rows = {row for row, _ in turned}
    cols = {col for _, col in turned}
    # Rotated by 90 deg the long axis lies along y.
    assert len(rows) >= len(cols)


def test_side_obstacle_next_to_the_body_does_not_block_backing_up():
    # The robot stopped 2.5 cm from a wall on its left, facing +x: Nav2's
    # BackUp reports a collision at the start pose, but backing straight up
    # moves no cell centre of the wall into the footprint.
    costs = _costs()
    for x in np.arange(-0.6, 0.6, 0.05):
        _mark(costs, x, 0.133 + 0.025 + 0.001)
    pose = (0.0, 0.0, 0.0)
    assert first_blocked_distance(
        GRID, costs, pose, pose, FOOTPRINT, 0.13) is None


def test_obstacle_behind_blocks_at_its_distance():
    costs = _costs()
    for y in np.arange(-0.2, 0.2, 0.05):
        _mark(costs, -0.130 - 0.08, y)
    pose = (0.0, 0.0, 0.0)
    blocked = first_blocked_distance(GRID, costs, pose, pose, FOOTPRINT, 0.13)
    # The wall's cell centres lie 0.055-0.105 m behind the rear edge.
    assert blocked is not None
    assert 0.03 <= blocked <= 0.11


def test_cells_already_under_the_robot_are_ignored():
    costs = _costs()
    _mark(costs, 0.10, 0.10)  # a stale or mislocalised cell under the body
    pose = (0.0, 0.0, 0.0)
    assert (int(math.floor((0.10 + 2.0) / 0.05)),
            int(math.floor((0.10 + 2.0) / 0.05))) in footprint_cells(
                GRID, pose, FOOTPRINT)
    assert first_blocked_distance(
        GRID, costs, pose, pose, FOOTPRINT, 0.13) is None


def test_sweep_is_measured_from_the_current_pose():
    costs = _costs()
    for y in np.arange(-0.2, 0.2, 0.05):
        _mark(costs, -0.130 - 0.18, y)
    start = (0.0, 0.0, 0.0)
    assert first_blocked_distance(
        GRID, costs, start, start, FOOTPRINT, 0.10) is None
    moved = (-0.10, 0.0, 0.0)
    blocked = first_blocked_distance(
        GRID, costs, start, moved, FOOTPRINT, 0.10)
    # The wall's cell centres are at x = -0.325, 0.095 m behind the moved
    # rear edge.
    assert blocked is not None and 0.08 <= blocked <= 0.10


def test_forward_sweep_checks_the_space_ahead_only():
    # Rear against a wall (forward fallback case): ahead is clear, behind
    # is blocked within the 0.05 m minimum progress (cell centres 4.5 cm
    # behind the rear edge).
    costs = _costs()
    for y in np.arange(-0.2, 0.2, 0.05):
        _mark(costs, -0.130 - 0.03, y)
    pose = (0.0, 0.0, 0.0)
    assert first_blocked_distance(
        GRID, costs, pose, pose, FOOTPRINT, 0.13, direction=-1.0) <= 0.05
    assert first_blocked_distance(
        GRID, costs, pose, pose, FOOTPRINT, 0.13, direction=1.0) is None
    # An obstacle ahead stops the forward move at its distance.
    for y in np.arange(-0.2, 0.2, 0.05):
        _mark(costs, 0.155 + 0.08, y)
    blocked = first_blocked_distance(
        GRID, costs, pose, pose, FOOTPRINT, 0.13, direction=1.0)
    assert blocked is not None and 0.03 <= blocked <= 0.11


def test_escape_tree_calls_reverse_escape_after_a_controller_failure():
    tree = ElementTree.parse(
        PACKAGE_DIR / 'behavior_trees/navigate_to_pose_backup_escape_real.xml')
    recovery = tree.find('.//RecoveryNode')
    assert recovery.get('number_of_retries') == '2'
    follow, escape = list(recovery)
    assert follow.tag == 'FollowPath'
    backup, replan = list(escape)
    assert backup.tag == 'BackUp'
    assert backup.get('server_name') == 'reverse_escape'
    assert float(backup.get('backup_dist')) == 0.10
    assert float(backup.get('backup_speed')) == 0.05
    assert replan.tag == 'ComputePathToPose'
    # A planner failure aborts without backing up: the first planner call
    # sits outside the recovery.
    assert tree.find('.//RateController//ComputePathToPose') is not None
    assert not tree.findall('.//ClearEntireCostmap')
    assert not tree.findall('.//Spin')


def test_launch_runs_the_escape_by_default_with_the_default_tree_only():
    source = (PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py').read_text(
        encoding='utf-8')
    block = source.split("DeclareLaunchArgument(\n            'backup_escape',", 1)[1]
    assert "default_value='true'," in block.split('),', 1)[0]
    escape = source.split('bt_navigator_escape = Node(', 1)[1].split(')\n', 1)[0]
    assert "'default_nav_to_pose_bt_xml': backup_escape_tree" in escape
    node = source.split('reverse_escape = Node(', 1)[1].split('    )\n', 1)[0]
    assert "remappings=[('cmd_vel', 'cmd_vel_nav')]" in node
    # The detour tree calls reverse_escape as well.
    assert "obstacle_detour, \"' == 'true'\"" in node
    navigation = yaml.safe_load((
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml').read_text(
            encoding='utf-8'))
    behavior = navigation['behavior_server']['ros__parameters']
    # The tree targets reverse_escape; Nav2's own backup plugin is unchanged.
    assert 'backup' in behavior['behavior_plugins']
