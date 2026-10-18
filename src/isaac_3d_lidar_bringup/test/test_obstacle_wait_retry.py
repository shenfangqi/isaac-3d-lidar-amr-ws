"""Issue #1: opt-in wait-and-retry after a FollowPath failure (real robot)."""

from pathlib import Path
import xml.etree.ElementTree as ElementTree

import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
TREE = PACKAGE_DIR / 'behavior_trees/navigate_to_pose_wait_and_retry_real.xml'
DETOUR_TREE = PACKAGE_DIR / 'behavior_trees/navigate_to_pose_detour_real.xml'
LAUNCH = PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py'


def _main_tree(path=TREE):
    root = ElementTree.parse(path).getroot()
    tree = root.find("BehaviorTree[@ID='MainTree']")
    assert tree is not None
    assert len(tree) == 1
    return tree[0]


class _Run:
    """Tick the tree once to completion against scripted action results."""

    def __init__(self, follow_results):
        self.follow_results = list(follow_results)
        self.calls = []

    def tick(self, node):
        tag = node.tag
        if tag == 'Sequence':
            return all(self.tick(child) for child in node)
        if tag == 'RecoveryNode':
            body, recovery = node
            retries = int(node.get('number_of_retries'))
            for attempt in range(retries + 1):
                if self.tick(body):
                    return True
                if attempt == retries or not self.tick(recovery):
                    return False
            return False
        self.calls.append(tag)
        if tag == 'FollowPath':
            return self.follow_results.pop(0)
        if tag in ('ComputePathToPose', 'Wait'):
            return True
        raise AssertionError(f'unexpected node {tag}')


@pytest.mark.parametrize('follow_results, follows, success', [
    # Clear route.
    ([True], 1, True),
    # Transient obstacle: wait, then the unchanged path succeeds.
    ([False, True], 2, True),
    # Persistent obstacle: the goal aborts with the robot stopped.
    ([False, False], 2, False),
])
def test_tree_retries_the_unchanged_path_once(follow_results, follows, success):
    run = _Run(follow_results)

    assert run.tick(_main_tree()) is success
    assert run.calls.count('ComputePathToPose') == 1
    assert run.calls.count('FollowPath') == follows
    assert run.follow_results == []


def test_tree_has_no_detour_replan_loop_or_motion_recovery():
    # Isaac acceptance showed a detour is unsafe with the 0.5 m blind zone;
    # clearing a costmap there would also forget a stopped-at obstacle.
    names = [element.tag for element in _main_tree().iter()]

    assert set(names) == {'Sequence', 'ComputePathToPose', 'RecoveryNode',
                          'FollowPath', 'Wait'}
    assert names.count('ComputePathToPose') == 1
    retry = _main_tree().find('RecoveryNode')
    assert retry.get('number_of_retries') == '1'
    assert retry.find('Wait').get('wait_duration') == '2'
    plan = _main_tree().find('ComputePathToPose')
    assert plan.get('goal') == '{goal}'
    assert plan.get('planner_id') == 'GridBased'


def test_launch_keeps_obstacle_wait_retry_opt_in_and_exclusive():
    source = LAUNCH.read_text(encoding='utf-8')
    setup = (PACKAGE_DIR / 'setup.py').read_text(encoding='utf-8')

    assert "'obstacle_wait_retry',\n            default_value='false'," in source
    # The default bt_navigator (with or without the reverse escape) yields
    # to every alternative tree.
    assert ("    default_tree = PythonExpression([\n"
            "        \"'\", recovery_enabled, \"' == 'false' and '\",\n"
            "        obstacle_wait_retry, \"' == 'false' and '\",\n"
            "        obstacle_detour, \"' == 'false'\"])") in source
    assert source.count('default_tree, " and \'", backup_escape') == 3
    assert "if obstacle_wait_retry.perform(context) != 'true':" in source
    assert "if recovery_enabled.perform(context) == 'true':" in source
    assert 'OpaqueFunction(function=configure_obstacle_wait_retry)' in source
    assert "'default_nav_to_pose_bt_xml':\n" in source
    assert 'obstacle_wait_retry_tree}' in source
    # Only the tree changes: wait-and-retry starts no global obstacle chain.
    # (The keep-off chain exists only behind novel_obstacle_keepoff_preview.)
    wait_retry = source.split('def configure_obstacle_wait_retry', 1)[1]
    wait_retry = wait_retry.split('return LaunchDescription', 1)[0]
    assert 'static_map_scan_filter' not in wait_retry
    assert 'novel_obstacle_keepoff' not in wait_retry
    assert 'scan_to_scan_filter_chain' not in source
    assert "glob('behavior_trees/*.xml')" in setup


@pytest.mark.parametrize('follow_results, plans, follows, success', [
    ([True], 1, 1, True),
    # Transient obstacle: no detour.
    ([False, True], 1, 2, True),
    # Persistent obstacle: one detour.
    ([False, False, True], 2, 3, True),
    # Detours are bounded: three, each with one transient-pause retry.
    ([False] * 8, 4, 8, False),
])
def test_detour_tree_bounds_its_replans(follow_results, plans, follows, success):
    run = _Run(follow_results)

    assert run.tick(_main_tree(DETOUR_TREE)) is success
    assert run.calls.count('ComputePathToPose') == plans
    assert run.calls.count('FollowPath') == follows
    assert run.follow_results == []


def test_detour_tree_never_clears_costmaps_or_moves_on_its_own():
    names = {element.tag for element in _main_tree(DETOUR_TREE).iter()}

    assert names == {'Sequence', 'ComputePathToPose', 'RecoveryNode',
                     'FollowPath', 'Wait'}
    budget = _main_tree(DETOUR_TREE).find('RecoveryNode')
    assert budget.get('name') == 'DetourBudget'
    assert budget.get('number_of_retries') == '3'
