"""Issue #1: opt-in wait-and-retry after a FollowPath failure (real robot)."""

from pathlib import Path
import xml.etree.ElementTree as ElementTree

import pytest
import yaml


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

    def __init__(self, follow_results, escape_succeeds=True):
        self.follow_results = list(follow_results)
        self.escape_succeeds = escape_succeeds
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
        if tag == 'ForceSuccess':
            self.tick(node[0])
            return True
        self.calls.append(tag)
        if tag == 'FollowPath':
            return self.follow_results.pop(0)
        if tag == 'BackUp':
            return self.escape_succeeds
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


class _Cancelled(Exception):
    """The operator cancelled the goal while the tree was waiting."""


class _DetourRun:
    """
    Tick the detour tree to completion on a simulated clock.

    blocked(path, t) says whether the route ahead of path number `path`
    (0 is the first plan, each successful plan adds one) crosses a keep-off
    at time t; only the Wait nodes advance t.  FollowPath pops 'goal' or
    'collision' (RPP refused to drive on).  A Wait that ends after
    cancel_at raises _Cancelled, like an operator cancel.
    """

    def __init__(self, follows=(), blocked=lambda path, t: False,
                 plans=(), escape_succeeds=True, cancel_at=None):
        root = ElementTree.parse(DETOUR_TREE).getroot()
        self.trees = {tree.get('ID'): tree[0] for tree in root}
        self.follows = list(follows)
        self.blocked = blocked
        self.plans = list(plans)
        self.escape_succeeds = escape_succeeds
        self.cancel_at = cancel_at
        self.t = 0.0
        self.path = -1
        self.calls = []
        self.waits = []

    def run(self):
        return self.tick(self.trees['MainTree'])

    def tick(self, node):
        tag = node.tag
        if tag in ('Sequence', 'ReactiveSequence'):
            return all(self.tick(child) for child in node)
        if tag == 'Fallback':
            return any(self.tick(child) for child in node)
        if tag == 'RecoveryNode':
            body, recovery = node
            retries = int(node.get('number_of_retries'))
            for attempt in range(retries + 1):
                if self.tick(body):
                    return True
                if attempt == retries or not self.tick(recovery):
                    return False
            return False
        if tag == 'RetryUntilSuccessful':
            assert node.get('num_attempts') == '-1'
            while not self.tick(node[0]):
                pass
            return True
        if tag == 'ForceSuccess':
            self.tick(node[0])
            return True
        if tag == 'Inverter':
            return not self.tick(node[0])
        if tag == 'SubTree':
            return self.tick(self.trees[node.get('ID')])
        self.calls.append(tag)
        if tag == 'TruncatePathLocal':
            return True
        if tag == 'IsPathValid':
            return not self.blocked(self.path, self.t)
        if tag == 'FollowPath':
            return self.follows.pop(0) == 'goal'
        if tag == 'BackUp':
            return self.escape_succeeds
        if tag == 'Wait':
            duration = float(node.get('wait_duration'))
            self.waits.append(duration)
            self.t += duration
            if self.cancel_at is not None and self.t > self.cancel_at:
                raise _Cancelled
            return True
        if tag == 'ComputePathToPose':
            planned = self.plans.pop(0) if self.plans else True
            if planned:
                self.path += 1
            return planned
        raise AssertionError(f'unexpected node {tag}')


def test_detour_tree_drives_a_clear_route_with_one_plan():
    run = _DetourRun(['goal'])

    assert run.run() is True
    assert run.calls.count('ComputePathToPose') == 1
    assert run.calls.count('FollowPath') == 1
    assert run.waits == []


def test_route_blocked_ahead_then_clear_continues_on_the_original_path():
    # A person crosses the route: stop, wait 3 s, keep the path.
    run = _DetourRun(['goal'], blocked=lambda path, t: t < 3)

    assert run.run() is True
    assert run.calls.count('ComputePathToPose') == 1
    assert run.path == 0
    assert run.waits == [3.0]
    assert 'BackUp' not in run.calls


def test_route_still_blocked_after_the_pause_plans_a_detour_without_escape():
    run = _DetourRun(['goal'], blocked=lambda path, t: path == 0)

    assert run.run() is True
    assert run.calls.count('ComputePathToPose') == 2
    assert run.path == 1
    assert run.waits == [3.0]
    # Stopped early by the route check, so no reverse escape.
    assert 'BackUp' not in run.calls
    # FollowPath never started on the blocked route.
    assert run.calls.count('FollowPath') == 1


def test_failed_detour_plan_keeps_waiting_until_the_original_route_clears():
    run = _DetourRun(['goal'], plans=[True, False, False],
                     blocked=lambda path, t: path == 0 and t < 9)

    assert run.run() is True
    assert run.path == 0
    assert run.calls.count('ComputePathToPose') == 3
    assert run.waits == [3.0, 3.0, 3.0]


def test_failed_detour_plan_keeps_waiting_until_a_detour_plans():
    run = _DetourRun(['goal'], plans=[True, False, False, True],
                     blocked=lambda path, t: path == 0)

    assert run.run() is True
    assert run.path == 1
    assert run.waits == [3.0, 3.0, 3.0]


def test_failed_detour_plan_waits_without_a_time_limit():
    run = _DetourRun(['goal'], plans=[True] + [False] * 1000,
                     blocked=lambda path, t: True, cancel_at=600)

    with pytest.raises(_Cancelled):
        run.run()
    assert run.calls.count('FollowPath') == 0
    assert set(run.waits) == {3.0}
    assert len(run.waits) > 200


def test_a_blocked_detour_is_a_new_detour():
    run = _DetourRun(['goal'], blocked=lambda path, t: path < 2)

    assert run.run() is True
    assert run.path == 2
    assert run.calls.count('ComputePathToPose') == 3
    assert run.waits == [3.0, 3.0]


def test_route_blocks_are_not_counted_and_never_abort():
    # Every 3 s a new detour is planned; none is limited and none escapes.
    run = _DetourRun(blocked=lambda path, t: True, cancel_at=600)

    with pytest.raises(_Cancelled):
        run.run()
    assert run.calls.count('ComputePathToPose') > 50
    assert run.calls.count('FollowPath') == 0
    assert set(run.waits) == {3.0}
    assert 'BackUp' not in run.calls


def test_route_blocks_then_clear_reach_the_goal():
    run = _DetourRun(['goal'], blocked=lambda path, t: t < 30)

    assert run.run() is True
    assert run.calls.count('ComputePathToPose') > 4
    assert run.calls.count('FollowPath') == 1
    assert 'BackUp' not in run.calls


@pytest.mark.parametrize('follows, plans, waits', [
    # Transient obstacle the keep-off does not mark: no detour.
    (['collision', 'goal'], 1, [2.0]),
    # Persistent: escape, settle, plan one detour.
    (['collision', 'collision', 'goal'], 2, [2.0, 2.0]),
])
@pytest.mark.parametrize('escape_succeeds', [True, False])
def test_controller_failure_chain_is_unchanged(follows, plans, waits,
                                               escape_succeeds):
    run = _DetourRun(follows, escape_succeeds=escape_succeeds)

    assert run.run() is True
    assert run.calls.count('ComputePathToPose') == plans
    assert run.calls.count('BackUp') == plans - 1
    assert run.waits == waits
    assert run.follows == []


@pytest.mark.parametrize('escape_succeeds', [True, False])
def test_three_escapes_then_the_goal_is_kept(escape_succeeds):
    # Three escape detours, each after a retried failure, then every 3 s
    # the current path is driven again without escaping.
    run = _DetourRun(['collision'] * 8 + ['collision', 'collision', 'goal'],
                     escape_succeeds=escape_succeeds)

    assert run.run() is True
    assert run.calls.count('ComputePathToPose') == 4
    assert run.calls.count('BackUp') == 3
    assert run.calls[-3:] == ['TruncatePathLocal', 'IsPathValid',
                              'FollowPath']
    assert run.waits[-5:] == [2.0, 2.0, 3.0, 2.0, 3.0]
    assert run.follows == []


def test_refused_forever_escapes_three_times_only():
    # A gap RPP never accepts: no rocking back and forth.
    run = _DetourRun(['collision'] * 1000, cancel_at=600)

    with pytest.raises(_Cancelled):
        run.run()
    assert run.calls.count('BackUp') == 3
    assert run.calls.count('FollowPath') > 100
    assert run.calls.count('ComputePathToPose') == 4


def test_route_blocks_leave_all_three_escapes_for_a_corner():
    # 2026-10-19 robot: a box's growing keep-off stopped the robot many
    # times; a corner later still gets its escapes.
    run = _DetourRun(['collision'] * 1000, cancel_at=600,
                     blocked=lambda path, t: t < 60)

    with pytest.raises(_Cancelled):
        run.run()
    assert run.calls.count('BackUp') == 3


def test_detour_tree_checks_the_route_ahead_while_following():
    root = ElementTree.parse(DETOUR_TREE).getroot()
    trees = {tree.get('ID'): tree[0] for tree in root}
    names = {element.tag for tree in trees.values() for element in tree.iter()}

    # Never a costmap clear, a spin or a rate-limited replanning loop.
    assert names == {
        'Sequence', 'Fallback', 'RecoveryNode', 'RetryUntilSuccessful',
        'ReactiveSequence', 'Inverter', 'ForceSuccess', 'SubTree',
        'ComputePathToPose', 'FollowPath', 'Wait', 'BackUp',
        'TruncatePathLocal', 'IsPathValid'}
    guarded = trees['FollowRoute'].find(
        ".//ReactiveSequence[@name='FollowWhileRouteAheadClear']")
    assert [child.tag for child in guarded] == ['SubTree', 'FollowPath']
    assert guarded[0].get('ID') == 'RouteAheadClear'
    truncate, check = trees['RouteAheadClear']
    assert truncate.get('input_path') == '{path}'
    assert truncate.get('output_path') == check.get('path') == '{path_ahead}'
    assert float(truncate.get('distance_backward')) == 0.0
    # Check as far ahead as the keep-off is marked.
    keepoff = yaml.safe_load((
        PACKAGE_DIR / 'config/nav2/carbot_obstacle_detour_real.yaml'
    ).read_text(encoding='utf-8'))
    assert float(truncate.get('distance_forward')) == keepoff[
        'novel_obstacle_keepoff']['ros__parameters']['mark_max_range_m']
    pause = trees['FollowRoute'].find(".//Sequence[@name='BlockedPause']")
    assert pause.find('Wait').get('wait_duration') == '3'
    for backup in trees['MainTree'].iter('BackUp'):
        assert backup.get('server_name') == 'reverse_escape'
        assert float(backup.get('backup_dist')) == 0.10
    budget = trees['MainTree'].find(".//RecoveryNode[@name='EscapeBudget']")
    assert budget.get('number_of_retries') == '3'
    assert len(list(trees['MainTree'].iter('BackUp'))) == 1
    # The route-block detour never backs away.
    assert not list(trees['DetourAroundBlocks'].iter('BackUp'))
    for loop in root.iter('RetryUntilSuccessful'):
        # Each pass waits before it can fail again, so a cancel is served.
        assert loop[0].tag == 'Sequence' and loop[0][0].tag == 'Wait'


def test_detour_costmap_radius_matches_the_footprint_inscribed_radius():
    keepoff = yaml.safe_load((
        PACKAGE_DIR / 'config/nav2/carbot_obstacle_detour_real.yaml'
    ).read_text(encoding='utf-8'))
    navigation = yaml.safe_load((
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml'
    ).read_text(encoding='utf-8'))
    detour = keepoff['global_costmap']['global_costmap']['ros__parameters']
    real = navigation['global_costmap']['global_costmap']['ros__parameters']
    polygon = yaml.safe_load(real['footprint'])
    inscribed = min(min(abs(x) for x, _ in polygon),
                    min(abs(y) for _, y in polygon))

    assert detour['footprint'] == '[]'
    assert detour['robot_radius'] == pytest.approx(inscribed)
    assert real['inflation_layer']['inflation_radius'] > detour['robot_radius']
