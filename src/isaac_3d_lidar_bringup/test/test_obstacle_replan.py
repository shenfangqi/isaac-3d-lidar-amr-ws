"""Issue #1: opt-in failure-triggered global detour for the real robot."""

import copy
from pathlib import Path
import xml.etree.ElementTree as ElementTree

import pytest
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
TREE = (
    PACKAGE_DIR / 'behavior_trees'
    / 'navigate_to_pose_replan_after_controller_failure_real.xml')
OVERLAY = PACKAGE_DIR / 'config/nav2/carbot_obstacle_replan_real.yaml'
NAVIGATION = PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml'
LAUNCH = PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py'


def _load_yaml(path):
    with path.open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _main_tree():
    root = ElementTree.parse(TREE).getroot()
    tree = root.find("BehaviorTree[@ID='MainTree']")
    assert tree is not None
    assert len(tree) == 1
    return tree[0]


def _merge(base, overlay):
    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


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
        if tag in ('ComputePathToPose', 'Wait', 'ClearEntireCostmap'):
            return True
        raise AssertionError(f'unexpected node {tag}')


@pytest.mark.parametrize('follow_results, plans, follows, success', [
    # Clear route: the initial path is kept.
    ([True], 1, 1, True),
    # Transient obstacle: wait, then the unchanged path succeeds.
    ([False, True], 1, 2, True),
    # Persistent obstacle with a side route: exactly one detour.
    ([False, False, True], 2, 3, True),
    # The detour path also gets one transient-pause retry (Nav2 resets the
    # inner RecoveryNode when it is ticked again).
    ([False, False, False, True], 2, 4, True),
    # Fully blocked: the tree terminates instead of looping.
    ([False, False, False, False], 2, 4, False),
])
def test_tree_plans_at_most_one_detour(follow_results, plans, follows, success):
    run = _Run(follow_results)

    assert run.tick(_main_tree()) is success
    assert run.calls.count('ComputePathToPose') == plans
    assert run.calls.count('FollowPath') == follows
    assert run.follow_results == []
    assert 'ClearEntireCostmap' not in run.calls


def test_tree_has_no_replan_loop_or_motion_recovery():
    names = {element.tag for element in _main_tree().iter()}

    assert names <= {'Sequence', 'RecoveryNode', 'ComputePathToPose',
                     'ClearEntireCostmap', 'FollowPath', 'Wait'}
    for element in _main_tree().iter('RecoveryNode'):
        assert element.get('number_of_retries') == '1'
    for element in _main_tree().iter('ComputePathToPose'):
        assert element.get('goal') == '{goal}'
        assert element.get('planner_id') == 'GridBased'


def test_costmaps_are_never_cleared_after_the_robot_has_moved():
    # The real /scan cannot see the 0.5 m around the LiDAR where RPP stops;
    # a cleared obstacle there would never be marked again.
    initial, follow = list(_main_tree())

    assert initial.get('name') == 'ComputeInitialPath'
    assert follow.get('name') == 'FollowAndReplanOnPersistentFailure'
    assert list(follow.iter('ClearEntireCostmap')) == []
    cleared = [element.get('service_name')
               for element in initial.iter('ClearEntireCostmap')]
    assert cleared == ['global_costmap/clear_entirely_global_costmap']


def test_overlay_only_feeds_filtered_obstacles_to_the_global_costmap():
    base = _load_yaml(NAVIGATION)
    overlay = _load_yaml(OVERLAY)
    merged = _merge(base, overlay)

    global_layer = merged['global_costmap']['global_costmap'][
        'ros__parameters']['obstacle_layer']
    assert global_layer['enabled'] is True
    assert global_layer['scan']['topic'] == '/scan_global_filtered'
    assert global_layer['scan']['clearing'] is True
    assert global_layer['scan']['marking'] is True

    # Everything else, including the raw-scan local costmap, footprint and
    # inflation, is the reviewed real configuration.
    expected = copy.deepcopy(base)
    expected['global_costmap']['global_costmap']['ros__parameters'][
        'obstacle_layer']['enabled'] = True
    expected['global_costmap']['global_costmap']['ros__parameters'][
        'obstacle_layer']['scan']['topic'] = '/scan_global_filtered'
    expected['static_map_scan_filter'] = overlay['static_map_scan_filter']
    expected['global_scan_filter'] = overlay['global_scan_filter']
    assert merged == expected
    local = merged['local_costmap']['local_costmap']['ros__parameters']
    assert local['obstacle_layer']['scan']['topic'] == '/scan'


def test_global_scan_chain_matches_the_launch_remappings():
    overlay = _load_yaml(OVERLAY)
    static = overlay['static_map_scan_filter']['ros__parameters']
    median = overlay['global_scan_filter']['ros__parameters']['filter1']
    source = LAUNCH.read_text(encoding='utf-8')

    assert static['use_sim_time'] is False
    assert static['input_topic'] == '/scan'
    assert static['output_topic'] == '/scan_global_static_filtered'
    assert static['map_frame'] == 'map'
    assert median['type'] == 'laser_filters/LaserArrayFilter'
    for chain in ('range_filter_chain', 'intensity_filter_chain'):
        assert median['params'][chain]['filter1']['params'][
            'number_of_observations'] == 5
    assert ("remappings=[('scan', '/scan_global_static_filtered'),\n"
            "                             ('scan_filtered', "
            "'/scan_global_filtered')]") in source


def test_launch_keeps_obstacle_replan_opt_in_and_exclusive():
    source = LAUNCH.read_text(encoding='utf-8')
    setup = (PACKAGE_DIR / 'setup.py').read_text(encoding='utf-8')

    assert "'obstacle_replan',\n            default_value='false'," in source
    assert 'condition=UnlessCondition(obstacle_replan)' in source
    assert ("\"'\", recovery_enabled, \"' == 'false' and '\",\n"
            "            obstacle_replan, \"' == 'false'\"") in source
    assert "if obstacle_replan.perform(context) != 'true':" in source
    assert "if recovery_enabled.perform(context) == 'true':" in source
    assert 'OpaqueFunction(function=configure_obstacle_replan)' in source
    assert "'default_nav_to_pose_bt_xml':\n" in source
    assert 'obstacle_replan_tree}' in source
    assert "glob('behavior_trees/*.xml')" in setup
