"""Issue #1: the keep-off chain runs on the robot for observation only."""

from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = PACKAGE_DIR.parents[1]


def _read(path):
    return path.read_text(encoding='utf-8')


def test_keepoff_preview_is_opt_in_and_starts_the_whole_chain():
    source = _read(PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py')
    assert ("'novel_obstacle_keepoff_preview',\n"
            "            default_value='false',") in source
    assert source.count('condition=keepoff_chain,') == 3
    assert ("\"'\", keepoff_preview, \"' == 'true' or '\",\n"
            "        obstacle_detour, \"' == 'true'\"") in source
    for executable in ('static_map_scan_filter', 'scan_temporal_median',
                       'novel_obstacle_keepoff'):
        assert f"executable='{executable}'" in source
    assert '*keepoff_nodes,' in source
    # The temporal median feeds the keep-off node's marking topic.
    keepoff = yaml.safe_load(_read(
        PACKAGE_DIR / 'config/nav2/carbot_obstacle_detour_real.yaml'))
    assert keepoff['novel_obstacle_keepoff']['ros__parameters'][
        'marking_scan_topic'] == '/scan_global_filtered'
    assert "'output_topic': '/scan_global_filtered'" in source


def test_detour_is_opt_in_exclusive_and_keeps_the_controller():
    source = _read(PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py')
    assert ("'novel_obstacle_detour',\n"
            "            default_value='false',") in source
    assert 'OpaqueFunction(function=configure_obstacle_detour)' in source
    detour = source.split('def configure_obstacle_detour', 1)[1]
    detour = detour.split('# Issue #1: wait for a transient obstacle', 1)[0]
    assert "if obstacle_detour.perform(context) != 'true':" in detour
    assert "recovery_enabled.perform(context) == 'true'" in detour
    assert "obstacle_wait_retry.perform(context) == 'true'" in detour
    assert 'parameters=[navigation_config, keepoff_config]' in detour
    assert "{'default_nav_to_pose_bt_xml': detour_tree}" in detour
    # Only the planner and the tree change for the detour test.
    assert 'controller_server' not in detour
    assert 'velocity_smoother' not in detour
    assert 'condition=UnlessCondition(obstacle_detour),' in source
    assert "obstacle_detour, \"' == 'false'\"]))," in source
    keepoff = yaml.safe_load(_read(
        PACKAGE_DIR / 'config/nav2/carbot_obstacle_detour_real.yaml'))
    layers = keepoff['global_costmap']['global_costmap']['ros__parameters']
    assert layers['plugins'] == [
        'static_layer', 'novel_keepoff_layer', 'obstacle_layer',
        'inflation_layer']
    assert 'controller_server' not in keepoff
    assert 'local_costmap' not in keepoff


def test_real_nav2_does_not_read_the_keepoff_grid():
    navigation = _read(
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml')
    assert 'novel_obstacle_keepoff' not in navigation
    assert 'novel_keepoff_layer' not in navigation


def test_keepoff_chain_never_commands_velocity():
    for module in ('static_map_scan_filter', 'scan_temporal_median',
                   'novel_obstacle_keepoff', 'tf_wait_queue'):
        source = _read(PACKAGE_DIR / 'isaac_3d_lidar_bringup' / f'{module}.py')
        assert 'Twist' not in source
        assert 'cmd_vel' not in source


def test_robot_scripts_forward_the_preview_flag():
    container = _read(PROJECT_DIR / 'scripts/jetson_nvblox_container.sh')
    start = _read(
        PROJECT_DIR / 'scripts/start_real_robot_navigation_rviz.sh')
    assert 'if [[ "${18:-}" == "keepoff-preview" ]]; then' in container
    assert 'elif [[ "${18:-}" == "keepoff-detour" ]]; then' in container
    assert ('keepoff_launch_argument=" '
            'novel_obstacle_detour:=true"') in container
    assert 'requires navigation-safe mode.' in container
    assert ('keepoff_launch_argument=" '
            'novel_obstacle_keepoff_preview:=true"') in container
    assert '${diagnostic_launch_argument}${keepoff_launch_argument}' in (
        container)
    assert 'keepoff_mode=none' in start
    assert '--keepoff-preview)' in start
    assert '--keepoff-detour)' in start
    assert 'echo "${keepoff_mode}")" \\' in start
