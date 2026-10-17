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
    assert source.count('condition=IfCondition(keepoff_preview)') == 3
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
    assert 'keepoff-preview requires navigation-safe mode.' in container
    assert ('keepoff_launch_argument=" '
            'novel_obstacle_keepoff_preview:=true"') in container
    assert '${diagnostic_launch_argument}${keepoff_launch_argument}' in (
        container)
    assert 'keepoff_preview=false' in start
    assert '--keepoff-preview)' in start
    assert 'echo keepoff-preview)' in start
