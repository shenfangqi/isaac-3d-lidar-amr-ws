"""The near-field channel is opt-in and only adds a local-costmap source."""

from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = PACKAGE_DIR.parents[1]


def _read(path):
    return path.read_text(encoding='utf-8')


def test_near_field_is_opt_in():
    source = _read(PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py')
    assert ("'near_field_scan',\n"
            "            default_value='false',") in source
    assert 'OpaqueFunction(function=configure_near_field)' in source
    assert "executable='near_field_scan'" in source
    assert 'condition=IfCondition(near_field),' in source
    near = source.split('def configure_near_field', 1)[1]
    near = near.split('# Issue #1 preview', 1)[0]
    assert 'parameters=[navigation_config, near_field_config],' in near
    recovery = source.split('def configure_recovery', 1)[1]
    assert 'parameters=[navigation_config, overlay, *near],' in recovery


def test_near_field_only_adds_a_local_costmap_source():
    overlay = yaml.safe_load(_read(
        PACKAGE_DIR / 'config/nav2/carbot_near_field_real.yaml'))
    assert set(overlay) == {'local_costmap', 'near_field_scan'}
    local = overlay['local_costmap']['local_costmap']['ros__parameters']
    assert set(local) == {'obstacle_layer'}
    layer = local['obstacle_layer']
    assert layer['observation_sources'] == 'scan scan_near'
    # /scan keeps its configuration from carbot_navigation_real.yaml.
    assert set(layer) == {'observation_sources', 'scan_near'}
    near = layer['scan_near']
    assert near['topic'] == '/scan_near'
    assert near['obstacle_max_range'] == 0.6
    assert near['raytrace_max_range'] == 0.65


def test_near_field_footprint_and_band_match_navigation():
    overlay = yaml.safe_load(_read(
        PACKAGE_DIR / 'config/nav2/carbot_near_field_real.yaml'))
    node = overlay['near_field_scan']['ros__parameters']
    navigation = yaml.safe_load(_read(
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml'))
    local = navigation['local_costmap']['local_costmap']['ros__parameters']
    assert yaml.safe_load(node['footprint']) == yaml.safe_load(
        local['footprint'])
    launch = _read(PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py')
    assert "'min_height': 0.10," in launch
    assert "'max_height': 0.35," in launch
    assert (node['min_height_m'], node['max_height_m']) == (0.10, 0.35)


def test_near_field_never_commands_velocity():
    source = _read(
        PACKAGE_DIR / 'isaac_3d_lidar_bringup' / 'near_field_scan.py')
    assert 'Twist' not in source
    assert 'cmd_vel' not in source


def test_robot_scripts_forward_the_near_field_flag():
    container = _read(PROJECT_DIR / 'scripts/jetson_nvblox_container.sh')
    start = _read(
        PROJECT_DIR / 'scripts/start_real_robot_navigation_rviz.sh')
    assert 'if [[ "${19:-}" == "near-field" ]]; then' in container
    assert 'near_field_launch_argument=" near_field_scan:=true"' in container
    assert ('${keepoff_launch_argument}${near_field_launch_argument}'
            in container)
    assert 'near_field=false' in start
    assert '--near-field)' in start
    assert 'echo near-field)" >/dev/null' in start
