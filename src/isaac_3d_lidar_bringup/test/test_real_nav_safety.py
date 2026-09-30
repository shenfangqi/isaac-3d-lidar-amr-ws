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
    controller_server = nav['controller_server']['ros__parameters']
    goal_checker = nav['controller_server']['ros__parameters'][
        'general_goal_checker']
    assert controller['desired_linear_vel'] <= 0.10
    assert goal_checker['yaw_goal_tolerance'] == 0.03
    assert controller['rotate_to_heading_angular_vel'] == 0.50
    # The requested turn must cross the measured 0.40 rad/s track deadband in
    # one RPP cycle. The final smoother owns the physical output ramp.
    assert controller['max_angular_accel'] == 10.00
    assert controller['use_collision_detection'] is True
    assert controller_server['controller_frequency'] == 10.0
    assert controller['transform_tolerance'] >= 0.7
    assert controller['lookahead_dist'] <= 0.15

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


def test_real_amcl_does_not_scan_match_stationary_noise():
    amcl = _load_yaml(
        PACKAGE_DIR / 'config/nav2/carbot_amcl_real.yaml'
    )['amcl']['ros__parameters']

    assert amcl['update_min_d'] >= 0.05
    assert amcl['update_min_a'] >= 0.05
    assert amcl['transform_tolerance'] >= 1.5


def test_legacy_real_profiles_keep_the_same_tf_timing_guards():
    nav = _load_yaml(PROJECT_DIR / 'configs/nav2_params_real.yaml')
    amcl = _load_yaml(PROJECT_DIR / 'configs/amcl_params_real.yaml')
    controller = nav['controller_server']['ros__parameters']
    follow_path = controller['FollowPath']
    amcl_params = amcl['amcl']['ros__parameters']

    assert controller['controller_frequency'] == 10.0
    assert follow_path['transform_tolerance'] >= 0.7
    assert amcl_params['update_min_d'] >= 0.05
    assert amcl_params['update_min_a'] >= 0.05


def test_real_local_costmap_retains_mapped_and_recent_table_legs():
    nav = _load_yaml(
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml')
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    scan = local['obstacle_layer']['scan']
    planner = nav['planner_server']['ros__parameters']['GridBased']

    assert local['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer']
    assert local['static_layer']['map_subscribe_transient_local'] is True
    assert scan['observation_persistence'] >= 2.0
    assert planner['tolerance'] <= 0.15
    assert planner['plugin'] == 'nav2_smac_planner/SmacPlanner2D'
    assert planner['downsample_costmap'] is False
    assert planner['smoother']['max_iterations'] > 0


def test_real_controller_and_recoveries_use_the_map_frame():
    nav = _load_yaml(
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml')
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    behavior = nav['behavior_server']['ros__parameters']

    # The saved-map global plan is in map. Keeping the local controller in
    # odom made RPP request a transform at the newest odometry timestamp while
    # AMCL's map transform was still one 10 Hz sample behind. That aborts the
    # controller and makes the BT alternate spin/backup paths.
    assert local['global_frame'] == 'map'
    assert behavior['global_frame'] == 'map'


def test_real_table_approach_has_stable_path_without_motion_recoveries():
    nav = _load_yaml(
        PACKAGE_DIR / 'config/nav2/carbot_navigation_real.yaml')
    local = nav['local_costmap']['local_costmap']['ros__parameters']
    global_ = nav['global_costmap']['global_costmap']['ros__parameters']
    bt = nav['bt_navigator']['ros__parameters']

    # 0.25 m remains outside the roughly 0.204 m circumscribed footprint while
    # preserving substantially more narrow-space clearance than 0.45 m.
    assert 0.204 < local['inflation_layer']['inflation_radius'] <= 0.25
    assert global_['inflation_layer']['inflation_radius'] == \
        local['inflation_layer']['inflation_radius']
    assert global_['obstacle_layer']['enabled'] is False
    assert local['obstacle_layer'].get('enabled', True) is True
    assert bt['default_nav_to_pose_bt_xml'].endswith(
        'navigate_w_replanning_only_if_path_becomes_invalid.xml')


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
    assert "executable='manual_map_localizer'" in launch_source
    assert "condition=UnlessCondition(automatic_localization)" in launch_source
    assert "executable='amcl'" in launch_source
    assert launch_source.count(
        'condition=IfCondition(automatic_localization)') == 3
    assert "'node_names': ['map_server']" in launch_source
    assert "'node_names': ['map_server', 'amcl']" in launch_source
    assert "'autostart': lifecycle_autostart" in launch_source
    assert "automatic_localization, \"' == 'false'\"" in launch_source
    assert "default_value='false'" in launch_source
    assert "'cmd_vel_topic': cmd_vel_output" in launch_source
    assert "'validation_only': auto_localization_validation_only" in launch_source
    assert "default_value='true'" in launch_source


def test_health_check_allows_read_only_command_monitors():
    control_source = (
        PROJECT_DIR / 'scripts/jetson_navigation_control.py'
    ).read_text(encoding='utf-8')

    assert control_source.count(
        'observed[0] != counts[0] or observed[1] < counts[1]'
    ) == 2


def test_automatic_localization_defaults_preserve_safety_gates():
    config = _load_yaml(
        PACKAGE_DIR / 'config/nav2/carbot_auto_localization_real.yaml')
    params = config['automatic_localization_manager']['ros__parameters']
    assert params['rotation_speed_rad_s'] == 0.40
    assert params['rotation_target_rad'] == pytest.approx(2.0 * 3.141592653589793)
    assert params['min_scan_beams_for_motion'] >= 10
    assert params['min_rotation_clearance_m'] >= 0.50
    assert params['rotation_obstacle_confirmation_sec'] <= 0.30
    assert params['scan_topic'] == '/scan_localization'
    assert params['safety_scan_topic'] == '/scan'
    assert params['max_amcl_xy_std'] <= 0.20
    assert params['max_amcl_yaw_std'] <= 0.15
    assert params['min_particle_concentration'] >= 0.65
    assert params['nomotion_update_service'] == '/request_nomotion_update'
    assert params['min_scan_map_score'] >= 0.65
    assert params['min_scan_map_coverage'] >= 0.65
    assert params['global_search_min_score_margin'] >= 0.10
    assert params['global_search_max_wall_conflict_ratio'] <= 0.25
    assert params['global_search_scan_count'] == 3
    assert params['global_search_timeout_sec'] >= 100.0
    assert params['quality_hold_sec'] >= params['tf_window_sec']
    assert params['candidate_recheck_grace_sec'] >= (
        params['tf_window_sec'] + params['sensor_freshness_sec']
    )
    assert params['localization_evidence_freshness_sec'] >= (
        params['tf_window_sec'] + params['quality_hold_sec']
    )
    assert params['localization_evidence_freshness_sec'] < (
        params['verification_timeout_sec']
    )

    amcl = _load_yaml(
        PACKAGE_DIR / 'config/nav2/carbot_amcl_real.yaml')
    amcl_params = amcl['amcl']['ros__parameters']
    assert amcl_params['tf_broadcast'] is True
    assert amcl_params['scan_topic'] == '/scan_localization'
    # The manager owns map-wide discovery; AMCL uses ray-consistent local
    # tracking after a tight seed without starving the scan watchdog.
    assert amcl_params['laser_model_type'] == 'beam'
    assert 400 <= amcl_params['max_particles'] <= 800
    assert 40 <= amcl_params['max_beams'] <= 90
    assert sum(amcl_params[name] for name in (
        'z_hit', 'z_rand', 'z_max', 'z_short')) == pytest.approx(1.0)

    launch_source = (
        PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py'
    ).read_text(encoding='utf-8')
    assert "name='mid360_localization_pointcloud_to_laserscan'" in launch_source
    assert "('scan', '/scan_localization')" in launch_source
    assert "'min_height': 0.22" in launch_source


def test_real_mapping_uses_fast_lio_pose_and_nvblox_map_only():
    launch_source = (
        PACKAGE_DIR / 'launch/mid360_mapping_real.launch.py'
    ).read_text(encoding='utf-8')
    assert launch_source.count("'/fast_lio/cloud_registered_body'") == 2
    assert "'global_frame': 'odom'" in launch_source
    assert 'slam_toolbox' not in launch_source


def test_real_nvblox_slice_is_floor_referenced_with_fast_lio():
    nvblox = _load_yaml(
        PACKAGE_DIR / 'config/nvblox/mid360_nvblox_real.yaml')
    mapper = nvblox['/**']['ros__parameters']['static_mapper']
    imu_height = 0.164790453526
    assert mapper['esdf_slice_height'] == pytest.approx(0.09 - imu_height)
    assert mapper['esdf_slice_min_height'] == pytest.approx(0.09 - imu_height)
    assert mapper['esdf_slice_max_height'] == pytest.approx(0.35 - imu_height)


def test_real_lio_is_tightly_coupled_and_has_one_tf_owner():
    lio = _load_yaml(
        PACKAGE_DIR / 'config/state_estimation/fast_lio_mid360.yaml')
    params = lio['/**']['ros__parameters']
    assert params['common']['lid_topic'] == '/livox/lidar'
    assert params['common']['imu_topic'] == '/mid360/imu/data_raw'
    assert params['common']['world_frame'] == 'odom'
    assert params['common']['body_frame'] == 'fast_lio_imu'
    assert params['mapping']['extrinsic_est_en'] is False
    assert params['publish']['tf_en'] is False
    assert params['preprocess']['lidar_type'] == 1

    launch_source = (
        PACKAGE_DIR / 'launch/mid360_fast_lio_odometry.launch.py'
    ).read_text(encoding='utf-8')
    assert "package='fast_lio'" in launch_source
    assert "'/fast_lio/imu_odom'" in launch_source
    assert "executable='fast_lio_base_adapter'" in launch_source

    nvblox_source = (
        PACKAGE_DIR / 'launch/mid360_nvblox_real.launch.py'
    ).read_text(encoding='utf-8')
    assert "executable='pointcloud_padder'" in nvblox_source
    assert "('pointcloud', nvblox_pointcloud_topic)" in nvblox_source

    mapping_source = (
        PACKAGE_DIR / 'launch/mid360_mapping_real.launch.py'
    ).read_text(encoding='utf-8')
    navigation_source = (
        PACKAGE_DIR / 'launch/carbot_navigation_real.launch.py'
    ).read_text(encoding='utf-8')
    assert 'mid360_fast_lio_odometry.launch.py' in mapping_source
    assert 'mid360_fast_lio_odometry.launch.py' in navigation_source


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


def test_automatic_localization_is_armed_only_after_preflight():
    start_source = (
        PROJECT_DIR / 'scripts/jetson_navigation_start.sh'
    ).read_text(encoding='utf-8')
    container_source = (
        PROJECT_DIR / 'scripts/jetson_nvblox_container.sh'
    ).read_text(encoding='utf-8')
    manager_source = (
        PACKAGE_DIR / 'isaac_3d_lidar_bringup'
        / 'automatic_localization_manager.py'
    ).read_text(encoding='utf-8')

    preflight = start_source.index('jetson_nav_preflight.sh')
    arm = start_source.index('/automatic_localization/start')
    assert preflight < arm
    assert 'automatic_localization:=true' in container_source
    assert container_source.count(
        'Automatic physical localization requires navigation-safe mode.'
    ) == 1
    assert start_source.count(
        'Automatic physical localization requires navigation-safe mode.'
    ) == 1
    assert 'State.WAIT_FOR_START' in manager_source
    assert 'ReliabilityPolicy.BEST_EFFORT' in manager_source
    assert 'self._command_publisher = None' in manager_source
    assert manager_source.index('def _on_start_request') < (
        manager_source.index(
            'self._ensure_command_publisher()',
            manager_source.index('def _on_start_request'),
        )
    )
