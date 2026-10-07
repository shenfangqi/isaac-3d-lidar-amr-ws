"""Run the on-Jetson saved-map navigation stack for the real Carbot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')
    amcl_config = os.path.join(
        bringup_dir, 'config', 'nav2', 'carbot_amcl_real.yaml')
    navigation_config = os.path.join(
        bringup_dir, 'config', 'nav2', 'carbot_navigation_real.yaml')
    auto_localization_config = os.path.join(
        bringup_dir, 'config', 'nav2',
        'carbot_auto_localization_real.yaml')
    lio_launch = os.path.join(
        bringup_dir, 'launch', 'mid360_fast_lio_odometry.launch.py')

    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    cmd_vel_output = LaunchConfiguration('cmd_vel_output')
    automatic_localization = LaunchConfiguration('automatic_localization')
    auto_localization_validation_only = LaunchConfiguration(
        'auto_localization_validation_only')
    localization_strategy = LaunchConfiguration('localization_strategy')
    recovery_enabled = LaunchConfiguration('bounded_recovery_enabled')
    acceptance_profile = LaunchConfiguration('recovery_acceptance_profile')
    recovery_validation_preview = LaunchConfiguration(
        'recovery_validation_preview')
    complex_route_validation = LaunchConfiguration(
        'complex_route_validation')
    rotation_preview = LaunchConfiguration('rotation_preview')
    motion_policy = LaunchConfiguration('motion_policy')
    motion_profile_path = LaunchConfiguration('motion_profile_path')
    extrinsics_hash = LaunchConfiguration('extrinsics_hash')
    control_chain_hash = LaunchConfiguration('control_chain_hash')
    operator_rotation_clear = LaunchConfiguration('operator_rotation_clear')
    force_probe_once = LaunchConfiguration('force_probe_once')
    probe_parameters = {
        'motion_policy': motion_policy,
        'motion_profile_path': motion_profile_path,
        'extrinsics_hash': extrinsics_hash,
        'control_chain_hash': control_chain_hash,
        'operator_rotation_clear': PythonExpression([
            "'", operator_rotation_clear, "' == 'true'"]),
    }
    complex_route_braking_profile = LaunchConfiguration(
        'complex_route_braking_profile')
    lifecycle_autostart = PythonExpression([
        "'", autostart, "' == 'true' and '",
        automatic_localization, "' == 'false'",
    ])

    lio_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lio_launch),
    )

    scan_projection = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='mid360_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/fast_lio/cloud_registered_body'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'use_sim_time': False,
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            # At 10 Hz, a deep TF filter queue can release seconds-old scans
            # after a CPU scheduling pause. Keep only a short transform grace
            # window so Nav2 and recovery guards never consume stale geometry.
            'queue_size': 3,
            'min_height': 0.10,
            # Preserve 0.11 m above the top-mounted MID-360.
            'max_height': 0.35,
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'angle_increment': 0.017453293,
            'scan_time': 0.1,
            'range_min': 0.5,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen',
    )

    # Localization uses the upper part of the saved-map obstacle slice.  The
    # broader /scan remains the safety and costmap source so low obstacles are
    # never hidden from motion checks.
    localization_scan_projection = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='mid360_localization_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/fast_lio/cloud_registered_body'),
            ('scan', '/scan_localization'),
        ],
        parameters=[{
            'use_sim_time': False,
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            'queue_size': 3,
            'min_height': 0.22,
            'max_height': 0.35,
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'angle_increment': 0.017453293,
            'scan_time': 0.1,
            'range_min': 0.5,
            'range_max': 20.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        output='screen',
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[amcl_config, {'yaml_filename': map_yaml}],
        output='screen',
    )
    manual_localizer = Node(
        package='isaac_3d_lidar_bringup',
        executable='manual_map_localizer',
        name='manual_map_localizer',
        parameters=[{
            'global_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_rate': 20.0,
        }],
        condition=UnlessCondition(automatic_localization),
        output='screen',
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[amcl_config],
        remappings=[('initialpose', '/amcl_initialpose')],
        condition=IfCondition(automatic_localization),
        output='screen',
    )
    manual_localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': False,
            'autostart': autostart,
            'node_names': ['map_server'],
        }],
        condition=UnlessCondition(automatic_localization),
        output='screen',
    )
    automatic_localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': False,
            'autostart': False,
            'node_names': ['map_server', 'amcl'],
        }],
        condition=IfCondition(automatic_localization),
        output='screen',
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[navigation_config],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        condition=UnlessCondition(recovery_enabled),
        output='screen',
    )
    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[navigation_config],
        output='screen',
    )
    behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[navigation_config],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        condition=UnlessCondition(recovery_enabled),
        output='screen',
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[navigation_config],
        condition=UnlessCondition(recovery_enabled),
        output='screen',
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[navigation_config],
        output='screen',
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[navigation_config],
        remappings=[
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', cmd_vel_output),
        ],
        output='screen',
    )
    navigation_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': False,
            'autostart': lifecycle_autostart,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }],
        output='screen',
    )
    automatic_localization_manager = Node(
        package='isaac_3d_lidar_bringup',
        executable='automatic_localization_manager',
        name='automatic_localization_manager',
        parameters=[
            auto_localization_config,
            {
                'cmd_vel_topic': cmd_vel_output,
                'validation_only': auto_localization_validation_only,
                'localization_strategy': localization_strategy,
                **probe_parameters,
                'force_probe_once': PythonExpression([
                    "'", force_probe_once, "' == 'true'"]),
            },
        ],
        condition=IfCondition(automatic_localization),
        output='screen',
    )
    recovery_preview = Node(
        package='carbot_nav_recovery',
        executable='recovery_validation_visualizer',
        name='carbot_nav_recovery_validation',
        parameters=[{
            'costmap_topic': '/local_costmap/costmap_raw',
            'marker_topic': '/carbot_nav_recovery/markers',
            'base_frame': 'base_footprint',
            'localization_valid': False,
            'footprint_xy': [0.155, 0.133, 0.155, -0.133,
                             -0.130, -0.133, -0.130, 0.133],
        }],
        condition=IfCondition(recovery_validation_preview),
        output='screen',
    )
    recovery_runtime_observer = Node(
        package='carbot_nav_recovery',
        executable='recovery_runtime_observer',
        name='carbot_recovery_runtime_observer',
        parameters=[{'global_frame': 'map', 'base_frame': 'base_footprint'}],
        condition=IfCondition(PythonExpression([
            "'", recovery_validation_preview, "' == 'true' and '",
            recovery_enabled, "' == 'false'"])),
        output='screen',
    )
    complex_route_advisor = Node(
        package='carbot_nav_recovery',
        executable='complex_route_advisor',
        name='carbot_complex_route_advisor',
        parameters=[{
            'global_frame': 'map',
            'base_frame': 'base_footprint',
            'braking_profile': complex_route_braking_profile,
            'navigation_config_path': navigation_config,
            'footprint_xy': [0.155, 0.133, 0.155, -0.133,
                             -0.130, -0.133, -0.130, 0.133],
        }],
        condition=IfCondition(complex_route_validation),
        output='screen',
    )

    # Issue #13 PR3: the only startup velocity source for segmented
    # rotation.  Launched only for segmented_rotation with motion_policy
    # guarded; the manager validates that combination at startup.
    localization_motion_guard = Node(
        package='isaac_3d_lidar_bringup',
        executable='localization_motion_guard',
        name='localization_motion_guard',
        parameters=[{
            'cmd_vel_topic': cmd_vel_output,
            **probe_parameters,
        }],
        condition=IfCondition(PythonExpression([
            "'", automatic_localization, "' == 'true' and '",
            localization_strategy, "' == 'segmented_rotation' and '",
            motion_policy, "' == 'guarded'"])),
        output='screen',
    )

    # Issue #13 PR2: read-only sweep evidence; no velocity publisher.
    localization_rotation_preview = Node(
        package='isaac_3d_lidar_bringup',
        executable='localization_rotation_preview',
        name='localization_rotation_preview',
        parameters=[{
            'scan_topic': '/scan',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'footprint_xy': [0.155, 0.133, 0.155, -0.133,
                             -0.130, -0.133, -0.130, 0.133],
        }],
        condition=IfCondition(rotation_preview),
        output='screen',
    )

    def configure_recovery(context):
        if recovery_enabled.perform(context) != 'true':
            return []
        profile = acceptance_profile.perform(context)
        if not profile or not os.path.isfile(profile):
            raise RuntimeError('bounded recovery requires a physical acceptance profile')
        if automatic_localization.perform(context) != 'true':
            raise RuntimeError('bounded recovery requires automatic localization status')
        plugin_dir = get_package_share_directory('carbot_recovery_plugins')
        overlay = os.path.join(plugin_dir, 'config', 'recovery_overlay.yaml')
        tree = os.path.join(plugin_dir, 'behavior_trees', 'bounded_recovery.xml')
        return [
            Node(package='nav2_controller', executable='controller_server',
                 name='controller_server', parameters=[navigation_config, overlay],
                 remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
            Node(package='nav2_behaviors', executable='behavior_server',
                 name='behavior_server', parameters=[navigation_config, overlay],
                 remappings=[('cmd_vel', 'cmd_vel_nav')], output='screen'),
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 name='bt_navigator',
                 parameters=[navigation_config, overlay,
                             {'default_nav_to_pose_bt_xml': tree}],
                 output='screen'),
            Node(package='carbot_nav_recovery', executable='recovery_coordinator',
                 name='carbot_recovery_runtime_observer',
                 parameters=[{'acceptance_profile': profile}], output='screen'),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('bounded_recovery_enabled', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('recovery_acceptance_profile', default_value=''),
        DeclareLaunchArgument(
            'map',
            description='Absolute path to the real-robot Nav2 map YAML.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Activate localization and navigation lifecycle nodes.'
            ),
        ),
        DeclareLaunchArgument(
            'cmd_vel_output',
            default_value='/cmd_vel_command',
            description=(
                'Compensator input topic. Use /cmd_vel_diagnostic for a '
                'non-actuating control-pipeline diagnostic.'
            ),
        ),
        DeclareLaunchArgument(
            'automatic_localization',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Run guarded AMCL global localization before activating '
                'navigation. This forces lifecycle autostart off.'
            ),
        ),
        DeclareLaunchArgument(
            'auto_localization_validation_only',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Keep Nav2 inactive after a valid automatic localization. '
                'The maintained --automatic-activate entry is the only '
                'physical workflow that sets this false.'
            ),
        ),
        DeclareLaunchArgument(
            'localization_strategy',
            default_value='legacy_full_rotation',
            choices=['legacy_full_rotation', 'stationary_only',
                     'segmented_rotation'],
            description=(
                'Issue #13 startup localization strategy. stationary_only '
                'searches the whole map without any rotation command; '
                'segmented_rotation may probe only with motion_policy '
                'guarded and an ACCEPTED motion profile.'
            ),
        ),
        DeclareLaunchArgument(
            'motion_policy',
            default_value='forbid',
            choices=['forbid', 'guarded'],
            description='Whether segmented_rotation may rotate at all.',
        ),
        DeclareLaunchArgument(
            'motion_profile_path', default_value='',
            description='ACCEPTED rotation motion profile (JSON).'),
        DeclareLaunchArgument('extrinsics_hash', default_value=''),
        DeclareLaunchArgument('control_chain_hash', default_value=''),
        DeclareLaunchArgument(
            'force_probe_once',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Real-robot test: probe once even if the stationary result '
                'passed. Only with segmented_rotation, guarded motion and '
                'validation-only localization.'
            ),
        ),
        DeclareLaunchArgument(
            'operator_rotation_clear',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Per-launch operator attestation that the placement can '
                'rotate in place; covers unobserved sweep cells only.'
            ),
        ),
        DeclareLaunchArgument(
            'recovery_validation_preview',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Show non-actuating rotation sweep evaluations in RViz. '
                'This preview never publishes velocity commands.'
            ),
        ),
        DeclareLaunchArgument(
            'complex_route_validation',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Publish Issue #12 curvature/braking advice without changing '
                'controller commands.'
            ),
        ),
        DeclareLaunchArgument(
            'rotation_preview',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Show Issue #13 in-place rotation sweep evidence (observed '
                'free, occupied, unknown) in RViz. Never publishes velocity.'
            ),
        ),
        DeclareLaunchArgument(
            'complex_route_braking_profile',
            default_value='',
            description=(
                'Absolute accepted physical braking profile. Empty keeps the '
                'advisor in CALIBRATION_REQUIRED mode.'
            ),
        ),
        OpaqueFunction(function=configure_recovery),
        lio_odometry,
        scan_projection,
        localization_scan_projection,
        map_server,
        manual_localizer,
        amcl,
        manual_localization_lifecycle,
        automatic_localization_lifecycle,
        controller,
        planner,
        behaviors,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_lifecycle,
        automatic_localization_manager,
        recovery_preview,
        recovery_runtime_observer,
        complex_route_advisor,
        localization_rotation_preview,
        localization_motion_guard,
    ])
