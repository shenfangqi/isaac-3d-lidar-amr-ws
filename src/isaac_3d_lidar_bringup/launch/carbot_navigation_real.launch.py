"""Run the on-Jetson saved-map navigation stack for the real Carbot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')
    amcl_config = os.path.join(
        bringup_dir, 'config', 'nav2', 'carbot_amcl_real.yaml')
    navigation_config = os.path.join(
        bringup_dir, 'config', 'nav2', 'carbot_navigation_real.yaml')
    lio_launch = os.path.join(
        bringup_dir, 'launch', 'mid360_fast_lio_odometry.launch.py')

    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    cmd_vel_output = LaunchConfiguration('cmd_vel_output')

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
            'queue_size': 30,
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

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[amcl_config, {'yaml_filename': map_yaml}],
        output='screen',
    )
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[amcl_config],
        output='screen',
    )
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': False,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
        }],
        output='screen',
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[navigation_config],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
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
        output='screen',
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[navigation_config],
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
            'autostart': autostart,
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

    return LaunchDescription([
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
        lio_odometry,
        scan_projection,
        map_server,
        amcl,
        localization_lifecycle,
        controller,
        planner,
        behaviors,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_lifecycle,
    ])
