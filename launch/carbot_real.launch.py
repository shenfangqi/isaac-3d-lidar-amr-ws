"""Compatibility wrapper for the canonical Jetson navigation launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')
    canonical_launch = os.path.join(
        bringup_dir, 'launch', 'carbot_navigation_real.launch.py')

    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')
    cmd_vel_output = LaunchConfiguration('cmd_vel_output')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Absolute path to the real-robot Nav2 map YAML.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'cmd_vel_output',
            default_value='/cmd_vel_command',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(canonical_launch),
            launch_arguments={
                'map': map_yaml,
                'autostart': autostart,
                'cmd_vel_output': cmd_vel_output,
            }.items(),
        ),
    ])
