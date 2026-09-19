"""Build aligned 2D SLAM Toolbox and 3D nvblox maps on the real Carbot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')
    nvblox_launch = os.path.join(
        bringup_dir, 'launch', 'mid360_nvblox_real.launch.py')
    slam_config = os.path.join(
        bringup_dir,
        'config',
        'slam_toolbox',
        'carbot_mid360_mapping.yaml',
    )

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen',
    )

    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nvblox_launch),
        launch_arguments={
            'global_frame': 'map',
            'pose_frame': 'livox_frame',
            'pointcloud_topic': '/mid360/points_xyz',
            'enable_scan': 'true',
            'scan_pointcloud_topic': '/livox/lidar',
        }.items(),
    )

    return LaunchDescription([
        slam_toolbox,
        # Give SLAM Toolbox time to publish map -> odom before nvblox starts.
        TimerAction(period=3.0, actions=[nvblox]),
    ])
