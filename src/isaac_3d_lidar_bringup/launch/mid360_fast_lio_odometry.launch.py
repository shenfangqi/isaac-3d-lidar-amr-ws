"""Run tightly coupled FAST-LIO2 for the physical MID-360."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')
    config = os.path.join(
        bringup_dir, 'config', 'state_estimation', 'fast_lio_mid360.yaml'
    )

    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio2',
        parameters=[config],
        remappings=[
            ('/Odometry', '/fast_lio/imu_odom'),
            ('/cloud_registered', '/fast_lio/cloud_registered'),
            ('/cloud_registered_body', '/fast_lio/cloud_registered_body'),
            ('/cloud_effected', '/fast_lio/cloud_effected'),
            ('/Laser_map', '/fast_lio/map'),
            ('/path', '/fast_lio/path'),
        ],
        output='screen',
    )

    base_adapter = Node(
        package='isaac_3d_lidar_bringup',
        executable='fast_lio_base_adapter',
        name='fast_lio_base_adapter',
        output='screen',
    )

    return LaunchDescription([fast_lio, base_adapter])
