"""Build the real Carbot's 2.5D nvblox map from FAST-LIO2 odometry."""

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
    lio_launch = os.path.join(
        bringup_dir, 'launch', 'mid360_fast_lio_odometry.launch.py')
    lio_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lio_launch),
    )

    mesh_voxel_relay = Node(
        package='isaac_3d_lidar_bringup',
        executable='mesh_voxel_relay',
        name='nvblox_mesh_voxel_relay',
        parameters=[{
            'publish_period_sec': 2.0,
            'max_points': 6000,
            'cube_size_m': 0.05,
            'cloud_fallback_topic': '/fast_lio/cloud_registered',
            'fixed_frame': 'odom',
            # Full nvblox Mesh messages exceed the UDP DDS transport budget.
            # Use the aligned accumulated-cloud fallback for remote RViz and
            # avoid making nvblox serialize layers that cannot be delivered.
            'enable_nvblox_inputs': False,
        }],
        output='screen',
    )

    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nvblox_launch),
        launch_arguments={
            # FAST-LIO2 is the pose source.  nvblox builds its 2.5D map
            # directly in odom; no second scan matcher may rewrite the pose.
            'global_frame': 'odom',
            'pose_frame': 'fast_lio_imu',
            # FAST-LIO2 publishes this cloud after tightly coupled IMU deskew.
            'pointcloud_topic': '/fast_lio/cloud_registered_body',
            'enable_scan': 'true',
            'scan_pointcloud_topic': '/fast_lio/cloud_registered_body',
            'enable_lio': 'false',
            # Whole-area mapping must be cumulative from component startup.
            # Supplying these here avoids the 7 m rolling defaults used by
            # navigation and avoids ineffective post-start parameter writes.
            'map_clearing_radius_m': '1000.0',
            # nvblox requires the constructor value to be strictly below 1.0.
            # Disable the decay tick itself, then the runbook sets the exposed
            # factor to 1.0 after startup for independent verification.
            'static_tsdf_decay_factor': '0.95',
            'static_decay_deallocate': 'false',
            'decay_tsdf_rate_hz': '0.0',
            'clear_map_outside_radius_rate_hz': '0.0',
        }.items(),
    )

    return LaunchDescription([
        lio_odometry,
        # Subscribe before nvblox starts so its first Mesh message is a full
        # snapshot; later incremental block updates are cached by the relay.
        mesh_voxel_relay,
        # Let FAST-LIO finish stationary IMU initialization before integration.
        TimerAction(period=3.0, actions=[nvblox]),
    ])
