"""Publish the sole real-robot odometry TF from 3D LiDAR and MID-360 IMU."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')
    ekf_config = os.path.join(
        bringup_dir,
        'config',
        'state_estimation',
        'mid360_lio_ekf.yaml',
    )

    lidar_odometry = Node(
        package='kiss_icp',
        executable='kiss_icp_node',
        name='kiss_icp_node',
        remappings=[
            ('pointcloud_topic', '/mid360/points_xyz_mapping'),
            ('kiss/odometry', '/kiss/odometry'),
        ],
        parameters=[{
            'use_sim_time': False,
            'base_frame': 'base_footprint',
            'lidar_odom_frame': 'odom',
            # The EKF below is the only odom -> base_footprint TF owner.
            'publish_odom_tf': False,
            'invert_odom_tf': False,
            'publish_debug_clouds': False,
            'max_range': 20.0,
            'min_range': 0.5,
            # The packed XYZ stream intentionally has no per-point timestamp.
            # MID-360 gyro-z is fused below rather than deskewed inside KISS.
            'deskew': False,
            'voxel_size': 0.20,
            'max_points_per_voxel': 20,
            'initial_threshold': 0.5,
            'min_motion_th': 0.02,
            'max_num_iterations': 100,
            'convergence_criterion': 0.0001,
            'max_num_threads': 4,
            'position_covariance': 0.02,
            'orientation_covariance': 0.01,
        }],
        output='screen',
    )

    fused_odometry = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom')],
        output='screen',
    )

    return LaunchDescription([
        lidar_odometry,
        # Let KISS establish its first 3D pose before the sole TF owner starts.
        TimerAction(period=1.0, actions=[fused_odometry]),
    ])
