"""Run nvblox against the physical Carbot MID-360."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')
    config_dir = os.path.join(bringup_dir, 'config', 'nvblox')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    nvblox_pointcloud_topic = LaunchConfiguration('nvblox_pointcloud_topic')
    global_frame = LaunchConfiguration('global_frame')
    pose_frame = LaunchConfiguration('pose_frame')
    enable_scan = LaunchConfiguration('enable_scan')
    scan_pointcloud_topic = LaunchConfiguration('scan_pointcloud_topic')
    enable_lio = LaunchConfiguration('enable_lio')

    lio_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_dir, 'launch', 'mid360_fast_lio_odometry.launch.py')),
        condition=IfCondition(enable_lio),
    )

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=[('pointcloud', nvblox_pointcloud_topic)],
        parameters=[
            os.path.join(config_dir, 'mid360_nvblox_base.yaml'),
            os.path.join(config_dir, 'mid360_nvblox_real.yaml'),
            {
                'global_frame': global_frame,
                'pose_frame': pose_frame,
                'num_cameras': 0,
                'use_depth': False,
                'use_color': False,
                'use_lidar': True,
                'input_qos': 'SENSOR_DATA',
            },
        ],
    )

    container = ComposableNodeContainer(
        name='nvblox_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        composable_node_descriptions=[nvblox_node],
        output='screen',
    )

    # nvblox's CUDA LiDAR projector requires exactly lidar_width *
    # lidar_height points. MID-360 scans have a variable point count, so pad
    # each packed XYZ scan with NaNs without changing its timestamp or frame.
    pointcloud_padder = Node(
        package='isaac_3d_lidar_bringup',
        executable='pointcloud_padder',
        name='mid360_pointcloud_padder',
        output='screen',
        parameters=[{
            'input_topic': pointcloud_topic,
            'output_topic': nvblox_pointcloud_topic,
            'target_width': 1000,
            'target_height': 40,
        }],
    )

    scan_projection = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='mid360_pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', scan_pointcloud_topic),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.05,
            # FAST-LIO publishes odometry immediately before the matching
            # body cloud.  Keep enough transform-filter headroom for short
            # Jetson CPU bursts without dropping scans or flooding logs.
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
        condition=IfCondition(enable_scan),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lio',
            default_value='true',
            choices=['true', 'false'],
            description='Start the sole 3D LiDAR-IMU odometry owner.',
        ),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/fast_lio/cloud_registered_body',
            description=(
                'Deskewed FAST-LIO2 cloud expressed in its full IMU frame.'
            ),
        ),
        DeclareLaunchArgument(
            'nvblox_pointcloud_topic',
            default_value='/mid360/points_xyz_nvblox',
            description='Fixed-shape, NaN-padded PointCloud2 for nvblox.',
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='odom',
            description='Fixed frame in which nvblox builds its map.',
        ),
        DeclareLaunchArgument(
            'pose_frame',
            default_value='fast_lio_imu',
            description=(
                'Frame whose pose is used to integrate the FAST-LIO body cloud.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_scan',
            default_value='true',
            description='Project the local MID-360 cloud to /scan.',
        ),
        DeclareLaunchArgument(
            'scan_pointcloud_topic',
            default_value='/fast_lio/cloud_registered_body',
            description='PointCloud2 input used for the 2D obstacle scan.',
        ),
        SetParameter(name='use_sim_time', value=False),
        lio_odometry,
        pointcloud_padder,
        container,
        scan_projection,
    ])
