"""Run nvblox against the Isaac Sim Mid-360 coverage proxy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')
    config_dir = os.path.join(bringup_dir, 'config', 'nvblox')

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=[('pointcloud', '/livox/lidar_nvblox')],
        parameters=[
            os.path.join(config_dir, 'mid360_nvblox_base.yaml'),
            os.path.join(config_dir, 'mid360_nvblox_sim.yaml'),
            {
                'global_frame': 'odom',
                'pose_frame': 'front_3d_lidar',
                'map_clearing_frame_id': 'odom',
                'esdf_slice_bounds_visualization_attachment_frame_id': 'odom',
                'workspace_height_bounds_visualization_attachment_frame_id': 'odom',
                'num_cameras': 0,
                'use_depth': False,
                'use_color': False,
                'use_lidar': True,
                'input_qos': 'DEFAULT',
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

    # nvblox's current lidar CUDA path reads exactly width*height points.
    # Livox frames are variable length, so unused entries must be NaN padded.
    pointcloud_padder = Node(
        package='isaac_3d_lidar_bringup',
        executable='pointcloud_padder',
        name='pointcloud_padder',
        output='screen',
        parameters=[{
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar_nvblox',
            'target_width': 1000,
            'target_height': 40,
        }],
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        pointcloud_padder,
        container,
    ])
