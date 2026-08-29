from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('isaac_3d_lidar_bringup')

    base_config = os.path.join(
        bringup_dir,
        'config',
        'nvblox',
        'xt32_nvblox_base.yaml'
    )

    sim_config = os.path.join(
        bringup_dir,
        'config',
        'nvblox',
        'xt32_nvblox_sim.yaml'
    )

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        remappings=[
            ('pointcloud', '/front_3d_lidar/lidar_points_nvblox'),
        ],
        parameters=[
            base_config,
            sim_config,
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
            }
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

    pointcloud_padder = Node(
        package='isaac_3d_lidar_bringup',
        executable='pointcloud_padder',
        name='pointcloud_padder',
        output='screen',
        parameters=[{
            'input_topic': '/front_3d_lidar/lidar_points',
            'output_topic': '/front_3d_lidar/lidar_points_nvblox',
            'target_width': 1800,
            'target_height': 31,
        }],
    )

    return LaunchDescription([
        # Isaac Sim publishes sensor data, TF, and /clock in simulation time.
        # Start nvblox on the same clock so its timers and queued sensor stamps
        # never mix simulation time with the host wall clock.
        SetParameter(name='use_sim_time', value=True),
        pointcloud_padder,
        container,
    ])
