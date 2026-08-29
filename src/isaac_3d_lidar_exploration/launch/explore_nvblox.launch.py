from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    package_share = get_package_share_directory('isaac_3d_lidar_exploration')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_topic = LaunchConfiguration('map_topic')
    global_frame = LaunchConfiguration('global_frame')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    source_odom_topic = LaunchConfiguration('source_odom_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    start_navigation = LaunchConfiguration('start_navigation')
    start_scan_converter = LaunchConfiguration('start_scan_converter')
    start_odom_relay = LaunchConfiguration('start_odom_relay')
    start_enabled = LaunchConfiguration('start_enabled')

    nav2_params = RewrittenYaml(
        source_file=os.path.join(
            package_share, 'config', 'nav2_live_mapping.yaml'
        ),
        root_key='',
        param_rewrites={
            'use_sim_time': use_sim_time,
            'global_frame': global_frame,
            'robot_base_frame': robot_base_frame,
            'odom_topic': odom_topic,
            'map_topic': map_topic,
            'topic': scan_topic,
        },
        convert_types=True,
    )

    scan_converter = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='exploration_pointcloud_to_laserscan',
        condition=IfCondition(start_scan_converter),
        remappings=[
            ('cloud_in', pointcloud_topic),
            ('scan', scan_topic),
        ],
        parameters=[{
            'use_sim_time': use_sim_time,
            'target_frame': robot_base_frame,
            'min_height': 0.10,
            'max_height': 0.65,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.0174,
            'range_min': 0.50,
            'range_max': 20.0,
            'use_inf': True,
        }],
        output='screen',
    )

    odom_relay = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'topic_tools', 'relay',
            source_odom_topic, odom_topic,
        ],
        condition=IfCondition(start_odom_relay),
        output='screen',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, 'launch', 'navigation_launch.py')
        ),
        condition=IfCondition(start_navigation),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items(),
    )

    explorer = Node(
        package='isaac_3d_lidar_exploration',
        executable='frontier_explorer',
        name='frontier_explorer',
        parameters=[
            os.path.join(package_share, 'config', 'frontier_explorer.yaml'),
            {
                'use_sim_time': use_sim_time,
                'map_topic': map_topic,
                'global_frame': global_frame,
                'robot_base_frame': robot_base_frame,
                'start_enabled': start_enabled,
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/nvblox_node/static_occupancy_grid',
        ),
        DeclareLaunchArgument('global_frame', default_value='odom'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/front_3d_lidar/lidar_points',
        ),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument(
            'source_odom_topic', default_value='/chassis/odom'
        ),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_scan_converter', default_value='true'),
        DeclareLaunchArgument('start_odom_relay', default_value='true'),
        DeclareLaunchArgument(
            'start_enabled',
            default_value='false',
            description='Explicit motion interlock for autonomous exploration',
        ),
        scan_converter,
        odom_relay,
        navigation,
        explorer,
    ])
