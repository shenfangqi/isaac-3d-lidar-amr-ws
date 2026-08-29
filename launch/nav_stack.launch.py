from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    amcl_params = "/workspace/ros-humble/isaac_3d_lidar_amr_ws/configs/amcl_params.yaml"
    nav2_params = "/workspace/ros-humble/isaac_3d_lidar_amr_ws/configs/nav2_params.yaml"
    amcl_map_yaml = "/workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/2d/warehouse_v3.yaml"
    ground_truth_map_yaml = "/workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/2d/warehouse_v3.yaml"
    localization_mode = LaunchConfiguration("localization_mode")
    amcl_initial_pose_mode = LaunchConfiguration("amcl_initial_pose_mode")
    amcl_initial_x = LaunchConfiguration("amcl_initial_x")
    amcl_initial_y = LaunchConfiguration("amcl_initial_y")
    amcl_initial_yaw = LaunchConfiguration("amcl_initial_yaw")
    use_ground_truth = IfCondition(PythonExpression([
        "'", localization_mode, "' == 'ground_truth'"
    ]))
    use_amcl = IfCondition(PythonExpression([
        "'", localization_mode, "' == 'amcl'"
    ]))
    use_amcl_initializer = IfCondition(PythonExpression([
        "'", localization_mode, "' == 'amcl' and '",
        amcl_initial_pose_mode, "' != 'manual'"
    ]))

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            ("/cloud_in", "/front_3d_lidar/lidar_points"),
            ("/scan", "/scan"),
        ],
        parameters=[{
            # Project only the obstacle band occupied by the robot. Using the
            # full 3D cloud here also projects floors and overhead shelving
            # into the 2D costmaps and AMCL scan.
            "target_frame": "base_link",
            "min_height": 0.10,
            "max_height": 0.65,
            "angle_min": -3.14,
            "angle_max": 3.14,
            "angle_increment": 0.0174,
            # The simulated lidar sees the chassis at about 0.344 m.
            "range_min": 0.5,
            "range_max": 20.0,
            "use_inf": True,
        }],
        output="screen",
    )

    odom_relay = ExecuteProcess(
        cmd=["ros2", "run", "topic_tools", "relay", "/chassis/odom", "/odom"],
        output="screen",
    )

    localization = ExecuteProcess(
        cmd=[
            "ros2", "launch", "nav2_bringup", "localization_launch.py",
            "use_sim_time:=true",
            f"map:={amcl_map_yaml}",
            f"params_file:={amcl_params}",
        ],
        output="screen",
        condition=use_amcl,
    )

    amcl_pose_initializer = Node(
        package="isaac_3d_lidar_bringup",
        executable="amcl_pose_initializer",
        name="amcl_pose_initializer",
        parameters=[{
            "use_sim_time": True,
            "mode": amcl_initial_pose_mode,
            "odom_topic": "/chassis/odom",
            "fixed_x": ParameterValue(amcl_initial_x, value_type=float),
            "fixed_y": ParameterValue(amcl_initial_y, value_type=float),
            "fixed_yaw": ParameterValue(amcl_initial_yaw, value_type=float),
        }],
        condition=use_amcl_initializer,
        output="screen",
    )

    # The nvblox map is built in the odom frame. In simulation, Isaac's odom
    # is ground truth, so an identity map->odom transform avoids AMCL drift.
    ground_truth_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ground_truth_map_to_odom",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
        condition=use_ground_truth,
        output="screen",
    )

    ground_truth_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{
            "use_sim_time": True,
            "yaml_filename": ground_truth_map_yaml,
        }],
        condition=use_ground_truth,
        output="screen",
    )

    ground_truth_localization_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["map_server"],
        }],
        condition=use_ground_truth,
        output="screen",
    )

    navigation = ExecuteProcess(
        cmd=[
            "ros2", "launch", "nav2_bringup", "navigation_launch.py",
            "use_sim_time:=true",
            f"params_file:={nav2_params}",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "localization_mode",
            default_value="ground_truth",
            description="ground_truth for Isaac Sim, amcl for real robots",
        ),
        DeclareLaunchArgument(
            "amcl_initial_pose_mode",
            default_value="odom_identity",
            description=(
                "odom_identity for the aligned Isaac map, fixed for a known "
                "map pose, or manual for RViz 2D Pose Estimate"
            ),
        ),
        DeclareLaunchArgument("amcl_initial_x", default_value="0.0"),
        DeclareLaunchArgument("amcl_initial_y", default_value="0.0"),
        DeclareLaunchArgument("amcl_initial_yaw", default_value="0.0"),
        pointcloud_to_laserscan,
        odom_relay,
        amcl_pose_initializer,
        ground_truth_tf,

        TimerAction(period=15.0, actions=[
            localization,
            ground_truth_map_server,
            ground_truth_localization_manager,
        ]),
        TimerAction(period=40.0, actions=[navigation]),
    ])
