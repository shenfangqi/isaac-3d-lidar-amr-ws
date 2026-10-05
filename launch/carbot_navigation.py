"""Shared ROS-side launch construction for Carbot simulation and hardware."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = Path("/workspace/ros-humble/isaac_3d_lidar_amr_ws")
CONFIG_ROOT = WORKSPACE / "configs"


def load_runtime_profile(runtime):
    """Load common interface names and one explicit runtime profile."""
    common = yaml.safe_load(
        (CONFIG_ROOT / "carbot/common.yaml").read_text(encoding="utf-8")
    )
    profile = yaml.safe_load(
        (CONFIG_ROOT / f"carbot/{runtime}.yaml").read_text(encoding="utf-8")
    )
    expected = "simulation" if runtime == "sim" else "real"
    if profile["runtime"] != expected:
        raise RuntimeError(f"Invalid Carbot runtime profile: {runtime}")
    return common, profile


def build_navigation_actions(
    runtime,
    localization_mode,
    initial_pose_mode,
    initial_x,
    initial_y,
    initial_yaw,
    nav2_params_file=None,
):
    """Build description, scan, localization, and Nav2 launch actions."""
    common, profile = load_runtime_profile(runtime)
    use_sim_time = profile["use_sim_time"]
    frames = common["frames"]
    topics = common["topics"]
    scan = common["scan_projection"]
    map_yaml = common["maps"]["nav2_yaml"]
    use_amcl = IfCondition(
        PythonExpression(["'", localization_mode, "' == 'amcl'"])
    )
    use_ground_truth = IfCondition(
        PythonExpression(["'", localization_mode, "' == 'ground_truth'"])
    )
    use_initializer = IfCondition(
        PythonExpression(
            [
                "'", localization_mode, "' == 'amcl' and '",
                initial_pose_mode, "' != 'manual'",
            ]
        )
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("carbot_description"))
                / "launch/description.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": str(use_sim_time).lower()}.items(),
    )

    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        remappings=[
            ("/cloud_in", topics["pointcloud"]),
            ("/scan", topics["scan"]),
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": frames["robot_base"],
                "min_height": scan["min_height_m"],
                "max_height": scan["max_height_m"],
                "angle_min": scan["angle_min_rad"],
                "angle_max": scan["angle_max_rad"],
                "angle_increment": scan["angle_increment_rad"],
                "range_min": scan["min_range_m"],
                "range_max": scan["max_range_m"],
                "use_inf": True,
            }
        ],
        output="screen",
    )

    overhead_clearance_markers = Node(
        package="isaac_3d_lidar_bringup",
        executable="overhead_clearance_marker_publisher",
        name="overhead_clearance_marker_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "config_path": str(CONFIG_ROOT / "carbot/common.yaml"),
                "frame_id": frames["map"],
                "topic": topics["overhead_clearance_markers"],
            }
        ],
        output="screen",
    )

    # Keep the local controller on the raw scan so emergency collision
    # reactions do not wait for filtering.  The global branch first removes
    # returns already explained by the static map, then requires temporal
    # consistency before a novel obstacle can invalidate the global path.
    static_map_scan_filter = Node(
        package="isaac_3d_lidar_bringup",
        executable="static_map_scan_filter",
        name="static_map_scan_filter",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": topics["scan"],
                "output_topic": topics["global_scan_static_filtered"],
                "map_topic": "/map",
                "map_frame": frames["map"],
                "static_margin_m": 0.12,
                "occupied_threshold": 65,
            }
        ],
        output="screen",
    )

    global_scan_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="global_scan_filter",
        remappings=[
            ("scan", topics["global_scan_static_filtered"]),
            ("scan_filtered", topics["global_scan"]),
        ],
        parameters=[
            str(CONFIG_ROOT / "laser_filters_global_sim.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("nav2_bringup"))
                / "launch/localization_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "map": map_yaml,
            "params_file": profile["amcl_params_file"],
        }.items(),
        condition=use_amcl,
    )

    amcl_pose_initializer = Node(
        package="isaac_3d_lidar_bringup",
        executable="amcl_pose_initializer",
        name="amcl_pose_initializer",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "mode": initial_pose_mode,
                "odom_topic": topics["odom"],
                "fixed_x": ParameterValue(initial_x, value_type=float),
                "fixed_y": ParameterValue(initial_y, value_type=float),
                "fixed_yaw": ParameterValue(initial_yaw, value_type=float),
            }
        ],
        condition=use_initializer,
        output="screen",
    )

    ground_truth_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="ground_truth_map_to_odom",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", frames["map"],
            "--child-frame-id", frames["odom"],
        ],
        condition=use_ground_truth,
        output="screen",
    )

    ground_truth_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
        condition=use_ground_truth,
        output="screen",
    )
    ground_truth_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_server"],
            }
        ],
        condition=use_ground_truth,
        output="screen",
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("nav2_bringup"))
                / "launch/navigation_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": str(use_sim_time).lower(),
            "params_file": nav2_params_file or profile["nav2_params_file"],
        }.items(),
    )

    actions = [
        description,
        pointcloud_to_laserscan,
    ]
    if runtime == "sim":
        actions.append(overhead_clearance_markers)
        actions.append(static_map_scan_filter)
        actions.append(global_scan_filter)
    actions.extend([
        amcl_pose_initializer,
        ground_truth_tf,
        TimerAction(
            period=5.0,
            actions=[
                localization,
                ground_truth_map_server,
                ground_truth_lifecycle,
            ],
        ),
        TimerAction(period=15.0, actions=[navigation]),
    ])
    return actions
