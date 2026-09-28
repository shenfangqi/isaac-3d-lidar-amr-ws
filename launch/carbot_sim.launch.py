"""Launch Carbot ROS description and saved-map navigation for Isaac Sim."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carbot_navigation import build_navigation_actions  # noqa: E402


def generate_launch_description():
    localization_mode = LaunchConfiguration("localization_mode")
    initial_pose_mode = LaunchConfiguration("amcl_initial_pose_mode")
    initial_x = LaunchConfiguration("amcl_initial_x")
    initial_y = LaunchConfiguration("amcl_initial_y")
    initial_yaw = LaunchConfiguration("amcl_initial_yaw")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    actions = build_navigation_actions(
        "sim", localization_mode, initial_pose_mode,
        initial_x, initial_y, initial_yaw, nav2_params_file,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "localization_mode",
                default_value="ground_truth",
                choices=["ground_truth", "amcl"],
            ),
            DeclareLaunchArgument(
                "amcl_initial_pose_mode",
                default_value="odom_identity",
                choices=["odom_identity", "fixed", "manual"],
            ),
            DeclareLaunchArgument("amcl_initial_x", default_value="0.0"),
            DeclareLaunchArgument("amcl_initial_y", default_value="0.0"),
            DeclareLaunchArgument("amcl_initial_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=(
                    "/workspace/ros-humble/isaac_3d_lidar_amr_ws/"
                    "configs/nav2_params_sim.yaml"
                ),
                description=(
                    "Nav2 parameter file. Select "
                    "configs/nav2_params_sim_real_parity.yaml for the "
                    "physical-navigation policy parity profile."
                ),
            ),
            *actions,
        ]
    )
