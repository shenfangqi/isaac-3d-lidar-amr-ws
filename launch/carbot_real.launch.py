"""Launch hardware-safe Carbot description and saved-map navigation."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carbot_navigation import build_navigation_actions  # noqa: E402


def generate_launch_description():
    initial_pose_mode = LaunchConfiguration("amcl_initial_pose_mode")
    initial_x = LaunchConfiguration("amcl_initial_x")
    initial_y = LaunchConfiguration("amcl_initial_y")
    initial_yaw = LaunchConfiguration("amcl_initial_yaw")
    actions = build_navigation_actions(
        "real", "amcl", initial_pose_mode,
        initial_x, initial_y, initial_yaw,
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "amcl_initial_pose_mode",
                default_value="manual",
                choices=["fixed", "manual"],
            ),
            DeclareLaunchArgument("amcl_initial_x", default_value="0.0"),
            DeclareLaunchArgument("amcl_initial_y", default_value="0.0"),
            DeclareLaunchArgument("amcl_initial_yaw", default_value="0.0"),
            *actions,
        ]
    )
