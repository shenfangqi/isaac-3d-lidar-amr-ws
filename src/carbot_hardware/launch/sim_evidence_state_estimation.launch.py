"""Run the real wheel-odometry + MID-360 EKF path against Isaac evidence topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    hardware_share = FindPackageShare("carbot_hardware")
    wheel_config = PathJoinSubstitution(
        [hardware_share, "config", "wheel_odometry_fused.yaml"]
    )
    ekf_config = PathJoinSubstitution(
        [hardware_share, "config", "mid360_wheel_ekf.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="carbot_hardware",
                executable="wheel_odometry",
                name="carbot_wheel_odometry",
                output="screen",
                parameters=[wheel_config, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_config, {"use_sim_time": use_sim_time}],
                remappings=[("odometry/filtered", "/odom")],
            ),
            Node(
                package="isaac_3d_lidar_bringup",
                executable="pointcloud_evidence_degrader",
                name="pointcloud_evidence_degrader",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
