from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="carbot_hardware",
                executable="mid360_imu_adapter",
                name="mid360_imu_adapter",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [
                            FindPackageShare("carbot_hardware"),
                            "config",
                            "mid360_imu.yaml",
                        ]
                    )
                ],
            )
        ]
    )
