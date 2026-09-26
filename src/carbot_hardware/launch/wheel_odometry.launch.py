from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    parameters = Path(get_package_share_directory("carbot_hardware")) / "config" / "wheel_odometry.yaml"
    return LaunchDescription(
        [
            Node(
                package="carbot_hardware",
                executable="wheel_odometry",
                name="carbot_wheel_odometry",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
