from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    livox_config = PathJoinSubstitution(
        [FindPackageShare("carbot_hardware"), "config", "MID360_config.json"]
    )
    imu_config = PathJoinSubstitution(
        [FindPackageShare("carbot_hardware"), "config", "mid360_imu.yaml"]
    )

    # The C++ port of the IMU adapter produces identical messages; the Python
    # node used about half a Jetson core at 200 Hz (2026-10-19).
    # imu_adapter:=python restores the original node.
    use_python_adapter = PythonExpression(
        ["'", LaunchConfiguration("imu_adapter"), "' == 'python'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "imu_adapter",
                default_value="cpp",
                choices=["cpp", "python"],
                description="MID-360 IMU adapter implementation",
            ),
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        # FAST-LIO2 requires Livox CustomMsg because offset_time
                        # is used for tightly coupled IMU scan deskewing.
                        "xfer_format": 1,
                        "multi_topic": 0,
                        "data_src": 0,
                        "publish_freq": 10.0,
                        "output_data_type": 0,
                        "frame_id": "livox_frame",
                        "lvx_file_path": "/tmp/unused.lvx",
                        "user_config_path": livox_config,
                        "cmdline_input_bd_code": "livox0000000001",
                    }
                ],
            ),
            Node(
                package="carbot_hardware_cpp",
                executable="mid360_imu_adapter",
                name="mid360_imu_adapter",
                output="screen",
                parameters=[imu_config],
                condition=UnlessCondition(use_python_adapter),
            ),
            Node(
                package="carbot_hardware",
                executable="mid360_imu_adapter",
                name="mid360_imu_adapter",
                output="screen",
                parameters=[imu_config],
                condition=IfCondition(use_python_adapter),
            ),
        ]
    )
