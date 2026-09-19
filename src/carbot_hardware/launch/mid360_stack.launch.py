from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    livox_config = PathJoinSubstitution(
        [FindPackageShare("livox_ros_driver2"), "config", "MID360_config.json"]
    )
    imu_config = PathJoinSubstitution(
        [FindPackageShare("carbot_hardware"), "config", "mid360_imu.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar_publisher",
                output="screen",
                parameters=[
                    {
                        # Keep the real robot interface identical to simulation
                        # and to the mapping/localization consumers. Livox's
                        # native PointCloud2 includes per-point timestamps.
                        "xfer_format": 0,
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
                package="carbot_hardware",
                executable="mid360_imu_adapter",
                name="mid360_imu_adapter",
                output="screen",
                parameters=[imu_config],
            ),
            Node(
                package="carbot_hardware",
                executable="pointcloud_xyz_relay",
                name="mid360_pointcloud_xyz_relay",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/livox/lidar",
                        "output_topic": "/mid360/points_xyz",
                        "point_stride": 4,
                    }
                ],
            ),
        ]
    )
