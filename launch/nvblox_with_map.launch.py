from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    nvblox_launch = ExecuteProcess(
        cmd=[
            "ros2", "launch",
            "isaac_3d_lidar_bringup",
            "xt32_nvblox.launch.py",
        ],
        output="screen",
    )

    load_map = ExecuteProcess(
        cmd=[
            "ros2", "service", "call",
            "/nvblox_node/load_map",
            "nvblox_msgs/srv/FilePath",
            "{file_path: '/workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/nvblox/warehouse_v3.nvblx'}",
        ],
        output="screen",
    )

    return LaunchDescription([
        nvblox_launch,
        TimerAction(period=5.0, actions=[load_map]),
    ])
