#!/usr/bin/env bash
set -euo pipefail

# The bridge's library path must exist before the Isaac Python process starts;
# setting it after SimulationApp is imported is too late for the ELF loader.
docker exec -i isaac-sim bash -lc '
    export ROS_DISTRO=humble
    export ROS_DOMAIN_ID=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
    export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH:-}
    cd /isaac-sim
    exec ./python.sh /workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/auto_play_mid360.py
'
