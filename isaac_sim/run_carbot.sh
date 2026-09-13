#!/usr/bin/env bash
set -euo pipefail

export isaac_sim_package_path=/isaac-sim
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${isaac_sim_package_path}/exts/isaacsim.ros2.bridge/humble/lib"

cd /isaac-sim
exec ./python.sh \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/auto_play_carbot.py
