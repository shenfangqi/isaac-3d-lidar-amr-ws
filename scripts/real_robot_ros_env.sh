#!/usr/bin/env bash

# Source this file inside a host-networked project container before running a
# real-robot ROS process. Do not use it for the default Isaac simulation stack.
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/isaac_3d_lidar_amr_ws/configs/cyclonedds_ros_jetson.xml

printf 'Real robot ROS environment: domain=%s rmw=%s dds=%s\n' \
  "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION" "$CYCLONEDDS_URI"
