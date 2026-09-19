#!/usr/bin/env bash

# Source this file inside a host-networked project container before running a
# real-robot ROS process. Do not use it for the default Isaac simulation stack.
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# The Jetson hardware nodes and micro-ROS Agent use Fast DDS. Matching the
# RMW here avoids the cross-vendor discovery failure observed on the physical
# LAN while retaining the loopback-only CycloneDDS profile for simulation.
unset CYCLONEDDS_URI

printf 'Real robot ROS environment: domain=%s rmw=%s\n' \
  "$ROS_DOMAIN_ID" "$RMW_IMPLEMENTATION"
