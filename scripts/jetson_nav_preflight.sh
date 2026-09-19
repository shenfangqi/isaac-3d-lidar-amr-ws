#!/usr/bin/env bash

set -uo pipefail

container_name="carbot-nvblox"
workspace="/home/shenfq/Projects/isaac_ros-dev"
container_workspace="/workspaces/isaac_ros-dev"
carbot_workspace="/home/shenfq/Projects/carbot-ros2"
failures=0

pass() {
  echo "PASS: $*"
}

fail() {
  echo "FAIL: $*" >&2
  failures=$((failures + 1))
}

ros2_in_container() {
  docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && $*"
}

check_user_service() {
  local service_name="$1"
  if systemctl --user is-active --quiet "${service_name}"; then
    pass "${service_name} is active"
  else
    fail "${service_name} is not active"
  fi
}

check_system_service() {
  local service_name="$1"
  if systemctl is-active --quiet "${service_name}"; then
    pass "${service_name} is active"
  else
    fail "${service_name} is not active"
  fi
}

topic_count() {
  local topic="$1"
  local endpoint="$2"
  ros2_in_container "ros2 topic info '${topic}'" 2>/dev/null \
    | sed -n "s/^${endpoint} count: //p" \
    | head -n 1
}

echo "Carbot Jetson navigation preflight (read-only)"
echo

check_user_service micro-ros-agent.service
check_user_service carbot-wheel-odometry.service
check_user_service carbot-mid360.service
check_user_service carbot-description.service
check_system_service carbot-mid360-ptp.service

if [[ "$(docker inspect -f '{{.State.Running}}' "${container_name}" 2>/dev/null || true)" != "true" ]]; then
  fail "${container_name} is not running"
  echo
  echo "Preflight failed with ${failures} error(s)." >&2
  exit 1
fi
pass "${container_name} is running"

map_state="$(ros2_in_container "ros2 lifecycle get /map_server" 2>/dev/null || true)"
if grep -Eq '^(unconfigured|inactive)' <<<"${map_state}"; then
  pass "Nav2 is not active (${map_state})"
else
  fail "expected map_server to be unconfigured or inactive, got: ${map_state:-unavailable}"
fi

cmd_publishers="$(topic_count /cmd_vel Publisher)"
cmd_subscribers="$(topic_count /cmd_vel Subscription)"
if [[ "${cmd_publishers:-x}" == "0" ]]; then
  pass "/cmd_vel has no publisher before Nav2 activation"
else
  fail "/cmd_vel publisher count is ${cmd_publishers:-unknown}; expected 0"
fi
if [[ "${cmd_subscribers:-0}" -ge 1 ]] 2>/dev/null; then
  pass "/cmd_vel has a base-controller subscriber"
else
  fail "/cmd_vel has no base-controller subscriber"
fi

if timeout --kill-after=1s 4s docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && timeout --kill-after=1s 2s ros2 topic echo /cmd_vel --once >/dev/null 2>&1"; then
  fail "/cmd_vel produced a message while Nav2 is not active"
else
  pass "/cmd_vel is silent"
fi

odom_publishers="$(topic_count /odom Publisher)"
if [[ "${odom_publishers:-x}" == "1" ]]; then
  pass "/odom has exactly one publisher"
else
  fail "/odom publisher count is ${odom_publishers:-unknown}; expected 1"
fi

freshness_output="$(bash -lc \
  "source /opt/ros/humble/setup.bash && source ${carbot_workspace}/install/setup.bash && ROS_DOMAIN_ID=0 python3 ${workspace}/scripts/jetson_ros_freshness_check.py" 2>&1)"
freshness_status=$?
echo "${freshness_output}"
if ((freshness_status > 0)); then
  failures=$((failures + freshness_status))
fi

echo
if ((failures > 0)); then
  echo "Preflight failed with ${failures} error(s). Do not activate Nav2." >&2
  exit 1
fi

echo "Preflight passed. Nav2 is still not active; this script never publishes velocity."
