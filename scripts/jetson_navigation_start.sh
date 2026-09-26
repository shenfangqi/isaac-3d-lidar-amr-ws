#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: $0 MAP_YAML X_M Y_M YAW_RAD [navigation-safe|navigation-diagnostic]" >&2
}

if (( $# < 4 || $# > 5 )); then
  usage
  exit 2
fi

map_yaml="$1"
initial_x="$2"
initial_y="$3"
initial_yaw="$4"
mode="${5:-navigation-safe}"
if [[ "${mode}" != "navigation-safe" && "${mode}" != "navigation-diagnostic" ]]; then
  usage
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
container_name="carbot-nvblox"
container_workspace="/workspaces/isaac_ros-dev"
initializer="${container_workspace}/scripts/jetson_navigation_initialize.py"

stop_on_error() {
  echo "Navigation initialization failed; stopping ${container_name}." >&2
  "${script_dir}/jetson_nvblox_container.sh" stop >/dev/null 2>&1 || true
}
trap stop_on_error ERR

"${script_dir}/jetson_nvblox_container.sh" recreate "${mode}" "${map_yaml}" >/dev/null
"${script_dir}/jetson_nav_preflight.sh"

docker exec "${container_name}" bash -lc \
  "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && python3 '${initializer}' --x '${initial_x}' --y '${initial_y}' --yaw '${initial_yaw}'"

topic_count() {
  local topic="$1"
  local endpoint="$2"
  docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && ros2 topic info '${topic}'" \
    | sed -n "s/^${endpoint} count: //p" | head -n 1
}

cmd_publishers="$(topic_count /cmd_vel Publisher)"
cmd_subscribers="$(topic_count /cmd_vel Subscription)"
command_publishers="$(topic_count /cmd_vel_command Publisher)"
command_subscribers="$(topic_count /cmd_vel_command Subscription)"
if [[ "${cmd_subscribers:-0}" -lt 1 ]]; then
  echo "/cmd_vel has no base-controller subscriber." >&2
  exit 1
fi

if [[ "${mode}" == "navigation-safe" ]]; then
  [[ "${cmd_publishers}" == "1" ]] || {
    echo "/cmd_vel compensator publisher count is ${cmd_publishers:-unknown}; expected 1." >&2
    exit 1
  }
  [[ "${command_publishers}" == "1" && "${command_subscribers}" == "1" ]] || {
    echo "/cmd_vel_command topology is ${command_publishers:-?} publisher(s), ${command_subscribers:-?} subscriber(s); expected 1/1." >&2
    exit 1
  }
else
  [[ "${cmd_publishers}" == "1" && "${command_publishers}" == "0" ]] || {
    echo "Diagnostic mode has an unexpected actuating command source." >&2
    exit 1
  }
  diagnostic_publishers="$(topic_count /cmd_vel_diagnostic Publisher)"
  [[ "${diagnostic_publishers}" == "1" ]] || {
    echo "/cmd_vel_diagnostic publisher count is ${diagnostic_publishers:-unknown}; expected 1." >&2
    exit 1
  }
fi

if timeout --kill-after=1s 4s docker exec "${container_name}" bash -lc \
  "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && timeout --kill-after=1s 2s ros2 topic echo /cmd_vel --once >/dev/null 2>&1"; then
  echo "/cmd_vel produced an unexpected message before any goal was sent." >&2
  exit 1
fi

trap - ERR
echo "READY: localization and navigation are active in ${mode}; no goal was sent and /cmd_vel is silent."
