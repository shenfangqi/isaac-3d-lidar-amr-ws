#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: $0 MAP_YAML {auto|auto-activate} [navigation-safe|navigation-diagnostic]" >&2
  echo "       $0 MAP_YAML fixed X_M Y_M YAW_RAD [navigation-safe|navigation-diagnostic]" >&2
  echo "       $0 MAP_YAML X_M Y_M YAW_RAD [navigation-safe|navigation-diagnostic]  # legacy" >&2
}

if (( $# < 2 )); then
  usage
  exit 2
fi

map_yaml="$1"
strategy="$2"
initial_x=""
initial_y=""
initial_yaw=""

case "${strategy}" in
  auto|auto-activate)
    if (( $# > 3 )); then
      usage
      exit 2
    fi
    mode="${3:-navigation-safe}"
    ;;
  fixed)
    if (( $# < 5 || $# > 6 )); then
      usage
      exit 2
    fi
    initial_x="$3"
    initial_y="$4"
    initial_yaw="$5"
    mode="${6:-navigation-safe}"
    ;;
  *)
    # Preserve the original MAP X Y YAW [MODE] interface.
    if (( $# < 4 || $# > 5 )); then
      usage
      exit 2
    fi
    initial_x="$2"
    initial_y="$3"
    initial_yaw="$4"
    strategy="fixed"
    mode="${5:-navigation-safe}"
    ;;
esac

if [[ "${mode}" != "navigation-safe" && "${mode}" != "navigation-diagnostic" ]]; then
  usage
  exit 2
fi
if [[ "${strategy}" != "fixed" && "${mode}" != "navigation-safe" ]]; then
  echo "Automatic physical localization requires navigation-safe mode." >&2
  exit 2
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
container_name="carbot-nvblox"
container_workspace="/workspaces/isaac_ros-dev"
initializer="${container_workspace}/scripts/jetson_navigation_initialize.py"
waiter="${container_workspace}/scripts/jetson_navigation_wait.py"

stop_on_error() {
  echo "Navigation initialization failed; stopping ${container_name}." >&2
  "${script_dir}/jetson_nvblox_container.sh" stop >/dev/null 2>&1 || true
}
trap stop_on_error ERR

"${script_dir}/jetson_nvblox_container.sh" recreate \
  "${mode}" "${map_yaml}" "${strategy}" >/dev/null
"${script_dir}/jetson_nav_preflight.sh"

if [[ "${strategy}" == "auto" || "${strategy}" == "auto-activate" ]]; then
  validation_mode="$(docker exec "${container_name}" bash -lc     "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && timeout 10s ros2 param get /automatic_localization_manager validation_only")"
  expected_validation='Boolean value is: True'
  [[ "${strategy}" == "auto-activate" ]] && expected_validation='Boolean value is: False'
  [[ "${validation_mode}" == *"${expected_validation}"* ]] || {
    echo "Automatic-localization activation mode does not match the requested workflow." >&2
    exit 1
  }
  docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && timeout --kill-after=1s 10s ros2 service call /automatic_localization/start std_srvs/srv/Trigger '{}'"
  set +e
  docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && python3 '${waiter}' --timeout 230"
  wait_result=$?
  set -e
  if [[ "${strategy}" == "auto" ]] && (( wait_result == 5 )); then
    trap - ERR
    echo "CANDIDATE_READY: validation only; Nav2 remains inactive. Confirm real pose and scan alignment."
    exit 0
  elif (( wait_result == 3 )); then
    trap - ERR
    echo "STOPPED: automatic localization is not trustworthy." >&2
    echo "In RViz, click 2D Pose Estimate once; navigation remains inactive until the pose passes verification." >&2
    exit 3
  elif (( wait_result != 0 )); then
    trap - ERR
    echo "Automatic localization did not reach READY; the guarded manager remains responsible for zero velocity." >&2
    exit "${wait_result}"
  fi
else
  docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && python3 '${initializer}' --x '${initial_x}' --y '${initial_y}' --yaw '${initial_yaw}'"
fi

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
