#!/usr/bin/env bash

set -euo pipefail

usage() {
  echo "Usage: $0 MAP_YAML {auto|auto-activate} [navigation-safe|navigation-diagnostic]" >&2
  echo "       $0 MAP_YAML [manual] [navigation-safe|navigation-diagnostic]  # RViz 2D Pose Estimate" >&2
}

if (( $# < 1 )); then
  usage
  exit 2
fi

map_yaml="$1"
strategy="${2:-manual}"
case "${strategy}" in
  auto|auto-activate|manual)
    if (( $# > 3 )); then
      usage
      exit 2
    fi
    mode="${3:-navigation-safe}"
    ;;
  navigation-safe|navigation-diagnostic)
    # MAP_YAML [MODE] keeps the manual RViz-pose interface.
    if (( $# > 2 )); then
      usage
      exit 2
    fi
    mode="$2"
    strategy="manual"
    ;;
  *)
    # The former fixed X/Y/YAW forms were removed: the container accepts only
    # manual, auto and auto-activate initialization.
    usage
    exit 2
    ;;
esac

if [[ "${mode}" != "navigation-safe" && "${mode}" != "navigation-diagnostic" ]]; then
  usage
  exit 2
fi
if [[ "${strategy}" != "manual" && "${mode}" != "navigation-safe" ]]; then
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

if systemctl --user is-active --quiet carbot-web-teleop.service; then
  echo 'Stopping carbot-web-teleop.service; phone teleop and Nav2 are mutually exclusive.'
  systemctl --user stop carbot-web-teleop.service
fi

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
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && python3 '${initializer}' --timeout '${CARBOT_INITIAL_POSE_TIMEOUT:-300}'"
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
