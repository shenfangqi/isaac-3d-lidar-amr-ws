#!/usr/bin/env bash

set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"

jetson_host="${CARBOT_JETSON_HOST:-isaac-jetson}"
jetson_fallback_ip="${CARBOT_JETSON_FALLBACK_IP:-192.168.1.109}"
jetson_identity_file="${CARBOT_JETSON_IDENTITY_FILE:-${HOME}/.ssh/id_ed25519_isaac_jetson}"
jetson_workspace="${CARBOT_JETSON_WORKSPACE:-/home/shenfq/Projects/isaac_ros-dev}"
jetson_carbot_workspace="${CARBOT_JETSON_CARBOT_WORKSPACE:-/home/shenfq/Projects/carbot-ros2}"
default_map="${jetson_workspace}/maps/real/carbot_map_20260928_215841.yaml"
map_yaml="${CARBOT_NAV_MAP:-${default_map}}"

rviz_image="${CARBOT_ROS_IMAGE:-ros2-dev-humble-backup:latest}"
rviz_container="carbot-real-navigation-rviz"
display="${DISPLAY:-:1}"
xauthority="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
initial_pose_timeout="${CARBOT_INITIAL_POSE_TIMEOUT:-600}"
health_check_only=false
# Normal startup uses the merged guarded localization flow and activates Nav2
# only after the automatic map match passes. Manual alignment remains an
# explicit recovery mode instead of a step required on every boot.
automatic_localization=true
automatic_activation=true
stationary_validation=false
complex_route_validation=false
localization_strategy=legacy_full_rotation
map_argument_seen=false

usage() {
  cat <<EOF
Usage: $0 [--health-check] [--automatic|--automatic-activate|--manual] [--complex-route-validation] [--localization-strategy STRATEGY] [MAP_YAML]

Start saved-map navigation for the physical Carbot and open RViz.

Options:
  --health-check  Validate an already-running navigation stack without changes.
  --manual        Require RViz 2D Pose Estimate before activating Nav2.
  --automatic     Rotate, score, and stop at CANDIDATE_READY; Nav2 stays inactive.
  --automatic-activate  Rotate, validate, then activate Nav2 without sending a goal (default).
  --validation-only  Alias for --automatic (does include rotation).
  --stationary-validation  No rotation; prepare manual reference and keep Nav2 inactive.
  --complex-route-validation  Start the read-only Issue #12 advisory and evidence topics.
  --localization-strategy STRATEGY
                  legacy_full_rotation (default) or stationary_only. stationary_only
                  searches the saved map without any rotation command; use it with
                  --automatic or --automatic-activate. segmented_rotation is not
                  available until the Issue #13 motion guard exists.
  -h, --help      Show this help.

Environment overrides:
  CARBOT_JETSON_HOST           SSH alias (default: isaac-jetson)
  CARBOT_JETSON_FALLBACK_IP    Fallback after mDNS failure (default: 192.168.1.109)
  CARBOT_JETSON_IDENTITY_FILE  SSH key used by the fallback connection
  CARBOT_NAV_MAP               Jetson map YAML path
  CARBOT_INITIAL_POSE_TIMEOUT  Seconds to wait for RViz 2D Pose Estimate
  CARBOT_ROS_IMAGE             Local ROS/RViz Docker image
EOF
}

while (( $# > 0 )); do
  case "$1" in
    --health-check)
      health_check_only=true
      ;;
    --stationary-validation)
      automatic_localization=true
      automatic_activation=false
      stationary_validation=true
      ;;
    --complex-route-validation)
      complex_route_validation=true
      ;;
    --localization-strategy)
      if (( $# < 2 )); then
        echo "--localization-strategy needs a value." >&2
        exit 2
      fi
      localization_strategy="$2"
      shift
      ;;
    --validation-only|--automatic)
      automatic_localization=true
      automatic_activation=false
      ;;
    --automatic-activate)
      automatic_localization=true
      automatic_activation=true
      ;;
    --manual)
      automatic_localization=false
      automatic_activation=false
      stationary_validation=false
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -* )
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ "${map_argument_seen}" == "true" ]]; then
        echo "Only one MAP_YAML may be supplied." >&2
        exit 2
      fi
      map_yaml="$1"
      map_argument_seen=true
      ;;
  esac
  shift
done

case "${localization_strategy}" in
  legacy_full_rotation|stationary_only) ;;
  segmented_rotation)
    echo "segmented_rotation needs the Issue #13 motion guard, which is not implemented yet." >&2
    exit 2
    ;;
  *)
    echo "Unknown localization strategy: ${localization_strategy}" >&2
    exit 2
    ;;
esac
if [[ "${localization_strategy}" != "legacy_full_rotation" ]] && {
    [[ "${automatic_localization}" != "true" ]] || [[ "${stationary_validation}" == "true" ]]; }; then
  echo "--localization-strategy requires --automatic or --automatic-activate." >&2
  exit 2
fi

case "${initial_pose_timeout}" in
  ''|*[!0-9]*)
    echo "CARBOT_INITIAL_POSE_TIMEOUT must be a positive integer." >&2
    exit 2
    ;;
esac
if (( initial_pose_timeout < 1 )); then
  echo "CARBOT_INITIAL_POSE_TIMEOUT must be a positive integer." >&2
  exit 2
fi

ssh_options=(-o BatchMode=yes -o ConnectTimeout=5)
if [[ -r "${jetson_identity_file}" ]]; then
  ssh_options+=(-i "${jetson_identity_file}")
fi

remote() {
  local command=""
  local argument quoted
  for argument in "$@"; do
    printf -v quoted '%q' "${argument}"
    command+="${quoted} "
  done
  ssh "${ssh_options[@]}" "${jetson_host}" "${command% }"
}

remote_ros() {
  local command="$1"
  remote docker exec carbot-nvblox bash -lc \
    "source /opt/ros/humble/setup.bash; source /home/shenfq/Projects/lidar-mid360/ws_livox/install/setup.bash 2>/dev/null || exit 1; source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || exit 1; ${command}"
}

deploy_runtime_helpers() {
  scp "${ssh_options[@]}" \
    "${script_dir}/jetson_navigation_control.py" \
    "${jetson_host}:/tmp/carbot_navigation_control.py" \
    >/dev/null
  scp "${ssh_options[@]}" \
    "${script_dir}/jetson_nav_preflight.sh" \
    "${jetson_host}:/tmp/carbot_nav_preflight.sh" \
    >/dev/null
  scp "${ssh_options[@]}" \
    "${script_dir}/jetson_ros_freshness_check.py" \
    "${jetson_host}:/tmp/carbot_ros_freshness_check.py" \
    >/dev/null
  scp "${ssh_options[@]}" \
    "${script_dir}/jetson_navigation_wait.py" \
    "${jetson_host}:/tmp/jetson_navigation_wait.py" \
    >/dev/null
}

control() {
  remote docker exec carbot-nvblox bash -lc \
    'source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash 2>/dev/null || exit 1; exec python3 /tmp/carbot_navigation_control.py "$@"' \
    carbot-navigation-control "$@"
}

require_command() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Required command not found: $1" >&2
    exit 1
  }
}

topic_count() {
  local topic="$1"
  local endpoint="$2"
  remote_ros \
    "timeout --kill-after=1s 8s ros2 topic info '${topic}' --no-daemon" \
    | sed -n "s/^${endpoint} count: //p" \
    | head -n 1
}

check_active_lifecycle_nodes() {
  remote_ros '
    failures=0
    nodes="map_server controller_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother"
    if ros2 node list --no-daemon | grep -qx /amcl; then
      nodes="map_server amcl controller_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother"
    fi
    for node in ${nodes}; do
      state="$(timeout 8s ros2 lifecycle get "/${node}" --no-daemon 2>&1 || true)"
      printf "%s: %s\n" "${node}" "${state:-unavailable}"
      if ! grep -q "active \[3\]" <<<"${state}"; then
        failures=$((failures + 1))
      fi
    done
    exit "${failures}"
  '
}

check_action_servers() {
  local services
  services="$(remote_ros 'ros2 service list --no-daemon --include-hidden-services')"
  local action
  for action in navigate_to_pose spin backup; do
    if ! grep -qx "/${action}/_action/send_goal" <<<"${services}"; then
      echo "Missing Nav2 action server: /${action}" >&2
      return 1
    fi
  done
  echo "PASS: NavigateToPose, Spin and BackUp action servers are ready."
}

check_command_topology() {
  local cmd_publishers cmd_subscribers upstream_publishers upstream_subscribers
  cmd_publishers="$(topic_count /cmd_vel Publisher)"
  cmd_subscribers="$(topic_count /cmd_vel Subscription)"
  upstream_publishers="$(topic_count /cmd_vel_command Publisher)"
  upstream_subscribers="$(topic_count /cmd_vel_command Subscription)"

  [[ "${cmd_publishers}" == "1" ]] || {
    echo "/cmd_vel publisher count is ${cmd_publishers:-unknown}; expected 1." >&2
    return 1
  }
  [[ "${cmd_subscribers:-0}" -ge 1 ]] || {
    echo "/cmd_vel has no ESP32 base subscriber." >&2
    return 1
  }
  [[ "${upstream_publishers}" == "1" && "${upstream_subscribers}" == "1" ]] || {
    echo "/cmd_vel_command topology is ${upstream_publishers:-?}/${upstream_subscribers:-?}; expected 1 publisher and 1 subscriber." >&2
    return 1
  }
  echo "PASS: velocity topology is Nav2 -> compensator -> ESP32 (1/1 at each edge)."
}

check_complex_route_advisor() {
  if [[ "${complex_route_validation}" != "true" ]]; then
    return 0
  fi
  remote docker exec carbot-nvblox bash -lc \
    'pgrep -af "/carbot_nav_recovery/complex_route_advisor" >/dev/null'
  local attempt
  for attempt in 1 2 3; do
    # Unlike topic-info, echo waits for Fast DDS discovery and proves the
    # publisher is delivering the validation-only schema, not merely listed.
    if remote_ros \
        'ros2 topic list --no-daemon >/dev/null; timeout --kill-after=1s 12s ros2 topic echo /carbot_nav_recovery/complex_route_advisory --once --field data 2>/dev/null | grep -q "validation_only.*true"'; then
      echo "PASS: Issue #12 advisor is active in read-only validation mode."
      return 0
    fi
    sleep 2
  done
  echo "Complex-route advisor did not deliver validation-only evidence after ${attempt} attempts." >&2
  return 1
}

check_fresh_hardware() {
  remote bash -lc "
    source /opt/ros/humble/setup.bash
    source '${jetson_carbot_workspace}/install/carbot_msgs/share/carbot_msgs/package.bash'
    export ROS_DOMAIN_ID=0
    timeout 8s ros2 topic echo /wheel_ticks --once >/dev/null
    timeout 8s ros2 topic echo /odom --once >/dev/null
  "
  remote_ros 'timeout 8s ros2 topic echo /scan --once >/dev/null'
  remote_ros 'timeout 8s ros2 topic echo /scan_localization --once >/dev/null'
  echo "PASS: wheel ticks, odometry, safety scan and localization scan are fresh."
}

check_tf() {
  remote_ros 'timeout 10s ros2 run tf2_ros tf2_echo map base_footprint -r 2 -p 3 2>&1 | grep -m1 "Translation:"'
  echo "PASS: map -> base_footprint is available."
}

check_cmd_vel_silent() {
  if remote_ros 'timeout 3s ros2 topic echo /cmd_vel --once >/dev/null 2>&1'; then
    echo "/cmd_vel produced a message while no navigation goal was active." >&2
    return 1
  fi
  echo "PASS: /cmd_vel is silent before the first goal."
}

clear_command_compensation_latch() {
  # The compensator deliberately keeps a localization emergency stop latched
  # even after FAST-LIO later publishes false.  Only clear that in-process
  # latch after the stationary preflight has passed and the current retained
  # localization state explicitly says that the emergency is gone.
  control verify-compensator --timeout 15 --settle 3
  remote systemctl --user restart carbot-command-compensation.service
  control verify-compensator --timeout 15 --settle 3
  echo "PASS: stale command-compensation safety latch is cleared after localization verification."
}

run_preflight_with_readiness_retry() {
  local output status failure_lines

  set +e
  output="$(remote bash /tmp/carbot_nav_preflight.sh 2>&1)"
  status=$?
  set -e
  printf '%s\n' "${output}"
  if ((status == 0)); then
    return 0
  fi

  failure_lines="$(grep '^FAIL:' <<<"${output}" || true)"
  # Retry only failures that can be caused by cold-start DDS discovery or a
  # late publisher/TF.  Hardware/service failures, nonzero velocity,
  # localization emergency stops, and LIO stability failures never enter this
  # branch and therefore still abort immediately.
  if [[ -z "${failure_lines}" ]] || grep -Evq \
      '^FAIL: (/wheel_ticks|/odom|/livox/lidar|/scan|/scan_localization|odom -> base_footprint|base_footprint -> livox_frame|map_server is inactive|/cmd_vel has exactly one compensator publisher|/cmd_vel has a base-controller subscriber|/cmd_vel_command has no publisher before Nav2 activation|/cmd_vel_command has exactly one compensator subscriber|/odom has exactly one publisher)$' \
      <<<"${failure_lines}"; then
    return "${status}"
  fi

  echo
  echo "Readiness-only preflight failure detected; waiting 5 seconds and retrying once..."
  sleep 5
  remote bash /tmp/carbot_nav_preflight.sh
}

check_rviz() {
  if [[ "$(docker inspect -f '{{.State.Running}}' "${rviz_container}" 2>/dev/null || true)" != "true" ]]; then
    echo "RViz container ${rviz_container} is not running." >&2
    return 1
  fi
  echo "PASS: RViz container is running."
}

health_check() {
  echo "Checking active physical navigation stack..."
  remote test -e /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
  remote systemctl --user is-active --quiet \
    micro-ros-agent.service \
    carbot-wheel-odometry.service \
    carbot-mid360.service \
    carbot-description.service \
    carbot-command-compensation.service
  [[ "$(remote docker inspect -f '{{.State.Running}}' carbot-nvblox 2>/dev/null || true)" == "true" ]] || {
    echo "Jetson container carbot-nvblox is not running." >&2
    return 1
  }
  check_fresh_hardware
  check_rviz
  control health --timeout 15 --settle 4
  echo "READY: physical Carbot navigation is healthy; no goal was sent."
}

stop_local_rviz() {
  if ! docker container inspect "${rviz_container}" >/dev/null 2>&1; then
    return 0
  fi
  docker stop --time 3 "${rviz_container}" >/dev/null 2>&1 || true
  if docker container inspect "${rviz_container}" >/dev/null 2>&1; then
    docker rm -f "${rviz_container}" >/dev/null 2>&1 || true
  fi
}

start_rviz() {
  [[ -S "/tmp/.X11-unix/X${display#:}" ]] || {
    echo "X11 display socket for ${display} is unavailable." >&2
    return 1
  }
  [[ -r "${xauthority}" ]] || {
    echo "Xauthority file is unreadable: ${xauthority}" >&2
    return 1
  }

  "${script_dir}/stop_real_robot_mapping_rviz.sh" >/dev/null 2>&1 || true
  stop_local_rviz
  docker run -d --rm \
    --name "${rviz_container}" \
    --init \
    --stop-timeout 3 \
    --restart no \
    --network host \
    --ipc host \
    -e DISPLAY="${display}" \
    -e XAUTHORITY=/tmp/.carbot.Xauthority \
    -e ROS_DOMAIN_ID=0 \
    -e ROS_LOCALHOST_ONLY=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/workspace/ros-humble/isaac_3d_lidar_amr_ws/configs/fastdds_ros_jetson.xml \
    -e XDG_RUNTIME_DIR=/tmp/runtime-carbot-nav \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "${xauthority}:/tmp/.carbot.Xauthority:ro" \
    -v "${workspace}:/workspace/ros-humble/isaac_3d_lidar_amr_ws:ro" \
    "${rviz_image}" bash -lc \
    'install -d -m 700 "${XDG_RUNTIME_DIR}"; source /opt/ros/humble/setup.bash; source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/local_setup.bash; exec rviz2 -d /workspace/ros-humble/isaac_3d_lidar_amr_ws/configs/rviz/carbot_navigation.rviz --ros-args -r /tf:=/carbot_workstation/tf -r /tf_static:=/carbot_workstation/tf_static' \
    >/dev/null

  sleep 2
  remote systemctl --user restart carbot-workstation-dds-anchor.service
  for _ in {1..20}; do
    if [[ "$(docker inspect -f '{{.State.Running}}' "${rviz_container}" 2>/dev/null || true)" == "true" ]]; then
      return 0
    fi
    sleep 0.25
  done
  docker logs --tail 80 "${rviz_container}" >&2 || true
  echo "RViz exited during startup." >&2
  return 1
}

wait_for_map() {
  for _ in {1..30}; do
    if remote_ros 'timeout 2s ros2 topic echo /map --once --field info >/dev/null 2>&1'; then
      return 0
    fi
  done
  echo "Timed out waiting for the saved map." >&2
  return 1
}

wait_for_initial_pose() {
  echo
  echo "RViz is ready. Click '2D Pose Estimate', then mark the robot's true position and heading."
  echo "Waiting up to ${initial_pose_timeout} seconds for /initialpose..."
  control wait-for-pose --timeout "${initial_pose_timeout}"
}

activate_localization() {
  control activate-localization --timeout 60
}

activate_navigation() {
  control activate-navigation --timeout 90
}

run_automatic_localization() {
  # Refuse to arm an older deployed manager that can still activate Nav2.
  local validation_mode=""
  # Restarting the workstation DDS anchor for RViz can briefly invalidate the
  # CLI discovery graph even though the manager is already running.  Treat
  # only the expected "node not found" result as bounded readiness and retry;
  # every other parameter error still fails immediately.
  local attempt output status
  for attempt in {1..10}; do
    set +e
    output="$(remote_ros 'timeout 5s ros2 param get /automatic_localization_manager validation_only' 2>&1)"
    status=$?
    set -e
    if (( status == 0 )); then
      validation_mode="${output}"
      break
    fi
    if [[ "${output}" != *"Node not found"* ]]; then
      printf '%s\n' "${output}" >&2
      return "${status}"
    fi
    sleep 1
  done
  if [[ -z "${validation_mode}" ]]; then
    echo "Timed out waiting for /automatic_localization_manager discovery." >&2
    return 1
  fi
  local expected_validation='Boolean value is: True'
  if [[ "${automatic_activation}" == "true" ]]; then
    expected_validation='Boolean value is: False'
  fi
  if [[ "${validation_mode}" != *"${expected_validation}"* ]]; then
    echo "Automatic-localization activation mode does not match the requested workflow." >&2
    return 1
  fi
  local deployed_strategy
  deployed_strategy="$(remote_ros 'timeout 5s ros2 param get /automatic_localization_manager localization_strategy' 2>&1)" || {
    printf '%s\n' "${deployed_strategy}" >&2
    echo "The deployed manager does not report a localization strategy; rebuild the Jetson workspace." >&2
    return 1
  }
  if [[ "${deployed_strategy}" != *"String value is: ${localization_strategy}"* ]]; then
    echo "Deployed localization strategy (${deployed_strategy}) does not match ${localization_strategy}." >&2
    return 1
  fi
  local service=/automatic_localization/start
  if [[ "${stationary_validation}" == "true" ]]; then
    service=/automatic_localization/prepare_stationary
  fi
  remote_ros \
    "timeout --kill-after=1s 30s ros2 service call ${service} std_srvs/srv/Trigger '{}'" || return 1
  remote docker exec carbot-nvblox bash -lc \
    "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec python3 /tmp/jetson_navigation_wait.py --timeout ${initial_pose_timeout}"
}

require_command docker
require_command ssh
require_command scp
require_command sed
require_command grep
require_command flock

if [[ "${health_check_only}" != "true" ]]; then
  navigation_lock="${XDG_RUNTIME_DIR:-/tmp}/carbot-real-navigation-start.lock"
  exec {navigation_lock_fd}>"${navigation_lock}"
  if ! flock -n "${navigation_lock_fd}"; then
    echo "Another real-navigation startup is already running. Do not start it twice; finish 2D Pose Estimate in the existing RViz window or run ./stop_real_nav.sh first." >&2
    exit 1
  fi

  if [[ "$(docker inspect -f '{{.State.Running}}' "${rviz_container}" 2>/dev/null || true)" == "true" ]]; then
    echo "Real navigation RViz is already running. Use ./start_real_nav.sh --health-check, or ./stop_real_nav.sh before starting again." >&2
    exit 1
  fi
fi

if ! remote true >/dev/null 2>&1; then
  if [[ -n "${CARBOT_JETSON_HOST+x}" ]]; then
    echo "Cannot reach explicitly configured Jetson host: ${jetson_host}" >&2
    exit 1
  fi
  [[ -r "${jetson_identity_file}" ]] || {
    echo "Jetson mDNS failed and fallback key is unreadable: ${jetson_identity_file}" >&2
    exit 1
  }
  echo "Warning: ${jetson_host} is unreachable; trying verified fallback ${jetson_fallback_ip}." >&2
  jetson_host="${jetson_fallback_ip}"
fi

identity="$(remote bash -lc 'printf "%s|%s" "$(hostname)" "$(whoami)"')"
if [[ "${identity}" != "ubuntu|shenfq" ]]; then
  echo "Unexpected Jetson identity: ${identity}" >&2
  exit 1
fi

# mDNS may disappear while a long startup is waiting for the operator.  Pin
# the rest of this run to the server address of the already verified SSH
# connection instead of resolving ubuntu.local for every command.
session_ip="$(remote bash -lc 'set -- ${SSH_CONNECTION}; printf "%s" "$3"')"
if [[ "${session_ip}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
  jetson_host="${session_ip}"
  identity="$(remote bash -lc 'printf "%s|%s" "$(hostname)" "$(whoami)"')"
  if [[ "${identity}" != "ubuntu|shenfq" ]]; then
    echo "Unexpected Jetson identity after address pinning: ${identity}" >&2
    exit 1
  fi
fi
deploy_runtime_helpers

if [[ "${health_check_only}" == "true" ]]; then
  health_check
  exit 0
fi

startup_complete=false
web_teleop_was_active=false

startup_failed() {
  local status=$?
  trap - ERR INT TERM
  if [[ "${startup_complete}" != "true" ]]; then
    echo "Startup failed; returning the robot to a non-navigation state." >&2
    remote "${jetson_workspace}/scripts/jetson_nvblox_container.sh" stop >/dev/null 2>&1 || true
    stop_local_rviz
    # The failure itself may have latched the localization stop in this
    # long-running host service.  With Nav2 and the localization publisher now
    # gone, restarting it is safe and prevents a failed attempt contaminating
    # the next startup.
    remote systemctl --user restart carbot-command-compensation.service >/dev/null 2>&1 || true
    if [[ "${web_teleop_was_active}" == "true" ]]; then
      remote systemctl --user start carbot-web-teleop.service >/dev/null 2>&1 || true
    fi
  fi
  exit "${status}"
}
trap startup_failed ERR INT TERM

if remote systemctl --user is-active --quiet carbot-web-teleop.service; then
  web_teleop_was_active=true
fi

echo "Target: ${identity} (${jetson_host})"
echo "Map: ${map_yaml}"
echo "Localization: $(if [[ "${automatic_localization}" == "true" ]]; then echo automatic; else echo manual; fi)"
if [[ "${automatic_localization}" == "true" ]]; then
  echo "Localization strategy: ${localization_strategy}"
fi
echo "Complex-route advisor: $(if [[ "${complex_route_validation}" == "true" ]]; then echo validation-only; else echo disabled; fi)"
remote test -f "${map_yaml}"

echo "[1/8] Disabling the conflicting web teleop publisher..."
remote systemctl --user stop carbot-web-teleop.service

echo "[2/8] Ensuring the hardware communication owners are active..."
remote systemctl --user start \
  micro-ros-agent.service \
  carbot-wheel-odometry.service \
  carbot-command-compensation.service

echo "[3/8] Recreating the inactive saved-map navigation container..."
initialization_mode=manual
complex_route_mode=none
if [[ "${automatic_localization}" == "true" ]]; then
  initialization_mode=auto
fi
if [[ "${automatic_activation}" == "true" ]]; then
  initialization_mode=auto-activate
fi
if [[ "${complex_route_validation}" == "true" ]]; then
  complex_route_mode=complex-route-validation
fi
remote "${jetson_workspace}/scripts/jetson_nvblox_container.sh" \
  recreate navigation-safe "${map_yaml}" "${initialization_mode}" "${complex_route_mode}" \
  "${localization_strategy}" >/dev/null

echo "[4/8] Running the non-motion hardware preflight..."
run_preflight_with_readiness_retry

echo "[5/8] Clearing any stale velocity safety latch after verified localization..."
clear_command_compensation_latch

if [[ "${automatic_localization}" == "true" ]]; then
  echo "[6/8] Opening RViz before guarded automatic localization..."
  start_rviz
  echo "[7/8] Running guarded automatic saved-map localization..."
  validation_result=0
  run_automatic_localization || validation_result=$?
  if [[ "${automatic_activation}" == "true" ]] && (( validation_result == 0 )); then
    echo "[8/8] Validating active Nav2 with no navigation goal..."
  elif [[ "${automatic_activation}" != "true" ]] && (( validation_result == 5 || validation_result == 3 )); then
    check_rviz
    startup_complete=true
    trap - ERR INT TERM
    echo "VALIDATION_ONLY: Nav2 remains inactive; no navigation goal is permitted."
    echo "Inspect candidate/status and save evidence; use 2D Pose Estimate for manual recovery."
    echo "Stop with: ${script_dir}/stop_real_robot_navigation_rviz.sh"
    exit 0
  else
    echo "Automatic localization did not reach the requested terminal state (code ${validation_result})." >&2
    false  # Invoke the maintained ERR cleanup before exiting.
  fi
else
  echo "[6/8] Activating the map and opening RViz..."
  activate_localization
  start_rviz
  echo "[7/8] Waiting for manual saved-map alignment..."
  wait_for_initial_pose
  echo "[8/8] Activating and validating Nav2..."
  activate_navigation
fi
check_rviz
control health --timeout 15 --settle 4
check_complex_route_advisor

startup_complete=true
trap - ERR INT TERM
echo
echo "READY: RViz navigation is active; no goal was sent and no nonzero /cmd_vel was observed."
echo "Use RViz '2D Goal Pose' only after confirming that Scan aligns with the saved map."
echo "Stop with: ${script_dir}/stop_real_robot_navigation_rviz.sh"
