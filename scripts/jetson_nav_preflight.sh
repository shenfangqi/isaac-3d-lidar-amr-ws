#!/usr/bin/env bash

set -uo pipefail

container_name="carbot-nvblox"
workspace="/home/shenfq/Projects/isaac_ros-dev"
container_workspace="/workspaces/isaac_ros-dev"
carbot_workspace="/home/shenfq/Projects/carbot-ros2"
esp32_serial_device="${CARBOT_MICROROS_SERIAL_DEVICE:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}"
esp32_serial_baudrate="${CARBOT_MICROROS_BAUDRATE:-921600}"
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

check_esp32_serial_transport() {
  local resolved_device=""
  local properties=""
  local unit_text=""
  local unit_environment=""

  resolved_device="$(readlink -f -- "${esp32_serial_device}" 2>/dev/null || true)"
  if [[ -z "${resolved_device}" || ! -c "${resolved_device}" ]]; then
    fail "ESP32 USB serial device is unavailable: ${esp32_serial_device}"
  elif [[ ! -r "${esp32_serial_device}" || ! -w "${esp32_serial_device}" ]]; then
    fail "ESP32 USB serial device is not readable/writable by $(id -un): ${esp32_serial_device}"
  else
    properties="$(udevadm info --query=property --name="${resolved_device}" 2>/dev/null || true)"
    if grep -qx 'ID_VENDOR_ID=10c4' <<<"${properties}" \
        && grep -qx 'ID_MODEL_ID=ea60' <<<"${properties}" \
        && grep -qx 'ID_SERIAL_SHORT=0001' <<<"${properties}"; then
      pass "ESP32 CP2102 is available at ${esp32_serial_device} -> ${resolved_device}"
    else
      fail "serial device identity does not match CP2102 10c4:ea60 serial 0001"
    fi
  fi

  unit_text="$(systemctl --user cat micro-ros-agent.service 2>/dev/null || true)"
  unit_environment="$(systemctl --user show micro-ros-agent.service --property=Environment --value 2>/dev/null || true)"
  if grep -Fq 'start_micro_ros_agent_serial.sh' <<<"${unit_text}" \
      && grep -Fq "CARBOT_MICROROS_SERIAL_DEVICE=${esp32_serial_device}" <<<"${unit_environment}" \
      && grep -Fq "CARBOT_MICROROS_BAUDRATE=${esp32_serial_baudrate}" <<<"${unit_environment}"; then
    pass "micro-ros-agent.service uses USB serial at ${esp32_serial_baudrate} baud"
  else
    fail "micro-ros-agent.service is not configured for the expected USB serial transport"
  fi

  if ss -H -lun 2>/dev/null | awk '{print $4}' | grep -Eq '(^|:)8888$'; then
    fail "legacy UDP micro-ROS Agent is still listening on port 8888"
  else
    pass "legacy UDP port 8888 is not listening"
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

check_esp32_serial_transport
check_user_service micro-ros-agent.service
check_user_service carbot-wheel-odometry.service
check_user_service carbot-mid360.service
check_user_service carbot-description.service
check_user_service carbot-command-compensation.service
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
command_publishers="$(topic_count /cmd_vel_command Publisher)"
command_subscribers="$(topic_count /cmd_vel_command Subscription)"
if [[ "${cmd_publishers:-x}" == "1" ]]; then
  pass "/cmd_vel has exactly one compensator publisher"
else
  fail "/cmd_vel publisher count is ${cmd_publishers:-unknown}; expected 1"
fi
if [[ "${cmd_subscribers:-0}" -ge 1 ]] 2>/dev/null; then
  pass "/cmd_vel has a base-controller subscriber"
else
  fail "/cmd_vel has no base-controller subscriber"
fi
if [[ "${command_publishers:-x}" == "0" ]]; then
  pass "/cmd_vel_command has no publisher before Nav2 activation"
else
  fail "/cmd_vel_command publisher count is ${command_publishers:-unknown}; expected 0"
fi
if [[ "${command_subscribers:-x}" == "1" ]]; then
  pass "/cmd_vel_command has exactly one compensator subscriber"
else
  fail "/cmd_vel_command subscriber count is ${command_subscribers:-unknown}; expected 1"
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
