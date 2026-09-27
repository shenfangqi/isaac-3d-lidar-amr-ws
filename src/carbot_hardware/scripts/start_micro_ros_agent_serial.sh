#!/usr/bin/env bash

set -euo pipefail

default_device="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
device="${CARBOT_MICROROS_SERIAL_DEVICE:-${default_device}}"
baudrate="${CARBOT_MICROROS_BAUDRATE:-921600}"
verbose="${MICRO_ROS_AGENT_VERBOSE:-4}"
agent_setup="${MICRO_ROS_AGENT_SETUP:-/home/shenfq/Projects/micro_ros_agent_ws/install/setup.bash}"
wait_seconds="${CARBOT_MICROROS_DEVICE_WAIT_SECONDS:-2}"

case "${baudrate}" in
  ''|*[!0-9]*)
    echo "Invalid CARBOT_MICROROS_BAUDRATE: ${baudrate}" >&2
    exit 64
    ;;
esac

case "${wait_seconds}" in
  ''|*[!0-9]*)
    echo "Invalid CARBOT_MICROROS_DEVICE_WAIT_SECONDS: ${wait_seconds}" >&2
    exit 64
    ;;
esac

if [[ "${wait_seconds}" == "0" ]]; then
  echo "CARBOT_MICROROS_DEVICE_WAIT_SECONDS must be greater than zero" >&2
  exit 64
fi

set +u
source /opt/ros/humble/setup.bash
if [[ ! -r "${agent_setup}" ]]; then
  echo "micro-ROS Agent setup is not readable: ${agent_setup}" >&2
  exit 66
fi
source "${agent_setup}"
set -u

last_notice_seconds=-30
while true; do
  resolved_device="$(readlink -f -- "${device}" 2>/dev/null || true)"
  if [[ -n "${resolved_device}" && -c "${resolved_device}" && -r "${device}" && -w "${device}" ]]; then
    break
  fi

  if ((SECONDS - last_notice_seconds >= 30)); then
    echo "Waiting for readable/writable Carbot serial device: ${device}" >&2
    last_notice_seconds=${SECONDS}
  fi
  sleep "${wait_seconds}"
done

echo "Starting Carbot micro-ROS Agent over USB serial"
echo "Device: ${device} -> ${resolved_device}"
echo "Baudrate: ${baudrate}; ROS domain: ${ROS_DOMAIN_ID:-0}"
echo "This transport bridge never publishes /cmd_vel."

exec ros2 run micro_ros_agent micro_ros_agent serial \
  --dev "${device}" \
  --baudrate "${baudrate}" \
  --verbose "${verbose}"
