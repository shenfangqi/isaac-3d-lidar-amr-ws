#!/usr/bin/env bash

set -euo pipefail

jetson_host="${CARBOT_JETSON_HOST:-isaac-jetson}"
jetson_fallback_ip="${CARBOT_JETSON_FALLBACK_IP:-192.168.1.109}"
jetson_identity_file="${CARBOT_JETSON_IDENTITY_FILE:-${HOME}/.ssh/id_ed25519_isaac_jetson}"
jetson_workspace="${CARBOT_JETSON_WORKSPACE:-/home/shenfq/Projects/isaac_ros-dev}"
rviz_container="carbot-real-navigation-rviz"
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

session_ip="$(remote bash -lc 'set -- ${SSH_CONNECTION}; printf "%s" "$3"')"
if [[ "${session_ip}" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
  jetson_host="${session_ip}"
  identity="$(remote bash -lc 'printf "%s|%s" "$(hostname)" "$(whoami)"')"
  if [[ "${identity}" != "ubuntu|shenfq" ]]; then
    echo "Unexpected Jetson identity after address pinning: ${identity}" >&2
    exit 1
  fi
fi

echo "Stopping physical Carbot navigation on ${jetson_host}..."

# Ask Nav2 to shut down first so an active controller releases its command
# publisher before the container is stopped.  The container stop is the
# bounded fallback when lifecycle services are already unavailable.
remote docker exec carbot-nvblox bash -lc \
  'source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; timeout 20s ros2 service call /lifecycle_manager_navigation/manage_nodes nav2_msgs/srv/ManageLifecycleNodes "{command: 4}" >/dev/null 2>&1 || true' \
  >/dev/null 2>&1 || true
remote "${jetson_workspace}/scripts/jetson_nvblox_container.sh" stop \
  >/dev/null 2>&1 || true

if docker container inspect "${rviz_container}" >/dev/null 2>&1; then
  docker stop --time 3 "${rviz_container}" >/dev/null 2>&1 || true
  if docker container inspect "${rviz_container}" >/dev/null 2>&1; then
    docker rm -f "${rviz_container}" >/dev/null 2>&1 || true
  fi
fi

# Nav2 and its command publisher are now gone.  Restarting the compensator at
# this point clears any localization emergency-stop latch without allowing a
# stale navigation command to pass through.  The phone UI is restored only
# after the fresh compensator is confirmed active, and it starts disarmed.
echo "Clearing the command-compensation safety latch after Nav2 shutdown..."
remote systemctl --user restart carbot-command-compensation.service
for _ in {1..20}; do
  if remote systemctl --user is-active --quiet carbot-command-compensation.service; then
    break
  fi
  sleep 0.25
done
if ! remote systemctl --user is-active --quiet carbot-command-compensation.service; then
  echo "Command compensation failed to restart; web teleop will remain stopped." >&2
  exit 1
fi

# Navigation and web teleop are mutually exclusive.  Restore the phone UI only
# after Nav2 has released /cmd_vel_command.
remote systemctl --user start carbot-web-teleop.service

echo "STOPPED: Nav2/RViz are down, the stale safety latch is cleared, and web teleop was restored disarmed."
