#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"
image="${CARBOT_ROS_IMAGE:-ros2-dev-humble-backup:latest}"
container="carbot-real-mapping-rviz"
display="${DISPLAY:-:1}"
xauthority="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
stop_script="${script_dir}/stop_real_robot_mapping_rviz.sh"

if [[ ! -S /tmp/.X11-unix/X${display#:} ]]; then
  echo "X11 display socket for ${display} is not available." >&2
  exit 1
fi
if [[ ! -r "${xauthority}" ]]; then
  echo "Xauthority file is not readable: ${xauthority}" >&2
  exit 1
fi

jetson_ip="${CARBOT_JETSON_IP:-$(
  getent ahostsv4 ubuntu.local | awk 'NR == 1 {print $1}'
)}"
if [[ "${jetson_ip}" != "192.168.1.109" ]]; then
  echo "Jetson address is ${jetson_ip:-unresolved}; regenerate configs/fastdds_ros_jetson.xml before launching." >&2
  exit 1
fi

"${stop_script}" >/dev/null
docker run -d --rm \
  --name "${container}" \
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
  -e XDG_RUNTIME_DIR=/tmp/runtime-carbot \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${xauthority}:/tmp/.carbot.Xauthority:ro" \
  -v "${workspace}:/workspace/ros-humble/isaac_3d_lidar_amr_ws:ro" \
  "${image}" bash -lc \
  'install -d -m 700 "${XDG_RUNTIME_DIR}"; source /opt/ros/humble/setup.bash; source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/local_setup.bash; exec rviz2 -d /workspace/ros-humble/isaac_3d_lidar_amr_ws/configs/rviz/carbot_mapping_real.rviz --ros-args -r /tf:=/carbot_workstation/tf -r /tf_static:=/carbot_workstation/tf_static' \
  >/dev/null

# Fast DDS sends its complete reciprocal discovery announcement when the
# anchor starts.  Restart it only after RViz owns workstation participant 0.
sleep 2
if ! ssh -o BatchMode=yes \
    -o ConnectTimeout=5 \
    -i "${HOME}/.ssh/id_ed25519_isaac_jetson" \
    "shenfq@${jetson_ip}" \
    'systemctl --user restart carbot-workstation-dds-anchor.service'; then
  "${stop_script}" >/dev/null
  echo "Failed to restart the Jetson DDS anchor; RViz was stopped again." >&2
  exit 1
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "${container}" 2>/dev/null || true)" != "true" ]]; then
  echo "RViz exited during startup." >&2
  docker logs --tail 80 "${container}" >&2 || true
  exit 1
fi

echo "RViz started in low-bandwidth mode (container: ${container})."
echo "Stop it with: ${stop_script}"
