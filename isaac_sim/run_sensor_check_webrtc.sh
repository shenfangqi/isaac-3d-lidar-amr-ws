#!/usr/bin/env bash
# Run on the workstation host. This is an Isaac Lab-only visual inspection app.
set -euo pipefail
container_name=isaac-sim
workspace_dir=/workspace/ros-humble/isaac_3d_lidar_amr_ws
if [[ "$(docker inspect -f '{{.State.Running}}' "$container_name")" != true ]]; then
  docker start "$container_name" >/dev/null
fi
if docker exec "$container_name" pgrep -f '^/isaac-sim/(kit/kit|kit/python/bin/python3)' >/dev/null; then
  echo 'Isaac Sim/Isaac Lab is already running in isaac-sim. Stop that process before starting the sensor check.' >&2
  exit 1
fi
printf '%s\n' 'Starting the sensor dashboard. Wait for CARBOT_SENSOR_CHECK_READY, then connect your WebRTC client to 127.0.0.1.'
# Keep this terminal open; use stop_sensor_check.sh to stop the server.
# No ROS commands are published.
exec docker exec -i -e TERM=xterm -e PYTHONUNBUFFERED=1 \
  -e PUBLIC_IP="${PUBLIC_IP:-127.0.0.1}" "$container_name" \
  bash -lc 'cd "$1" && exec /workspace/IsaacLab/isaaclab.sh -p isaac_lab/carbot_env/scripts/inspect_sensors.py --livestream 1 "${@:2}"' \
  bash "$workspace_dir" "$@"
