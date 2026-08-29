#!/usr/bin/env bash

set -Eeuo pipefail

ROS_ROOT="${1:-}"

usage() { echo "Usage: $0 /absolute/path/to/ros-humble" >&2; }
die() { echo "[ERROR] $*" >&2; exit 1; }

if [[ -z "$ROS_ROOT" || "$ROS_ROOT" != /* ]]; then
  usage
  exit 2
fi

[[ -d "$ROS_ROOT/isaac_3d_lidar_amr_ws" ]] || die "Workspace not found below $ROS_ROOT"
command -v docker >/dev/null 2>&1 || die "Docker is not installed"
docker info >/dev/null 2>&1 || die "Docker daemon is unavailable to the current user"
[[ "${ACCEPT_NVIDIA_EULA:-}" == "Y" ]] || die "Review the NVIDIA terms, then rerun with ACCEPT_NVIDIA_EULA=Y if accepted"

PROJECTS_ROOT="$(dirname -- "$ROS_ROOT")"
HOST_USER_NAME="$(id -un)"
HOST_USER_UID="$(id -u)"
HOST_USER_GID="$(id -g)"
HOST_USER_HOME="$(getent passwd "$HOST_USER_NAME" | awk -F: '{print $6}')"
[[ "$HOST_USER_HOME" == /* && "$HOST_USER_HOME" != / ]] || die "Cannot resolve a safe user home directory"

ISAAC_DATA_ROOT="${AMR_ISAAC_DATA_ROOT:-${HOST_USER_HOME}/docker/isaac-sim}"
[[ "$ISAAC_DATA_ROOT" == /* && "$ISAAC_DATA_ROOT" != / ]] || die "AMR_ISAAC_DATA_ROOT must be a non-root absolute path"

images=(
  isaac-sim-backup-before-ipc-fix:latest
  isaac-ros-nvblox-backup:latest
  ros2-dev-humble-backup:latest
)
containers=(isaac-sim isaac-ros-nvblox ros2-dev-humble)

for image in "${images[@]}"; do
  docker image inspect "$image" >/dev/null 2>&1 || die "Required image is not loaded: $image"
done

for container in "${containers[@]}"; do
  if docker container inspect "$container" >/dev/null 2>&1; then
    die "Container already exists and will not be overwritten: $container"
  fi
done

mkdir -p -- \
  "$ISAAC_DATA_ROOT/cache/kit" \
  "$ISAAC_DATA_ROOT/cache/ov" \
  "$ISAAC_DATA_ROOT/cache/pip" \
  "$ISAAC_DATA_ROOT/cache/glcache" \
  "$ISAAC_DATA_ROOT/cache/computecache" \
  "$ISAAC_DATA_ROOT/logs" \
  "$ISAAC_DATA_ROOT/data" \
  "$ISAAC_DATA_ROOT/documents"

DISPLAY_VALUE="${DISPLAY:-:0}"
PRIVACY_CHOICE="${NVIDIA_PRIVACY_CONSENT:-N}"
[[ "$PRIVACY_CHOICE" == "Y" || "$PRIVACY_CHOICE" == "N" ]] || die "NVIDIA_PRIVACY_CONSENT must be Y or N"

docker create -it \
  --name isaac-sim \
  --label isaac-amr.role=simulation \
  --entrypoint bash \
  --runtime=nvidia \
  --gpus all \
  --network host \
  --ipc host \
  -e ACCEPT_EULA=Y \
  -e PRIVACY_CONSENT="$PRIVACY_CHOICE" \
  -e DISPLAY="$DISPLAY_VALUE" \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PROJECTS_ROOT:/workspace:rw" \
  -v "$ISAAC_DATA_ROOT/cache/kit:/isaac-sim/kit/cache:rw" \
  -v "$ISAAC_DATA_ROOT/cache/ov:/root/.cache/ov:rw" \
  -v "$ISAAC_DATA_ROOT/cache/pip:/root/.cache/pip:rw" \
  -v "$ISAAC_DATA_ROOT/cache/glcache:/root/.cache/nvidia/GLCache:rw" \
  -v "$ISAAC_DATA_ROOT/cache/computecache:/root/.nv/ComputeCache:rw" \
  -v "$ISAAC_DATA_ROOT/logs:/root/.nvidia-omniverse/logs:rw" \
  -v "$ISAAC_DATA_ROOT/data:/root/.local/share/ov/data:rw" \
  -v "$ISAAC_DATA_ROOT/documents:/root/Documents:rw" \
  isaac-sim-backup-before-ipc-fix:latest >/dev/null

docker create -it \
  --name isaac-ros-nvblox \
  --label isaac-amr.role=nvblox \
  --runtime=nvidia \
  --gpus all \
  --network host \
  --ipc host \
  -e USER="$HOST_USER_NAME" \
  -e HOST_USER_UID="$HOST_USER_UID" \
  -e HOST_USER_GID="$HOST_USER_GID" \
  -e DISPLAY="$DISPLAY_VALUE" \
  -e ROS_DOMAIN_ID=0 \
  -v "$ROS_ROOT:/workspace/ros-humble:rw" \
  isaac-ros-nvblox-backup:latest >/dev/null

docker create -it \
  --name ros2-dev-humble \
  --label isaac-amr.role=navigation \
  --runtime=nvidia \
  --gpus all \
  --network host \
  --ipc host \
  -e DISPLAY="$DISPLAY_VALUE" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$ROS_ROOT:/workspace/ros-humble:rw" \
  ros2-dev-humble-backup:latest >/dev/null

echo "[ OK ] Created isaac-sim, isaac-ros-nvblox, and ros2-dev-humble."
echo "[INFO] Containers remain stopped; build the overlay before starting navigation."
