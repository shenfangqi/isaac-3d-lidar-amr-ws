#!/usr/bin/env bash

set -Eeuo pipefail

ROS_ROOT="${1:-}"
FAILURES=0
WARNINGS=0

usage() { echo "Usage: $0 /absolute/path/to/ros-humble" >&2; }
pass() { echo "[ OK ] $*"; }
fail() { echo "[FAIL] $*" >&2; FAILURES=$((FAILURES + 1)); }
warn() { echo "[WARN] $*" >&2; WARNINGS=$((WARNINGS + 1)); }

if [[ -z "$ROS_ROOT" || "$ROS_ROOT" != /* ]]; then
  usage
  exit 2
fi

PROJECT_ROOT="${ROS_ROOT}/isaac_3d_lidar_amr_ws"

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "24.04" ]]; then
    pass "Ubuntu 24.04 host"
  else
    fail "Expected Ubuntu 24.04; found ${PRETTY_NAME:-unknown}"
  fi
else
  fail "Cannot read /etc/os-release"
fi

if [[ "$(uname -m)" == "x86_64" ]]; then
  pass "x86_64 architecture"
else
  fail "Expected x86_64; found $(uname -m)"
fi

ram_kib="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo)"
if (( ram_kib >= 32 * 1024 * 1024 )); then
  pass "System RAM is at least 32 GiB"
else
  fail "System RAM is below the 32 GiB Isaac Sim minimum"
fi

disk_probe="$ROS_ROOT"
while [[ ! -e "$disk_probe" && "$disk_probe" != / ]]; do
  disk_probe="$(dirname -- "$disk_probe")"
done
disk_kib="$(df -Pk -- "$disk_probe" 2>/dev/null | awk 'NR==2 {print $4}')"
if [[ -n "$disk_kib" ]] && (( disk_kib >= 120 * 1024 * 1024 )); then
  pass "At least 120 GiB free on workspace filesystem"
elif [[ -n "$disk_kib" ]]; then
  fail "Less than 120 GiB free on workspace filesystem"
else
  warn "Cannot measure workspace free space"
fi

if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
  pass "NVIDIA driver responds"
  vram_mib="$(nvidia-smi --query-gpu=memory.total --format=csv,noheader,nounits | head -n1 | tr -d ' ')"
  if [[ "$vram_mib" =~ ^[0-9]+$ ]] && (( vram_mib >= 8192 )); then
    pass "GPU VRAM is at least 8 GiB"
  else
    fail "GPU VRAM is below 8 GiB or could not be read"
  fi
else
  fail "nvidia-smi is missing or the NVIDIA driver is not working"
fi

if command -v docker >/dev/null 2>&1; then
  if docker info >/dev/null 2>&1; then
    pass "Docker daemon is accessible by the current user"
    docker_root="$(docker info --format '{{.DockerRootDir}}' 2>/dev/null)"
    docker_disk_kib="$(df -Pk -- "$docker_root" 2>/dev/null | awk 'NR==2 {print $4}')"
    if [[ -n "$docker_disk_kib" ]] && (( docker_disk_kib >= 120 * 1024 * 1024 )); then
      pass "At least 120 GiB free on Docker data filesystem"
    elif [[ -n "$docker_disk_kib" ]]; then
      fail "Less than 120 GiB free on Docker data filesystem"
    else
      warn "Cannot measure Docker data filesystem free space"
    fi
  else
    fail "Docker exists but the daemon is unavailable to the current user"
  fi
else
  fail "Docker Engine is not installed"
fi

if command -v nvidia-ctk >/dev/null 2>&1; then
  pass "NVIDIA Container Toolkit CLI is installed"
else
  fail "nvidia-ctk is not installed"
fi

display_number="${DISPLAY:-}"
display_number="${display_number#*:}"
display_number="${display_number%%.*}"
if [[ -n "${DISPLAY:-}" && -S "/tmp/.X11-unix/X${display_number}" ]]; then
  pass "X11 display ${DISPLAY} is available for RViz"
else
  warn "No matching X11 DISPLAY socket; use START_RVIZ=0 or establish a desktop X11 session"
fi

required_files=(
  "$ROS_ROOT/start_nav_all.sh"
  "$ROS_ROOT/stop_nav_all.sh"
  "$ROS_ROOT/cyclonedds_ros_local.xml"
  "$PROJECT_ROOT/isaac_sim/auto_play_warehouse.py"
  "$PROJECT_ROOT/isaac_sim/streaming_auto_play.py"
  "$PROJECT_ROOT/isaac_sim/usd/warehouse_3d_nav_origin_carter.usd"
  "$PROJECT_ROOT/maps/nvblox/warehouse_v3.nvblx"
  "$PROJECT_ROOT/maps/2d/warehouse_v3.pgm"
  "$PROJECT_ROOT/maps/2d/warehouse_v3.yaml"
  "$PROJECT_ROOT/configs/nav2_params.yaml"
  "$PROJECT_ROOT/configs/amcl_params.yaml"
)

for required_file in "${required_files[@]}"; do
  if [[ -s "$required_file" ]]; then
    pass "Artifact exists: ${required_file#"$ROS_ROOT"/}"
  else
    fail "Missing or empty artifact: $required_file"
  fi
done

check_hash() {
  local expected="$1"
  local file="$2"
  if [[ ! -f "$file" ]]; then
    return
  fi
  local actual
  actual="$(sha256sum -- "$file" | awk '{print $1}')"
  if [[ "$actual" == "$expected" ]]; then
    pass "Validated baseline hash: ${file#"$PROJECT_ROOT"/}"
  else
    fail "Baseline hash mismatch: $file"
  fi
}

check_hash cae7664f29c8ced82c92efb17086cf3223646c102b7507866cb0f1e36eef47ca "$PROJECT_ROOT/maps/nvblox/warehouse_v3.nvblx"
check_hash a9e1e47bfad1292c0a1536425d89057c0604611aad9c05e31663dd3bd22bbe47 "$PROJECT_ROOT/maps/2d/warehouse_v3.pgm"
check_hash 5196d371292dd62781ef8c7433fcb4dc934a6b4d3b33e92d9198e942cbf524dd "$PROJECT_ROOT/maps/2d/warehouse_v3.yaml"
check_hash 4c7fed45c773862e75b347b0a2b6a33a695427c79482d3feba38ce13e81700aa "$PROJECT_ROOT/isaac_sim/usd/warehouse_3d_nav_origin_carter.usd"
check_hash a1c0afd820a94bfa665d7cc2e62efc8e7a7224a9a62bbdbb63aa013d0c06a781 "$PROJECT_ROOT/isaac_sim/auto_play_warehouse.py"
check_hash e2bce168de4dc4956c3b4698e62697c2f5045516df3cb6f878c0dd9fc6100666 "$PROJECT_ROOT/isaac_sim/streaming_auto_play.py"
check_hash 63253a3662ccbbbeeac15c34f1f707378da1a5acb48b0538eff202c0dbeabe45 "$PROJECT_ROOT/configs/nav2_params.yaml"
check_hash 1b3c8193ca5a41bf24cd2a9f151bfdfd81eca5da184fc1b5684094bdedf294f5 "$PROJECT_ROOT/configs/amcl_params.yaml"
check_hash 6b3514ba7e24469ba4b676a72faa3b0c0e3a0f873eeb2d72f5c73cfc907a6e30 "$PROJECT_ROOT/launch/nvblox_with_map.launch.py"
check_hash b9765f9d3da5447d3e1c3037b74f964e449500996f5d50763befe74b20893e71 "$PROJECT_ROOT/launch/nav_stack.launch.py"
check_hash 5889c4f07657c18ec7ae521aad31550931006a60a6bb93f2846f157b64195e31 "$ROS_ROOT/start_nav_all.sh"
check_hash ff0191a323838f44f305ee9614b1c5b19e3902fa3bd47934692a9c1a231834f4 "$ROS_ROOT/stop_nav_all.sh"
check_hash 6c21c9fb4942ff9e1c4b6930caf824c72eceef73bfce0ed2eadbff6cc5103b38 "$ROS_ROOT/cyclonedds_ros_local.xml"

images=(
  isaac-sim-backup-before-ipc-fix:latest
  isaac-ros-nvblox-backup:latest
  ros2-dev-humble-backup:latest
)

if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
  for image in "${images[@]}"; do
    if docker image inspect "$image" >/dev/null 2>&1; then
      pass "Docker image loaded: $image"
    else
      fail "Required Docker image not loaded: $image"
    fi
  done
fi

if (( FAILURES > 0 )); then
  echo "[FAIL] Preflight finished with ${FAILURES} failure(s) and ${WARNINGS} warning(s)." >&2
  exit 1
fi

echo "[ OK ] Preflight passed with ${WARNINGS} warning(s)."
