#!/usr/bin/env bash

set -euo pipefail

container_name="carbot-nvblox"
workspace="/home/shenfq/Projects/isaac_ros-dev"
container_workspace="/workspaces/isaac_ros-dev"
default_output_dir="${workspace}/maps/real"

usage() {
  echo "Usage: $0 [map_name] [output_directory]" >&2
  echo "The output directory must be inside ${workspace}." >&2
}

map_name="${1:-carbot_$(date +%Y%m%d_%H%M%S)}"
output_dir="${2:-${default_output_dir}}"

if [[ "${map_name}" == "-h" || "${map_name}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ ! "${map_name}" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
  echo "Invalid map name: ${map_name}" >&2
  usage
  exit 2
fi

output_dir="$(realpath -m "${output_dir}")"
if [[ ! "${output_dir}" =~ ^[A-Za-z0-9_./-]+$ ]]; then
  echo "Output directory contains unsupported characters: ${output_dir}" >&2
  exit 2
fi
case "${output_dir}/" in
  "${workspace}/"*) ;;
  *)
    echo "Refusing output outside ${workspace}: ${output_dir}" >&2
    exit 2
    ;;
esac

if [[ "$(docker inspect -f '{{.State.Running}}' "${container_name}" 2>/dev/null || true)" != "true" ]]; then
  echo "Container ${container_name} is not running. Start it in mapping mode first:" >&2
  echo "  ${workspace}/scripts/jetson_nvblox_container.sh recreate mapping" >&2
  exit 1
fi

mkdir -p "${output_dir}"
container_output_dir="${container_workspace}${output_dir#${workspace}}"
map_base="${container_output_dir}/${map_name}"

ros2_in_container() {
  docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && $*"
}

require_service() {
  local service_name="$1"
  if ! ros2_in_container "ros2 service list" | grep -Fxq "${service_name}"; then
    echo "Required service is unavailable: ${service_name}" >&2
    exit 1
  fi
}

call_and_check() {
  local label="$1"
  local service_name="$2"
  local service_type="$3"
  local request="$4"
  local expected="$5"
  local max_attempts="${6:-1}"
  local attempt
  local output

  for ((attempt = 1; attempt <= max_attempts; attempt++)); do
    echo "Saving ${label} (attempt ${attempt}/${max_attempts})..."
    if output="$(ros2_in_container \
        "timeout --kill-after=2s 30s ros2 service call '${service_name}' '${service_type}' \"${request}\"")"; then
      echo "${output}"
      if grep -Eq "${expected}" <<<"${output}"; then
        return 0
      fi
    else
      echo "${output}"
    fi
    if ((attempt < max_attempts)); then
      echo "Retrying ${label} after a transient failure..." >&2
      sleep 3
    fi
  done

  echo "Failed, timed out, or received an unexpected response while saving ${label}." >&2
  exit 1
}

require_service /slam_toolbox/save_map
require_service /slam_toolbox/serialize_map
require_service /nvblox_node/save_map
require_service /nvblox_node/save_ply

call_and_check \
  "Nav2 occupancy map" \
  /slam_toolbox/save_map \
  slam_toolbox/srv/SaveMap \
  "{name: {data: '${map_base}'}}" \
  'result[=:][[:space:]]*0' \
  2

call_and_check \
  "SLAM pose graph" \
  /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '${map_base}'}" \
  'result[=:][[:space:]]*0'

call_and_check \
  "nvblox map" \
  /nvblox_node/save_map \
  nvblox_msgs/srv/FilePath \
  "{file_path: '${map_base}.nvblx'}" \
  'success[=:][[:space:]]*[Tt]rue'

call_and_check \
  "nvblox mesh" \
  /nvblox_node/save_ply \
  nvblox_msgs/srv/FilePath \
  "{file_path: '${map_base}.ply'}" \
  'success[=:][[:space:]]*[Tt]rue'

echo
echo "Map bundle saved under ${output_dir}:"
missing_files=0
for suffix in .pgm .yaml .posegraph .data .nvblx .ply; do
  if [[ -f "${output_dir}/${map_name}${suffix}" ]]; then
    stat -c '  %n (%s bytes)' "${output_dir}/${map_name}${suffix}"
  else
    echo "  WARNING: missing ${output_dir}/${map_name}${suffix}" >&2
    missing_files=$((missing_files + 1))
  fi
done

if ((missing_files > 0)); then
  echo "Map service calls succeeded, but ${missing_files} expected file(s) are missing." >&2
  exit 1
fi
