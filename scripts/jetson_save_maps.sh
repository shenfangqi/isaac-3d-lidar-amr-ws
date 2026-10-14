#!/usr/bin/env bash
# Save the real-robot map of the running mapping container (run on the Jetson).
#
# Real mapping is FAST-LIO2 odometry plus nvblox (mid360_mapping_real.launch,
# since 2026-09-26); no SLAM Toolbox runs, so the earlier pose-graph save is
# gone.  One bundle, all in the mapping session's odom frame (= the map frame
# of navigation):
#   NAME.nvblx  nvblox map           (/nvblox_node/save_map)
#   NAME.ply    nvblox surface mesh  (/nvblox_node/save_ply; the 3D localization)
#   NAME.pgm/.yaml  2D occupancy     (map_saver_cli from
#                   /nvblox_node/static_occupancy_grid, trinary,
#                   free_thresh 0.196 so unknown grey 205 stays unknown)
#
# The ROS CLI discovers this container's graph only partly per call
# (2026-10-14), so services are called directly with a timeout (the call
# waits for its server) and retried; nothing is listed first.  An existing
# map name is never overwritten.

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

if [[ "$(docker inspect -f '{{.State.Running}}' "${container_name}" 2>/dev/null || true)" != "true" \
    || "$(docker inspect -f '{{index .Config.Labels "carbot.mode"}}' "${container_name}" 2>/dev/null)" != "mapping" ]]; then
  echo "Container ${container_name} is not running in mapping mode. Start it first:" >&2
  echo "  ${workspace}/scripts/jetson_nvblox_container.sh recreate mapping" >&2
  exit 1
fi

for suffix in .nvblx .ply .pgm .yaml; do
  if [[ -e "${output_dir}/${map_name}${suffix}" ]]; then
    echo "Refusing to overwrite ${output_dir}/${map_name}${suffix}; choose another map name." >&2
    exit 2
  fi
done

mkdir -p "${output_dir}"
container_output_dir="${container_workspace}${output_dir#${workspace}}"
map_base="${container_output_dir}/${map_name}"

ros2_in_container() {
  docker exec "${container_name}" bash -lc \
    "source /opt/ros/humble/setup.bash && source ${container_workspace}/install/setup.bash && $*"
}

call_and_check() {
  local label="$1" service_name="$2" service_type="$3" request="$4" expected="$5"
  local attempt output
  for attempt in 1 2 3; do
    echo "Saving ${label} (attempt ${attempt}/3)..."
    if output="$(ros2_in_container \
        "timeout --kill-after=2s 60s ros2 service call '${service_name}' '${service_type}' \"${request}\"" 2>&1)" \
        && grep -Eq "${expected}" <<<"${output}"; then
      return 0
    fi
    tail -n 3 <<<"${output}" >&2
    sleep 3
  done
  echo "Failed, timed out, or received an unexpected response while saving ${label}." >&2
  exit 1
}

call_and_check "nvblox map" /nvblox_node/save_map nvblox_msgs/srv/FilePath \
  "{file_path: '${map_base}.nvblx'}" 'success[=:][[:space:]]*[Tt]rue'
call_and_check "nvblox mesh" /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
  "{file_path: '${map_base}.ply'}" 'success[=:][[:space:]]*[Tt]rue'

saved_2d=false
for attempt in 1 2 3; do
  echo "Saving 2D occupancy map (attempt ${attempt}/3)..."
  if ros2_in_container \
      "timeout --kill-after=2s 60s ros2 run nav2_map_server map_saver_cli -f '${map_base}' --ros-args -r map:=/nvblox_node/static_occupancy_grid -p map_subscribe_transient_local:=false -p save_map_timeout:=20.0 -p map_mode:=trinary -p image_format:=pgm -p free_thresh_default:=0.196 -p occupied_thresh_default:=0.65" \
      >/dev/null 2>&1 \
      && [[ -s "${output_dir}/${map_name}.pgm" && -s "${output_dir}/${map_name}.yaml" ]]; then
    saved_2d=true
    break
  fi
  sleep 3
done
if [[ "${saved_2d}" != "true" ]]; then
  echo "Failed to save the 2D occupancy map." >&2
  exit 1
fi

# The YAML must keep unknown space unknown (grey 205 is p = 0.196).
yaml="${output_dir}/${map_name}.yaml"
if ! grep -Eq '^mode:[[:space:]]*trinary' "${yaml}" \
    || ! grep -Eq '^free_thresh:[[:space:]]*0\.196' "${yaml}" \
    || ! grep -Eq '^occupied_thresh:[[:space:]]*0\.65' "${yaml}"; then
  echo "Unexpected thresholds in ${yaml}:" >&2
  cat "${yaml}" >&2
  exit 1
fi

echo
echo "Map bundle saved under ${output_dir}:"
missing=0
for suffix in .nvblx .ply .pgm .yaml; do
  file="${output_dir}/${map_name}${suffix}"
  if [[ -s "${file}" ]]; then
    stat -c '  %n (%s bytes)' "${file}"
  else
    echo "  MISSING or empty: ${file}" >&2
    missing=$((missing + 1))
  fi
done
if ((missing > 0)); then
  exit 1
fi
sed 's/^/  /' "${yaml}"
echo
echo "Navigate with: ./start_auto_nav.sh ${yaml}   (on the workstation)"
