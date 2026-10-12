#!/usr/bin/env bash
# Open the RViz variable-region editor on the workstation display.
#
# Shows the real map, the regions in use (orange) and two corner handles;
# right-click a handle to save the box.  Saved boxes go to
# calibration_data/region_editor/new_regions.yaml, then:
#   python3 scripts/check_variable_regions.py quick <candidate.yaml> --only <name>
# Runs in its own container on ROS domain 78, localhost only, with the
# workspace read-only: it never talks to the robot.  Close with
#   docker rm -f carbot-region-editor

set -Eeuo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(cd -- "${script_dir}/.." && pwd)"
inside=/workspace/ros-humble/isaac_3d_lidar_amr_ws
map_dir="${CARBOT_REGION_MAP_DIR:-calibration_data/2026-10-09_localization_diagnosis/real_map}"
map_stem="${CARBOT_REGION_MAP_STEM:-carbot_map_20260928_215841}"
existing="src/isaac_3d_lidar_bringup/config/localization/${map_stem}_variable_regions.yaml"
image="${CARBOT_REGION_EDITOR_IMAGE:-ros2-dev-humble-backup:latest}"
output_dir="${workspace}/calibration_data/region_editor"

for path in "${map_dir}/${map_stem}.yaml" "${map_dir}/${map_stem}.ply"; do
  [[ -f "${workspace}/${path}" ]] || { echo "missing ${path}" >&2; exit 1; }
done
existing_arg=()
[[ -f "${workspace}/${existing}" ]] && existing_arg=(--existing "${inside}/${existing}")
mkdir -p "${output_dir}"
docker rm -f carbot-region-editor >/dev/null 2>&1 || true
rm -f "${output_dir}/editor.log" 2>/dev/null || true

docker run -d --rm --name carbot-region-editor --init --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:1}" -e XAUTHORITY=/tmp/.carbot.Xauthority \
  -e ROS_DOMAIN_ID=78 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e XDG_RUNTIME_DIR=/tmp/runtime-editor \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}:/tmp/.carbot.Xauthority:ro" \
  -v "${workspace}:${inside}:ro" -v "${output_dir}:/region_editor" \
  "${image}" bash -lc "
    source /opt/ros/humble/setup.bash
    ros2 run nav2_map_server map_server --ros-args \
      -p yaml_filename:=${inside}/${map_dir}/${map_stem}.yaml > /region_editor/map_server.log 2>&1 &
    ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -r __node:=region_editor_lm \
      -p autostart:=true -p node_names:=\"['map_server']\" > /region_editor/lifecycle.log 2>&1 &
    python3 ${inside}/scripts/region_editor/editor.py --mesh ${inside}/${map_dir}/${map_stem}.ply \
      ${existing_arg[*]} --output /region_editor/new_regions.yaml > /region_editor/editor.log 2>&1 &
    install -d -m 700 \"\${XDG_RUNTIME_DIR}\"
    exec rviz2 -d ${inside}/scripts/region_editor/region_editor.rviz" >/dev/null

for _ in $(seq 30); do
  grep -q 'ready' "${output_dir}/editor.log" 2>/dev/null && break
  sleep 2
done
tail -1 "${output_dir}/editor.log"
