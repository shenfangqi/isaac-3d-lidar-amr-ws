#!/bin/bash
# Replay a stationary 3D-capture bag through the real localization chain:
# automatic_localization_manager + map_server + AMCL under
# lifecycle_manager_localization (started by the manager, as on the robot),
# with simulated time from the bag.  Only the navigation lifecycle is stubbed
# (validation_only).  See replay_localization_node_support.py for the replay
# aids (static TF from the bag, odom TF from /odom, stationary gap filling).
#
# Run inside ros2-dev-humble after building isaac_3d_lidar_bringup.  Uses
# ROS_DOMAIN_ID 77; run one replay at a time.
#
# usage: replay_localization_node_chain.sh BAG off|record|decide OUTDIR [START_OFFSET_S]
#   START_OFFSET_S  bag time to start playback (default 0); the manager's
#                   start service is called START_DELAY seconds later (env, 25).
#   EXTRA_PARAMS    extra manager parameter lines (env), e.g.
#                   "    surface_max_candidates: 24"
set -eo pipefail
if [[ $# -lt 3 ]]; then
  sed -n '2,18p' "$0" >&2
  exit 2
fi
BAG=$(realpath "$1"); POLICY=$2; OUT=$3; OFFSET=${4:-0}
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(dirname "${HERE}")
CONFIG=${WS}/src/isaac_3d_lidar_bringup/config/nav2
MAP_YAML=${MAP_YAML:-${WS}/calibration_data/2026-10-09_localization_diagnosis/real_map/carbot_map_20260928_215841.yaml}
MESH=${MESH:-${MAP_YAML%.*}.ply}

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file:///workspace/ros-humble/cyclonedds_ros_local.xml}
export ROS_DOMAIN_ID=77

rm -rf "${OUT}"; mkdir -p "${OUT}"; cd "${OUT}"
cat > params.yaml <<PARAMS
automatic_localization_manager:
  ros__parameters:
    use_sim_time: true
    localization_strategy: stationary_only
    validation_only: true
    surface_recheck_policy: ${POLICY}
    surface_mesh_path: ${MESH}
${EXTRA_PARAMS:-}
PARAMS

PATTERNS=("[a]utomatic_localization_manager" "[r]eplay_localization_node_support"
          "[m]ap_server" "[l]ifecycle_manager_localization" "[n]av2_amcl/amcl" "[r]os2 bag play")
cleanup() {
  for p in "${PATTERNS[@]}"; do pkill -INT -f "$p" || true; done
  sleep 2
  for p in "${PATTERNS[@]}"; do pkill -9 -f "$p" || true; done
}
trap cleanup EXIT

ros2 run nav2_map_server map_server --ros-args --params-file "${CONFIG}/carbot_amcl_real.yaml" \
  -p use_sim_time:=true -p yaml_filename:="${MAP_YAML}" > map_server.log 2>&1 &
ros2 run nav2_amcl amcl --ros-args --params-file "${CONFIG}/carbot_amcl_real.yaml" \
  -p use_sim_time:=true -r initialpose:=/amcl_initialpose > amcl.log 2>&1 &
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -r __node:=lifecycle_manager_localization -p use_sim_time:=true -p autostart:=false \
  -p node_names:="['map_server','amcl']" > lifecycle.log 2>&1 &
python3 "${HERE}/replay_localization_node_support.py" "${BAG}" "${OUT}/status.jsonl" \
  > support.log 2>&1 &
ros2 run isaac_3d_lidar_bringup automatic_localization_manager --ros-args \
  --params-file "${CONFIG}/carbot_auto_localization_real.yaml" \
  --params-file "${OUT}/params.yaml" > manager.log 2>&1 &
sleep 4
ros2 bag play "${BAG}" --clock 100 --start-offset "${OFFSET}" \
  --topics /odom /fast_lio/imu_odom /fast_lio/cloud_registered_body /scan_localization /scan \
  --remap /odom:=/rec/odom /fast_lio/imu_odom:=/rec/imu_odom > play.log 2>&1 &
PLAY=$!
sleep "${START_DELAY:-25}"
timeout 30 ros2 service call /automatic_localization/start std_srvs/srv/Trigger > start.log 2>&1
for _ in $(seq 1 900); do
  kill -0 "${PLAY}" 2>/dev/null || break
  if grep -qE "STATE (WAIT_MANUAL_POSE|FAULT_STOPPED)" support.log; then sleep 3; break; fi
  if grep -qE "STATE CANDIDATE_READY" support.log; then sleep 15; break; fi
  sleep 1
done
grep -E "STATE " support.log | sed 's/^.*STATE /STATE /' | cut -c1-240
grep -E "3D decision" manager.log | sed 's/^.*\]: //' || true
