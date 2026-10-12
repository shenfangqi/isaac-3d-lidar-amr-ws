#!/bin/bash
# One stationary record-mode localization sample for ground-truth labelling.
#
# Localizes WITHOUT recording (a bag on the Jetson during localization delayed
# /odom by up to 2 s, 2026-10-10), shows the 3D leader and the 2D candidate as
# RViz arrows for the operator, then records 90 s of the still scene.  Process
# that capture with
#   scripts/diagnose_localization_bag.py BAG --stationary-capture ...
# The robot must not move until the post-result bag has finished.  Nav2 stays
# inactive (validation only); this script never publishes a pose or a velocity.
#
# usage: record_localization_sample.sh NAME [OUTDIR]
#   NAME    bag suffix: calibration_data/<date>_issue13_sample_NAME on the Jetson
#   OUTDIR  local directory for the launcher log and status (default /tmp)
set -o pipefail
NAME=${1:?usage: $0 NAME [OUTDIR]}
OUT=${2:-/tmp}
HERE=$(cd "$(dirname "$0")" && pwd)
JETSON=${CARBOT_JETSON_SSH:-isaac-jetson}
J="ssh -o BatchMode=yes ${JETSON}"
BAG=/home/shenfq/Projects/isaac_ros-dev/calibration_data/$(date +%F)_issue13_sample_${NAME}
LOG=${OUT}/sample_${NAME}_launch.log

"${HERE}/stop_real_robot_navigation_rviz.sh" >/dev/null 2>&1
${J} "systemctl --user stop issue13-pose-markers issue13-sample-bag 2>/dev/null; test ! -e ${BAG}" \
  || { echo "bag ${BAG} exists or the Jetson is unreachable" >&2; exit 1; }
scp -q "${HERE}/publish_pose_markers.py" "${JETSON}:/tmp/publish_pose_markers.py" || exit 1

CARBOT_INITIAL_POSE_TIMEOUT=900 "${HERE}/start_real_robot_navigation_rviz.sh" --automatic \
  --localization-strategy stationary_only --surface-recheck record \
  > "${LOG}" 2>&1 </dev/null &
for _ in $(seq 1 90); do
  grep -qE "STOPPED: click|Startup failed|VALIDATION_ONLY" "${LOG}" && break
  sleep 5
done
if grep -q "Startup failed" "${LOG}"; then
  echo "startup failed; see ${LOG}" >&2
  exit 1
fi
sleep 3
${J} "docker exec carbot-nvblox bash -lc 'source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; timeout 10 ros2 topic echo --once --full-length /automatic_localization/status std_msgs/msg/String'" \
  2>/dev/null | grep '^data:' > "${OUT}/sample_${NAME}_status.yaml"
ARROWS=$(python3 - "${OUT}/sample_${NAME}_status.yaml" <<'PY'
import json, math, sys
import yaml
try:
    status = json.loads(yaml.safe_load(open(sys.argv[1]).read())['data'])
except Exception as error:
    print(f'status unreadable: {error}', file=sys.stderr)
    sys.exit()
surface = status.get('surface_recheck') or {}
print(json.dumps({k: status.get(k) for k in ('state', 'failure_reason', 'hypothesis_count')}
                 | {'surface': {k: surface.get(k) for k in ('reason', 'composite_gaps',
                                                           'leader_pose_at_reference')}},
                 ensure_ascii=False), file=sys.stderr)
arrows = []
leader = surface.get('leader_pose_at_reference')
if leader:
    arrows.append(f"{leader['x']},{leader['y']},{math.degrees(leader['yaw']):.1f},0,1,0,3D_leader")
pose = status.get('candidate_anchor_pose') or status.get('candidate_pose')
if isinstance(pose, dict):
    pose = [pose['x'], pose['y'], pose['yaw']]
if pose:
    arrows.append(f'{pose[0]:.3f},{pose[1]:.3f},{math.degrees(pose[2]):.1f},0,0.4,1,2D_candidate')
print(' '.join(arrows))
PY
)
if [[ -n "${ARROWS}" ]]; then
  ${J} "systemctl --user reset-failed issue13-pose-markers 2>/dev/null; systemd-run --user --unit=issue13-pose-markers --setenv=ROS_DOMAIN_ID=0 bash -lc 'source /opt/ros/humble/setup.bash; exec python3 /tmp/publish_pose_markers.py ${ARROWS}' >/dev/null"
  echo "RViz arrows: ${ARROWS}  (green 3D leader, blue 2D candidate; arrow root = robot centre)"
else
  echo "no candidate pose to show"
fi
${J} "systemctl --user reset-failed issue13-sample-bag 2>/dev/null; systemd-run --user --unit=issue13-sample-bag -p StandardOutput=file:/tmp/sample_${NAME}_bag.log -p StandardError=file:/tmp/sample_${NAME}_bag.log --setenv=ROS_DOMAIN_ID=0 bash -lc \"source /opt/ros/humble/setup.bash; source /home/shenfq/Projects/carbot-ros2/install/local_setup.bash; exec timeout -s INT 90 ros2 bag record -o ${BAG} /fast_lio/cloud_registered_body /fast_lio/imu_odom /odom /tf /tf_static /scan /scan_localization /map /automatic_localization/status /amcl_pose /wheel_ticks /cmd_vel\" >/dev/null" \
  && echo "recording ${BAG} for 90 s: keep the robot still"
