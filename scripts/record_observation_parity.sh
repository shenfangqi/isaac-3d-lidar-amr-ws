#!/bin/bash
# 31-minute stationary observation capture for the observation_parity gate.
#
# Records on the Jetson (systemd-run, so it survives an SSH drop) with
# best-effort QoS on every topic: a reliable recorder subscription stalled
# FAST-LIO twice (2026-10-13).  No point clouds, and never /cmd_vel_command
# (preflight requires the compensator as its only subscriber).  The robot must
# stay still and powered; nothing here starts a stack, publishes or moves.
#
# usage: record_observation_parity.sh start|status|fetch NAME
#   start   preflight (publishers, Wi-Fi power save), then record 1860 s to
#           calibration_data/<date>_observation_parity_NAME on the Jetson
#   status  show the recorder unit and the bag size
#   fetch   copy the finished bag to the local calibration_data/ and run
#           scripts/analyze_observation_parity.py in ros2-dev-humble
# environment:
#   CARBOT_JETSON_SSH   SSH target (default isaac-jetson)
#   DURATION_S          recording length (default 1860)
set -o pipefail
ACTION=${1:?usage: $0 start|status|fetch NAME}
NAME=${2:?usage: $0 start|status|fetch NAME}
HERE=$(cd "$(dirname "$0")" && pwd)
WORKSPACE=$(cd "${HERE}/.." && pwd)
JETSON=${CARBOT_JETSON_SSH:-isaac-jetson}
J="ssh -o BatchMode=yes ${JETSON}"
DURATION=${DURATION_S:-1860}
UNIT=observation-parity-bag
REMOTE_ROOT=/home/shenfq/Projects/isaac_ros-dev/calibration_data
DATE_FILE=/tmp/observation_parity_${NAME}.date
ROS_ENV="source /opt/ros/humble/setup.bash; source /home/shenfq/Projects/carbot-ros2/install/local_setup.bash"
REQUIRED=(/wheel_ticks /carbot/status /mid360/imu/data_raw)
# Recorded only when published (FAST-LIO runs with the navigation stack).
OPTIONAL=(/odom /fast_lio/imu_odom)

remote_bag() {
  local date
  date=$(${J} "cat ${DATE_FILE} 2>/dev/null") || return 1
  [[ -n "${date}" ]] || return 1
  echo "${REMOTE_ROOT}/${date}_observation_parity_${NAME}"
}

case "${ACTION}" in
start)
  ${J} "systemctl --user is-active --quiet ${UNIT}" \
    && { echo "${UNIT} is already running on the Jetson" >&2; exit 1; }
  date=$(date +%F)
  bag=${REMOTE_ROOT}/${date}_observation_parity_${NAME}
  ${J} "test ! -e ${bag}" || { echo "${bag} exists or the Jetson is unreachable" >&2; exit 1; }

  topics=()
  for topic in "${REQUIRED[@]}" "${OPTIONAL[@]}"; do
    count=$(${J} "bash -lc '${ROS_ENV}; export ROS_DOMAIN_ID=0; timeout 15 ros2 topic info ${topic} 2>/dev/null'" \
      | awk '/Publisher count:/ {print $3}')
    if [[ "${count:-0}" -ge 1 ]]; then
      topics+=("${topic}")
      echo "OK      ${topic}: ${count} publisher(s)"
    elif [[ " ${REQUIRED[*]} " == *" ${topic} "* ]]; then
      echo "MISSING ${topic}: no publisher; is the chassis/MID-360 service up?" >&2
      exit 1
    else
      echo "skip    ${topic}: not published"
    fi
  done
  # Wi-Fi power save delays the ESP32 traffic in bursts; it must be off.
  ${J} 'for dev in $(iw dev 2>/dev/null | awk "/Interface/ {print \$2}"); do echo "wifi ${dev}: $(iw dev ${dev} get power_save 2>/dev/null)"; done'

  overrides=/tmp/observation_parity_qos.yaml
  qos=""
  for topic in "${topics[@]}"; do
    qos+="${topic}:\n  reliability: best_effort\n  history: keep_last\n  depth: 200\n"
  done
  ${J} "printf '${qos}' > ${overrides} && echo ${date} > ${DATE_FILE}" || exit 1
  ${J} "systemctl --user reset-failed ${UNIT} 2>/dev/null; systemd-run --user --unit=${UNIT} \
    -p StandardOutput=file:/tmp/${UNIT}_${NAME}.log -p StandardError=file:/tmp/${UNIT}_${NAME}.log \
    --setenv=ROS_DOMAIN_ID=0 bash -lc \"${ROS_ENV}; exec timeout -s INT ${DURATION} ros2 bag record \
    --qos-profile-overrides-path ${overrides} -o ${bag} ${topics[*]}\" >/dev/null" || exit 1
  echo "recording ${bag} for ${DURATION} s: keep the robot still and powered."
  echo "check with: $0 status ${NAME}; afterwards: $0 fetch ${NAME}"
  ;;
status)
  bag=$(remote_bag) || { echo "no recording named ${NAME} was started" >&2; exit 1; }
  ${J} "systemctl --user is-active ${UNIT}; du -sh ${bag} 2>/dev/null; tail -n 3 /tmp/${UNIT}_${NAME}.log 2>/dev/null"
  ;;
fetch)
  bag=$(remote_bag) || { echo "no recording named ${NAME} was started" >&2; exit 1; }
  ${J} "systemctl --user is-active --quiet ${UNIT}" \
    && { echo "still recording; never read a bag while it records" >&2; exit 1; }
  ${J} "test -f ${bag}/metadata.yaml" || { echo "${bag} has no metadata.yaml" >&2; exit 1; }
  local_bag=${WORKSPACE}/calibration_data/$(basename "${bag}")
  mkdir -p "${local_bag}"
  rsync -a "${JETSON}:${bag}/" "${local_bag}/" || exit 1
  container_root=/workspace/ros-humble/isaac_3d_lidar_amr_ws
  container_bag=${container_root}/calibration_data/$(basename "${bag}")
  docker exec ros2-dev-humble bash -c "source /opt/ros/humble/setup.bash; \
    source ${container_root}/install/setup.bash; cd ${container_root} && \
    python3 scripts/analyze_observation_parity.py ${container_bag} \
    --output ${container_bag}/observation_parity_analysis"
  ;;
*)
  echo "unknown action ${ACTION}; use start, status or fetch" >&2
  exit 2
  ;;
esac
