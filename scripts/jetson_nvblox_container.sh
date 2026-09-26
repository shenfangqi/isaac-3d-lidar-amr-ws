#!/usr/bin/env bash

set -euo pipefail

container_name="carbot-nvblox"
image_name="carbot-isaac-ros-nvblox:3.2"
workspace="/home/shenfq/Projects/isaac_ros-dev"
dds_profile="${workspace}/src/isaac_ros_common/docker/middleware_profiles/rtps_udp_profile.xml"
time_gate="${workspace}/scripts/wait_for_mapping_time_sync.py"

usage() {
  echo "Usage: $0 {start|recreate|stop|status|logs} [compact|full|mapping]" >&2
  echo "       $0 {start|recreate} navigation-safe MAP_YAML" >&2
  echo "       $0 {start|recreate} navigation-diagnostic MAP_YAML" >&2
}

mode="${2:-compact}"
case "${mode}" in
  compact)
    launch_command="mid360_nvblox_real.launch.py pointcloud_topic:=/mid360/points_xyz"
    ;;
  full)
    launch_command="mid360_nvblox_real.launch.py pointcloud_topic:=/livox/lidar"
    ;;
  mapping)
    launch_command="mid360_mapping_real.launch.py"
    ;;
  navigation-safe|navigation-diagnostic)
    map_yaml="${3:-}"
    if [[ -z "${map_yaml}" ]]; then
      echo "${mode} requires an absolute Nav2 map YAML path." >&2
      usage
      exit 2
    fi
    map_yaml="$(realpath -m "${map_yaml}")"
    case "${map_yaml}" in
      "${workspace}/"*) ;;
      *)
        echo "Map YAML must be inside ${workspace}: ${map_yaml}" >&2
        exit 2
        ;;
    esac
    if [[ ! -f "${map_yaml}" ]]; then
      echo "Map YAML does not exist: ${map_yaml}" >&2
      exit 2
    fi
    container_map_yaml="/workspaces/isaac_ros-dev${map_yaml#${workspace}}"
    printf -v quoted_container_map '%q' "${container_map_yaml}"
    if [[ "${mode}" == "navigation-diagnostic" ]]; then
      launch_command="carbot_navigation_real.launch.py map:=${quoted_container_map} autostart:=false cmd_vel_output:=/cmd_vel_diagnostic"
    else
      launch_command="carbot_navigation_real.launch.py map:=${quoted_container_map} autostart:=false"
    fi
    ;;
  *) usage; exit 2 ;;
esac

run_container() {
  test -d "${workspace}/install/nvblox_ros"
  test -f "${dds_profile}"
  test -f "${time_gate}"

  docker run -d \
    --name "${container_name}" \
    --label "carbot.mode=${mode}" \
    --label "carbot.launch=${launch_command}" \
    --restart unless-stopped \
    --runtime nvidia \
    --network host \
    --ipc host \
    --pid host \
    --privileged \
    --user 1000:1000 \
    -e HOME=/tmp/carbot-nvblox-home \
    -e ROS_LOG_DIR=/tmp/carbot-nvblox-logs \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/rtps_udp_profile.xml \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "${dds_profile}:/etc/fastdds/rtps_udp_profile.xml:ro" \
    -v "${workspace}:/workspaces/isaac_ros-dev" \
    -v /run/systemd/timesync:/run/host-systemd-timesync:ro \
    -v /tmp:/tmp \
    --entrypoint /bin/bash \
    "${image_name}" -lc \
    "mkdir -p \"\${HOME}\" \"\${ROS_LOG_DIR}\"; python3 /workspaces/isaac_ros-dev/scripts/wait_for_mapping_time_sync.py; source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec ros2 launch isaac_3d_lidar_bringup ${launch_command}"
}

action="${1:-}"
case "${action}" in
  start)
    if docker container inspect "${container_name}" >/dev/null 2>&1; then
      existing_mode="$(docker inspect -f '{{index .Config.Labels "carbot.mode"}}' "${container_name}")"
      existing_launch="$(docker inspect -f '{{index .Config.Labels "carbot.launch"}}' "${container_name}")"
      if [[ "${existing_mode}" != "${mode}" || "${existing_launch}" != "${launch_command}" ]]; then
        echo "Existing ${container_name} was created for a different launch configuration." >&2
        echo "Use 'recreate' to change mode or map." >&2
        exit 2
      fi
      docker start "${container_name}" >/dev/null
    else
      run_container
    fi
    ;;
  recreate)
    if docker container inspect "${container_name}" >/dev/null 2>&1; then
      docker rm -f "${container_name}" >/dev/null
    fi
    run_container
    ;;
  stop)
    docker stop -t 10 "${container_name}"
    ;;
  status)
    docker ps -a --filter "name=^/${container_name}$" \
      --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'
    ;;
  logs)
    docker logs --tail 200 -f "${container_name}"
    ;;
  *)
    usage
    exit 2
    ;;
esac
