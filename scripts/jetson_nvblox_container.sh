#!/usr/bin/env bash

set -euo pipefail

container_name="carbot-nvblox"
image_name="carbot-isaac-ros-nvblox:3.3-lio"
workspace="/home/shenfq/Projects/isaac_ros-dev"
livox_workspace="/home/shenfq/Projects/lidar-mid360/ws_livox"
dds_profile="${workspace}/src/isaac_ros_common/docker/middleware_profiles/rtps_udp_profile.xml"
time_gate="${workspace}/scripts/wait_for_mapping_time_sync.py"

usage() {
  echo "Usage: $0 {start|recreate|stop|status|logs} [compact|full|mapping]" >&2
  echo "       $0 {start|recreate} navigation-safe MAP_YAML [manual|auto|auto-activate] [none|complex-route-validation] [legacy_full_rotation|stationary_only|segmented_rotation] [forbid|guarded PROFILE_JSON EXTRINSICS_HASH CONTROL_CHAIN_HASH true|false]" >&2
  echo "       $0 {start|recreate} navigation-diagnostic MAP_YAML" >&2
  echo "       Append 'auto' for validation or 'auto-activate' for guarded Nav2 activation." >&2
}

mode="${2:-compact}"
case "${mode}" in
  compact)
    launch_command="mid360_nvblox_real.launch.py pointcloud_topic:=/fast_lio/cloud_registered_body"
    ;;
  full)
    launch_command="mid360_nvblox_real.launch.py pointcloud_topic:=/fast_lio/cloud_registered_body"
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
    initialization="${4:-manual}"
    if [[ "${initialization}" != "manual" && "${initialization}" != "auto" && "${initialization}" != "auto-activate" ]]; then
      echo "Initialization must be 'manual', 'auto', or 'auto-activate'." >&2
      exit 2
    fi
    if [[ "${initialization}" != "manual" && "${mode}" != "navigation-safe" ]]; then
      echo "Automatic physical localization requires navigation-safe mode." >&2
      exit 2
    fi
    diagnostic_mode="${5:-none}"
    if [[ "${diagnostic_mode}" != "none" && "${diagnostic_mode}" != "complex-route-validation" ]]; then
      echo "Diagnostic mode must be 'none' or 'complex-route-validation'." >&2
      exit 2
    fi
    if [[ "${diagnostic_mode}" != "none" && "${mode}" != "navigation-safe" ]]; then
      echo "Complex-route validation requires navigation-safe mode." >&2
      exit 2
    fi
    localization_strategy="${6:-legacy_full_rotation}"
    case "${localization_strategy}" in
      legacy_full_rotation|stationary_only|segmented_rotation) ;;
      *)
        echo "Localization strategy must be legacy_full_rotation, stationary_only or segmented_rotation." >&2
        exit 2
        ;;
    esac
    motion_policy="${7:-forbid}"
    motion_profile="${8:-}"
    extrinsics_hash="${9:-}"
    control_chain_hash="${10:-}"
    operator_rotation_clear="${11:-false}"
    if [[ "${motion_policy}" != "forbid" && "${motion_policy}" != "guarded" ]]; then
      echo "Motion policy must be 'forbid' or 'guarded'." >&2
      exit 2
    fi
    if [[ "${operator_rotation_clear}" != "true" && "${operator_rotation_clear}" != "false" ]]; then
      echo "operator_rotation_clear must be 'true' or 'false'." >&2
      exit 2
    fi
    motion_launch_argument=""
    if [[ "${motion_policy}" == "guarded" ]]; then
      # The workstation already checked ACCEPTED status and hashes; the
      # manager and guard re-check them, so a mismatch never moves.
      if [[ "${localization_strategy}" != "segmented_rotation" ]]; then
        echo "motion_policy guarded is valid only for segmented_rotation." >&2
        exit 2
      fi
      motion_profile="$(realpath -m "${motion_profile}")"
      case "${motion_profile}" in
        "${workspace}/"*) ;;
        *)
          echo "Motion profile must be inside ${workspace}: ${motion_profile}" >&2
          exit 2
          ;;
      esac
      if [[ ! -f "${motion_profile}" ]]; then
        echo "Motion profile does not exist: ${motion_profile}" >&2
        exit 2
      fi
      for value in "${extrinsics_hash}" "${control_chain_hash}"; do
        if [[ ! "${value}" =~ ^[0-9a-f]{64}$ ]]; then
          echo "Guarded rotation needs 64-hex extrinsics and control-chain hashes." >&2
          exit 2
        fi
      done
      container_profile="/workspaces/isaac_ros-dev${motion_profile#${workspace}}"
      printf -v quoted_container_profile '%q' "${container_profile}"
      motion_launch_argument=" motion_policy:=guarded motion_profile_path:=${quoted_container_profile} extrinsics_hash:=${extrinsics_hash} control_chain_hash:=${control_chain_hash} operator_rotation_clear:=${operator_rotation_clear}"
    elif [[ "${operator_rotation_clear}" == "true" ]]; then
      echo "operator_rotation_clear requires motion_policy guarded." >&2
      exit 2
    fi
    if [[ "${localization_strategy}" != "legacy_full_rotation" && "${initialization}" == "manual" ]]; then
      echo "A localization strategy requires automatic initialization." >&2
      exit 2
    fi
    auto_launch_argument=""
    if [[ "${initialization}" == "auto" ]]; then
      auto_launch_argument=" automatic_localization:=true"
    elif [[ "${initialization}" == "auto-activate" ]]; then
      auto_launch_argument=" automatic_localization:=true auto_localization_validation_only:=false"
    fi
    if [[ "${initialization}" != "manual" ]]; then
      auto_launch_argument+=" localization_strategy:=${localization_strategy}${motion_launch_argument}"
    fi
    diagnostic_launch_argument=""
    if [[ "${diagnostic_mode}" == "complex-route-validation" ]]; then
      diagnostic_launch_argument=" complex_route_validation:=true"
    fi
    if [[ "${mode}" == "navigation-diagnostic" ]]; then
      launch_command="carbot_navigation_real.launch.py map:=${quoted_container_map} autostart:=false cmd_vel_output:=/cmd_vel_diagnostic${auto_launch_argument}"
    else
      launch_command="carbot_navigation_real.launch.py map:=${quoted_container_map} autostart:=false${auto_launch_argument}${diagnostic_launch_argument}"
    fi
    ;;
  *) usage; exit 2 ;;
esac

run_container() {
  test -d "${workspace}/install/nvblox_ros"
  test -f "${livox_workspace}/install/setup.bash"
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
    -v "${livox_workspace}:${livox_workspace}:ro" \
    -v /run/systemd/timesync:/run/host-systemd-timesync:ro \
    -v /tmp:/tmp \
    --entrypoint /bin/bash \
    "${image_name}" -lc \
    "mkdir -p \"\${HOME}\" \"\${ROS_LOG_DIR}\"; python3 /workspaces/isaac_ros-dev/scripts/wait_for_mapping_time_sync.py; source /opt/ros/humble/setup.bash; source ${livox_workspace}/install/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; exec ros2 launch isaac_3d_lidar_bringup ${launch_command}"
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
