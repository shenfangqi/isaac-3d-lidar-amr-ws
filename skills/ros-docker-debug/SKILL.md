---
name: ros-docker-debug
description: Operate and diagnose the ROS 2 Humble, Isaac ROS nvblox, and Isaac Sim Docker containers for this workspace. Use for container access, CycloneDDS setup, project launch or shutdown, and live ROS topic, TF, lifecycle, Action, costmap, or nvblox diagnosis.
---

# ROS Docker Debug

Work from `/home/shenfq/projects/ros-humble`.

Use noninteractive `docker exec ... bash -lc` for automated checks. Use an interactive shell only when the user needs to operate a terminal. Never persist a sudo password in project files.

## Routing

- Read [references/container-environments.md](references/container-environments.md) when entering containers, sourcing ROS, or diagnosing DDS visibility.
- Read [references/runtime-operations.md](references/runtime-operations.md) when starting, stopping, rebuilding, loading, or saving project components.
- Read [references/diagnostics.md](references/diagnostics.md) for nodes, topics, TF, lifecycle, Actions, costmaps, QoS, duplicate processes, or logs.
- Also use the sibling `isaac-amr-project-state` skill for authoritative project state, validated map parameters, localization choices, and historical evidence.
- For any nvblox map build, gap-closing, save, export, or validation task, read `../../docs/map_gen/README.md` completely.

## Core operating rules

- Dedicated containers are `ros2-dev-humble`, `isaac-ros-nvblox`, and `isaac-sim`. Inspect their state before assuming any component is running.
- Use `rmw_cyclonedds_cpp` with `file:///workspace/ros-humble/cyclonedds_ros_local.xml` in every ROS environment.
- Run nvblox container commands as user `admin` and source Isaac ROS before the project overlay.
- Do not start duplicate nvblox or project launch processes. Require exactly one `/nvblox_node`, `/nvblox_container`, and `/pointcloud_padder`.
- Do not restart Isaac Sim or nvblox unless diagnosis requires it; state which process must stop.
- Use `--no-daemon` for supported Humble ROS CLI discovery when the shared daemon is stale. Detect Actions through hidden `ACTION/_action/send_goal` services when needed.
- After any velocity test, always publish a zero Twist. Do not repeatedly click RViz Goals; wait for the current Action to terminate.
- Preserve user changes, map versions, and diagnostic evidence. Never load an old `.nvblx` while creating a new map.
