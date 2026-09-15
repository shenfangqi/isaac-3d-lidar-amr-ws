---
name: isaac-amr-project-state
description: Resume and maintain the Isaac Sim 3D-LiDAR AMR project, including warehouse_v3 navigation, AMCL or ground-truth localization, nvblox mapping decisions, and regression context. Use for work on isaac_3d_lidar_amr_ws or the repository-level navigation launchers.
---

# Isaac AMR Project State

Work from `/home/shenfq/projects/ros-humble`.

## Authority and routing

- For any resume, startup, shutdown, navigation change, or current-state question, read [references/current-state.md](references/current-state.md). Its newest checkpoint is authoritative; use the cold-start baseline only when no live process survives.
- For any connection, inspection, deployment, configuration, build, launch, or diagnosis on the physical Jetson, read [references/jetson-target.md](references/jetson-target.md) first. Use its SSH alias and re-verify live state before mutation.
- For regression comparison, prior failures, or explaining why a parameter exists, read [references/validation-history.md](references/validation-history.md).
- For building, resuming, closing gaps in, saving, exporting, or validating an nvblox map, read `../../docs/map_gen/README.md` completely before acting.
- For Docker commands, DDS setup, live ROS diagnosis, and component launch details, use the sibling `ros-docker-debug` skill and read the reference it routes to.
- `../../docs/map_nav/README.md` is the user-facing navigation tutorial; consult it when changing documented behavior or instructions.

Do not treat statements such as “remains running” in historical evidence as live state. Inspect the system or use the authoritative shutdown resume point.

## Project invariants

- The project map chain is `3D LiDAR -> padded spherical cloud -> nvblox TSDF/ESDF -> static_occupancy_grid -> Nav2`. Do not substitute SLAM Toolbox for this map.
- `warehouse_v3` is the saved and visually validated map. Preserve v1/v2 and never load them while rebuilding v3.
- The Mid-360 Isaac RTX cloud must pass through `/pointcloud_padder` as `1000 x 40`; nvblox must not consume the raw variable-length `/livox/lidar` cloud directly.
- Mapping requires exactly one `/pointcloud_padder`, `/nvblox_node`, and `/nvblox_container`, `use_sim_time=true` from process startup, and a live `lidar_min_valid_range_m=0.5` check.
- Saved-map navigation uses RViz simulation time and Fixed Frame `map`. Use RViz `2D Goal Pose` for Nav2; `Publish Point` only publishes `/clicked_point` unless a separate bridge subscribes to it.
- Carbot Nav2 geometry is the measured polygon `[[0.155, 0.133], [0.155, -0.133], [-0.130, -0.133], [-0.130, 0.133]]`; `inflation_radius=0.45 m` remains the conservative baseline. The `0.80 m` obstacle and `0.60 m` unknown clearances were historical Carter regression target-selection filters, not persisted navigation limits.
- Frontier Explorer defaults are `0.55 m` obstacle, `0.40 m` unknown, and `0.60 m` boundary clearance. Its `min_goal_distance_m=0.80` measures robot-to-candidate distance.
- Manipulation and docking need task-aware transit, pre-grasp, final-approach, and docking behavior. Do not impose the regression filters on close final approaches.

## Evidence discipline

- Verify live topics, parameters, lifecycle states, TF, Actions, and process counts instead of relying on an old checkpoint.
- Before changing robot radius or inflation, identify whether the static map or live obstacle layer produces the cost.
- Do not infer map quality from a dense ESDF cloud alone; inspect the 2D OccupancyGrid and exported PGM.
- Keep temporary diagnostics out of the repository and preserve user changes and map versions.
