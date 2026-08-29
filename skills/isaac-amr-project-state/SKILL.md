---
name: isaac-amr-project-state
description: Resume and maintain the Isaac Sim 3D-LiDAR AMR project, including warehouse_v3 navigation, AMCL or ground-truth localization, nvblox mapping decisions, and regression context. Use for work on isaac_3d_lidar_amr_ws or the repository-level navigation launchers.
---

# Isaac AMR Project State

Work from `/home/shenfq/projects/ros-humble`.

## Authority and routing

- For any resume, startup, shutdown, navigation change, or current-state question, read [references/current-state.md](references/current-state.md). Its newest checkpoint is authoritative; use the cold-start baseline only when no live process survives.
- For regression comparison, prior failures, or explaining why a parameter exists, read [references/validation-history.md](references/validation-history.md).
- For building, resuming, closing gaps in, saving, exporting, or validating an nvblox map, read `../../docs/map_gen/README.md` completely before acting.
- For Docker commands, DDS setup, live ROS diagnosis, and component launch details, use the sibling `ros-docker-debug` skill and read the reference it routes to.
- `../../docs/map_nav/README.md` is the user-facing navigation tutorial; consult it when changing documented behavior or instructions.

Do not treat statements such as “remains running” in historical evidence as live state. Inspect the system or use the authoritative shutdown resume point.

## Project invariants

- The project map chain is `3D LiDAR -> padded spherical cloud -> nvblox TSDF/ESDF -> static_occupancy_grid -> Nav2`. Do not substitute SLAM Toolbox for this map.
- `warehouse_v3` is the saved and visually validated map. Preserve v1/v2 and never load them while rebuilding v3.
- The current Isaac RTX cloud must pass through `/pointcloud_padder` as `1800 x 31`; nvblox must not consume the raw variable-length cloud directly.
- Mapping requires exactly one `/pointcloud_padder`, `/nvblox_node`, and `/nvblox_container`, `use_sim_time=true` from process startup, and a live `lidar_min_valid_range_m=0.5` check.
- Saved-map navigation uses RViz simulation time and Fixed Frame `map`. Use RViz `2D Goal Pose` for Nav2; `Publish Point` only publishes `/clicked_point` unless a separate bridge subscribes to it.
- Permanent Nav2 geometry is `robot_radius=0.35 m` and `inflation_radius=0.45 m`. The `0.80 m` obstacle and `0.60 m` unknown clearances were regression target-selection filters, not persisted navigation limits.
- Frontier Explorer defaults are `0.55 m` obstacle, `0.40 m` unknown, and `0.60 m` boundary clearance. Its `min_goal_distance_m=0.80` measures robot-to-candidate distance.
- Manipulation and docking need task-aware transit, pre-grasp, final-approach, and docking behavior. Do not impose the regression filters on close final approaches.

## Evidence discipline

- Verify live topics, parameters, lifecycle states, TF, Actions, and process counts instead of relying on an old checkpoint.
- Before changing robot radius or inflation, identify whether the static map or live obstacle layer produces the cost.
- Do not infer map quality from a dense ESDF cloud alone; inspect the 2D OccupancyGrid and exported PGM.
- Keep temporary diagnostics out of the repository and preserve user changes and map versions.
