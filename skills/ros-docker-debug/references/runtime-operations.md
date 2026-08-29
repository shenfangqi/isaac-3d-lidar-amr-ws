# Runtime operations

Read the sibling project-state skill's [current state](../../isaac-amr-project-state/references/current-state.md) before resuming or changing the saved-map stack.

## Validated saved-map navigation

From the repository root:

```bash
./start_nav_all.sh
```

This starts the three containers, Isaac Sim headlessly, warehouse_v3 nvblox, Navigation, and RViz through detached processes. Require `[ OK ] All startup health checks passed.` Logs are under `logs/start_nav_all/` and the previous run is retained as `.previous`.

Recheck a running stack without changing it:

```bash
./start_nav_all.sh --health-check
```

Stop the dedicated project containers cleanly and idempotently:

```bash
./stop_nav_all.sh
```

RViz navigation requires `use_sim_time=true` and Fixed Frame `map`. Use `2D Goal Pose`; `Publish Point` needs a separate `/clicked_point` subscriber bridge.

## Component launch commands

Pure blank nvblox mapping, without loading an old map:

```bash
ros2 launch isaac_3d_lidar_bringup xt32_nvblox.launch.py
```

This starts `/pointcloud_padder`, publishes `/front_3d_lidar/lidar_points_nvblox` as `1800 x 31`, and starts nvblox with simulation time. Do not bypass the padder for the current Isaac RTX cloud.

Load the project map selected by the wrapper:

```bash
ros2 launch /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nvblox_with_map.launch.py
```

Start simulation ground-truth Nav2:

```bash
ros2 launch /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nav_stack.launch.py \
  localization_mode:=ground_truth
```

Start AMCL for the odom-aligned Isaac simulation:

```bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/local_setup.bash
ros2 launch /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nav_stack.launch.py \
  localization_mode:=amcl \
  amcl_initial_pose_mode:=odom_identity
```

For a real robot, use `manual` plus RViz `2D Pose Estimate`, or `fixed` with surveyed x/y/yaw. Never use `odom_identity` merely for convenience on hardware.

## Rebuild bringup

After changing the padder, launch file, or LiDAR model, run in the nvblox container:

```bash
cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
source /workspaces/isaac_ros-dev/install/setup.bash
colcon build --packages-select isaac_3d_lidar_bringup --symlink-install
```

## Map save commands

Follow [the map-generation tutorial](../../../docs/map_gen/README.md) completely and verify the target version does not overwrite a protected map.

In the nvblox container:

```bash
ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath \
  "{file_path: /workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/nvblox/warehouse_v3.nvblx}"

ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
  "{file_path: /workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/nvblox/warehouse_v3.ply}"
```

In the Humble container, export the Volatile occupancy topic:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/2d/warehouse_v3 \
  --ros-args -p map_subscribe_transient_local:=false \
  -r map:=/nvblox_node/static_occupancy_grid
```
