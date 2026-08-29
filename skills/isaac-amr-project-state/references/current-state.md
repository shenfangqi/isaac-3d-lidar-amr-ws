# Current authoritative project state

## Live checkpoint: 2026-08-29

This is the newest authoritative state after validating and stopping all three launch modes.

- `ISAAC_WEBRTC=1 ./start_nav_all.sh` completed with `[ OK ] All startup health checks passed.` The same Isaac instance provided WebRTC, loaded and played the warehouse automatically, and supplied the full nvblox/Nav2/RViz stack.
- The AppImage connected to `127.0.0.1`; the native Isaac UI showed `/nova_carter_ROS111` and the Pause control while RViz remained open behind it.
- A separate default `./start_nav_all.sh` cold-start regression also completed with all health checks passed and `Isaac WebRTC : disabled`, preserving the original low-overhead behavior.
- The standalone `/isaac-sim/runheadless.sh` route and AppImage connection to `127.0.0.1` were also validated for Isaac-only UI access.
- The latest `./stop_nav_all.sh` run stopped `isaac-sim`, `isaac-ros-nvblox`, and `ros2-dev-humble`; the standalone WebRTC Client and server processes were also closed. The next session starts from a clean stopped state.

## Cold-start baseline: 2026-07-19 end of day

Use this baseline after shutdown or whenever inspection confirms that no project process survives.

- The host was shut down after documentation synchronization. `ros2-dev-humble`, `isaac-ros-nvblox`, and `isaac-sim` were verified stopped; no ROS process survives that shutdown.
- `start_nav_all.sh` and `stop_nav_all.sh` passed Bash syntax validation. The workspace Isaac launcher passed Python syntax validation.
- For the next saved-map navigation session, run `./start_nav_all.sh` from `/home/shenfq/projects/ros-humble`.
- Use `ISAAC_WEBRTC=1 ./start_nav_all.sh` when RViz and the Isaac WebRTC UI are both required.
- The launcher starts Isaac Sim headlessly, loads `warehouse_v3` through nvblox, starts AMCL/Nav2, and opens only RViz2. Require the final line `[ OK ] All startup health checks passed.`
- Aggregate logs are `logs/start_nav_all/{isaac_sim,nvblox,navigation,rviz}.log`; the immediately preceding run is retained as `.previous`.
- Use `./start_nav_all.sh --health-check` for a read-only recheck of an already running stack. Use `./stop_nav_all.sh` for the project shutdown path.

The current engineering objective after validated simulation navigation is real-hardware migration: choose manual or surveyed fixed Initial Pose, measure the real base-to-LiDAR transform and odometry noise, then retune AMCL without changing the validated map or costmap safety settings.

## Validated artifacts and configuration

- `maps/nvblox/warehouse_v3.nvblx` is about 37 MB.
- `maps/nvblox/warehouse_v3.ply` is about 14 MB.
- `maps/2d/warehouse_v3.pgm` is `417 x 424` at `0.05 m/pixel`.
- `maps/2d/warehouse_v3.yaml` has origin `[-14.4, -7.6, 0]` and must use `free_thresh: 0.196` so gray-205 unknown cells remain unknown.
- Reloaded `/map` statistics were 117,570 unknown, 52,618 free, and 6,620 occupied cells.
- `launch/nvblox_with_map.launch.py` and `launch/nav_stack.launch.py` default to warehouse_v3.
- `configs/nav2_params.yaml` uses `GridBased.allow_unknown=false`, `global_costmap.track_unknown_space=true`, `robot_radius=0.35`, `inflation_radius=0.45`, and `xy_goal_tolerance=0.10`.
- The simulated projected `/scan` uses `base_link`, height `0.10..0.65 m`, range minimum `0.5 m`, 361 rays, and Best Effort/Volatile QoS.
- Rotation is limited to about `0.35 rad/s`; relevant behavior plugin limits require a Navigation restart after configuration changes.

## Saved-map runtime design

The normal launcher defaults are:

```text
LOCALIZATION_MODE=amcl
AMCL_INITIAL_POSE_MODE=odom_identity
START_RVIZ=1
```

`odom_identity` is valid only for the odom-aligned warehouse_v3 Isaac simulation. `nav_stack.launch.py` also supports:

```text
localization_mode:=ground_truth
localization_mode:=amcl
amcl_initial_pose_mode:=odom_identity
amcl_initial_pose_mode:=fixed amcl_initial_x:=X amcl_initial_y:=Y amcl_initial_yaw:=YAW
amcl_initial_pose_mode:=manual
```

- Use `ground_truth` for simulation verification; it publishes identity `map -> odom` and does not require `2D Pose Estimate`.
- Use AMCL `manual` for an arbitrary real-robot start and set RViz `2D Pose Estimate`.
- Use AMCL `fixed` only for a surveyed docking or start pose.
- The one-shot `/amcl_pose_initializer` waits for active AMCL and stationary odometry, publishes Initial Pose three times, verifies `/amcl_pose`, and exits. It must not remain running after success.

## Required saved-map navigation health

Before sending a Goal, require:

- Exactly one nvblox node, container, pointcloud padder, projected scan node, relay, map server, Nav2 server, AMCL, and RViz instance.
- No ground-truth `map -> odom` publisher while using AMCL, and no leftover one-shot initializer.
- `map_server`, `amcl`, `controller_server`, `planner_server`, `behavior_server`, and `bt_navigator` active.
- `/navigate_to_pose`, `/spin`, and `/backup` available.
- RViz Fixed Frame `map` and `use_sim_time=true`.
- Warehouse map dimensions `417 x 424`, resolution `0.05 m`, live `/scan`, live nvblox occupancy, and `map -> base_link` TF.
- The robot center free in `/map`, global costmap, and local costmap.

Use `2D Goal Pose` in RViz and wait for the current action to reach a terminal state before sending another Goal. Near map boundaries or obstacles, choose a nearby safer replacement rather than weakening the validated footprint or inflation geometry.

## Mapping boundary

Do not resume mapping into warehouse_v3 unless intentionally creating a newer version. For any mapping or map-save task, stop saved-map Navigation and old nvblox launches, then follow [the map-generation tutorial](../../../docs/map_gen/README.md) completely. Never load v1/v2 while creating a new version.
