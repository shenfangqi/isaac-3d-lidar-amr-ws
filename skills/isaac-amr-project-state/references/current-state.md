# Current authoritative project state

## Physical Jetson target: recorded 2026-09-06

The project's real-hardware target is reachable through the local SSH alias `isaac-jetson` as user `shenfq`. Connection details, verified platform inventory, authentication rules, and safe remote-operation conventions are maintained in [jetson-target.md](jetson-target.md). Read that reference before every Jetson operation; do not copy passwords into project files or commands.

- The Jetson `carbot-ros2` workspace retains the micro-ROS Agent and now runs it as the enabled user service `micro-ros-agent.service`, listening on UDP 8888 in `ROS_DOMAIN_ID=0` to match the completed ESP32 firmware. The service does not publish `/cmd_vel`.
- Isaac simulation also uses Domain 0 but remains isolated with loopback-only `cyclonedds_ros_local.xml`. The opt-in workstation LAN configuration is `configs/cyclonedds_ros_jetson.xml`, loaded by `scripts/real_robot_ros_env.sh` inside a host-networked project container.
- A non-motion `std_msgs/msg/String` probe passed bidirectionally between workstation CycloneDDS and Jetson Fast DDS on Domain 0. The ESP32 was intentionally off during this check, so repeat the `/cmd_vel` endpoint check after it is started.
- The ESP32 static-resource issue reported as `publisher init failed` was fixed in its firmware. After clearing stale Agent sessions, `carbot_base` maintained one `/cmd_vel` subscription and four publishers: `/wheel_ticks`, `/imu/data_raw`, `/battery_state`, and `/carbot/status`. With the chassis lifted, 2026-09-06 ROS 2 smoke tests verified all differential-drive directions through standard `/cmd_vel`: forward at `linear.x=0.08 m/s`, reverse at `linear.x=-0.08 m/s`, in-place left at `angular.z=+0.25 rad/s`, and in-place right at `angular.z=-0.25 rad/s`. The user visually confirmed the track directions were correct. Every motion was followed by repeated zero Twist messages, and the final `/cmd_vel` subscription count remained one. The Jetson currently lacks matching Python type support for the ESP32's `carbot_msgs`, so the custom wheel/status payloads were not decoded during those tests.
- Do not let the existing Isaac navigation launcher load the physical LAN DDS profile. Real hardware still requires verified `/odom`, `odom -> base_link`, Mid-360 data/extrinsics, `use_sim_time=false`, AMCL initialization, command timeout, and physical emergency stop before any nonzero command is allowed.

## Live checkpoint: 2026-08-29

This is the newest authoritative state after converting and validating the project as a Mid-360-only runtime. `ISAAC_WEBRTC=1 ./start_nav_all.sh` is currently running with RViz, and the Isaac WebRTC AppImage is connected/available at `127.0.0.1` for user inspection.

- The current live start completed with `[ OK ] All startup health checks passed.` Keep `isaac-sim`, `isaac-ros-nvblox`, and `ros2-dev-humble` running until the user finishes visual confirmation; stop them with `./stop_nav_all.sh` afterward.

- The normal and WebRTC Isaac launchers now install `Livox_Mid360_Approx`, publish `/livox/lidar`, and retain the Carter asset's legacy sensor prim path only as an internal mounting/graph connection.
- nvblox uses `mid360_nvblox.launch.py`, `/livox/lidar_nvblox`, and a `1000 x 40` padded spherical cloud. The independent XT32 launch and configuration files were removed.
- Nav2, Frontier Exploration, RViz, startup health checks, shutdown patterns, and user documentation now default to Mid-360.
- The post-conversion `./start_nav_all.sh` regression completed with `[ OK ] All startup health checks passed.` It verified `/livox/lidar`, `/livox/lidar_nvblox`, nvblox `1000 x 40`, live occupancy and Scan, AMCL/Nav2 lifecycle, map dimensions, RViz, and `map -> base_link`.
- The post-conversion `ISAAC_WEBRTC=1 START_RVIZ=0 ./start_nav_all.sh` regression also completed with all health checks passed, including the streamed Mid-360 stage and WebRTC endpoint. Both launch modes were stopped cleanly afterward.

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
- The only simulated raw 3D LiDAR topic is `/livox/lidar`; nvblox consumes `/livox/lidar_nvblox` padded to `1000 x 40`.
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
