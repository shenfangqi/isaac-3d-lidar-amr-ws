# ROS and Docker diagnostics

## Duplicate-process checks

Before mapping or loading a map:

```bash
ros2 node list --no-daemon | grep nvblox
ros2 topic info /front_3d_lidar/lidar_points -v --no-daemon
ps -eo pid,ppid,lstart,args | grep -E 'nvblox|component_container|ros2 launch' | grep -v grep
```

Require exactly one `/pointcloud_padder`, `/nvblox_node`, and `/nvblox_container`. A stale saved-map wrapper plus pure nvblox creates conflicting publishers.

## High-value ROS checks

```bash
ros2 node list --no-daemon
ros2 topic list -t --no-daemon
ros2 service list -t --no-daemon
ros2 lifecycle get /bt_navigator --no-daemon
ros2 lifecycle get /planner_server --no-daemon
ros2 lifecycle get /controller_server --no-daemon
ros2 lifecycle get /behavior_server --no-daemon
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /scan -v --no-daemon
ros2 topic info /map -v --no-daemon
ros2 topic info /global_costmap/costmap -v --no-daemon
ros2 topic info /front_3d_lidar/lidar_points_nvblox -v --no-daemon
ros2 param get /nvblox_node use_sim_time --no-daemon
ros2 param get /nvblox_node lidar_width --no-daemon
ros2 param get /nvblox_node lidar_height --no-daemon
ros2 param get /nvblox_node lidar_min_valid_range_m --no-daemon
```

Humble `ros2 action list` has no `--no-daemon`. When the CLI daemon is unreliable, discover an Action server through its hidden `ACTION/_action/send_goal` service:

```bash
ros2 service list --no-daemon --include-hidden-services | grep '/navigate_to_pose/_action/send_goal'
```

## Logs

Aggregate launcher logs are under `/home/shenfq/projects/ros-humble/logs/start_nav_all/`.

Newest Nav2 node logs inside the Humble container:

```bash
f=$(ls -t /root/.ros/log/bt_navigator_*.log | head -1); tail -80 "$f"
f=$(ls -t /root/.ros/log/planner_server_*.log | head -1); tail -80 "$f"
f=$(ls -t /root/.ros/log/controller_server_*.log | head -1); tail -80 "$f"
f=$(ls -t /root/.ros/log/behavior_server_*.log | head -1); tail -80 "$f"
```

Use `/home/admin/.ros/log` for processes launched as `admin` in the nvblox container.

## QoS facts

- `/scan`: Best Effort, Volatile. Configure RViz LaserScan the same way.
- `/nvblox_node/static_occupancy_grid`: Reliable, Volatile. Disable transient-local subscription when saving it.
- `/map`: Reliable, Transient Local.

## Diagnostic interpretations

- `/clicked_point` with one RViz publisher and zero subscribers: `Publish Point` cannot move the robot; use `2D Goal Pose` or start an explicit bridge.
- `/map=0`, global costmap `99`, local costmap `0`: isolate static-map/inflation cost from live Scan before changing geometry.
- `worldToMap failed`: the goal, footprint, or tolerance extends outside map bounds.
- Goal click with inactive `bt_navigator`: inspect lifecycle and confirm Spin/BackUp/Wait behavior servers exist.
- Ground-truth failure followed by movement can be Nav2 Spin/BackUp recovery; inspect Action feedback and recovery count.
- AMCL turn corrections can come from non-deskewed 3D-cloud projection and rotational noise; measure `map -> odom` and actual angular velocity before blaming the map.
- A grid near `64 x 24` with zero occupied cells indicates bad spherical cloud input or time configuration; verify `1800 x 31`, padded-topic subscription, and startup simulation time.
- Growing ESDF does not prove a usable 2D map. Inspect OccupancyGrid dimensions and unknown/free/occupied counts.
