# Isaac 3D LiDAR Frontier Exploration

这个包使用实时 `nav_msgs/OccupancyGrid` 寻找前沿区域，并通过 Nav2
`NavigateToPose` 自动巡航建图。探索节点不直接发布 `/cmd_vel`，因此可以复用
Nav2 的规划、避障、恢复和速度限制，仿真迁移到真机时只需替换传感器和里程计接口。

## 安全设计

- 启动后默认禁用自主运动，必须显式调用服务才能开始。
- 目标中心与已知障碍至少保持 `0.55 m`。
- 目标不会落在未知区边缘，而是在已知自由区内退让至少 `0.40 m`。
- 地图超时、TF 丢失、Nav2 未就绪时不发送目标。
- 地图在导航途中停止更新时会取消当前目标。
- `/cmd_vel` 存在键盘遥控等未授权发布者时拒绝启用。
- 失败、超时或变成障碍的目标会临时加入黑名单。
- 速度默认限制为 `0.20 m/s`，但真机仍必须有急停、底盘速度看门狗和独立碰撞保护。

## Isaac Sim + nvblox

先启动 Isaac Sim 和纯 nvblox，再在 Humble 容器中执行：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/setup.bash

ros2 launch isaac_3d_lidar_exploration explore_nvblox.launch.py \
  start_enabled:=false
```

确认 RViz 中机器人、点云、实时地图和代价地图对齐后，开始探索：

开始前先退出 `teleop_twist_keyboard`。不要让键盘遥控和自动驾驶同时直接发布
`/cmd_vel`；需要两者在线切换时，应通过 `twist_mux` 仲裁。

```bash
ros2 service call /frontier_explorer/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

随时暂停并取消当前目标：

```bash
ros2 service call /frontier_explorer/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

查看状态：

```bash
ros2 topic echo /frontier_explorer/status
```

在 RViz 添加 `MarkerArray`，话题选择
`/frontier_explorer/frontiers`，可以看到安全候选点。

## 真机迁移

如果真机发布 `/odom`、`odom -> base_link` TF 和 3D 点云，可从下面的参数开始：

```bash
ros2 launch isaac_3d_lidar_exploration explore_nvblox.launch.py \
  use_sim_time:=false \
  global_frame:=odom \
  robot_base_frame:=base_link \
  pointcloud_topic:=/lidar/points \
  source_odom_topic:=/odom \
  start_odom_relay:=false \
  start_enabled:=false
```

如果底盘已经提供合格的 `/scan`，再加：

```text
start_scan_converter:=false scan_topic:=/scan
```

探索算法只要求一个与 `global_frame` 对齐的 OccupancyGrid，因而也可替换
`map_topic` 接入其他建图器。若使用 nvblox，真机必须另外提供低漂移的姿态来源；
仅靠轮式里程计长期建大图会累积漂移。大场景需要 LiDAR/视觉惯性里程计，或能闭环的
SLAM/定位系统提供稳定 `map` 坐标系。

真机首次运行前必须实测并调整：

- `robot_radius` 或真实 footprint；
- `goal_clearance_m`、`goal_unknown_clearance_m` 和 inflation 半径；
- 点云转 Scan 的高度范围、自身点过滤和最小量程；
- 最大速度、加减速度、制动距离和传感器盲区。

首次测试应架起驱动轮或使用封闭空场，保持 `start_enabled:=false` 完成全部 TF、地图和
代价地图检查后，再低速启用。
