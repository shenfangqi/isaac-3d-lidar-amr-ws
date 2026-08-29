# nvblox 保存地图与 Nav2 导航教程

本文介绍如何在本项目中加载已经生成的 nvblox 2.5D 地图，启动 Nav2，使用 ground-truth 或 AMCL 定位，在 RViz 中发送目标，并完成安全检查与导航验收。

本文使用已经验证完成的 `warehouse_v3` 作为案例。重新建图、补图、保存和导出 PGM/YAML 的流程请阅读：

```text
docs/map_gen/README.md
```

完成本文后，你应当能够：

- 区分 nvblox 原生地图与 Nav2 二维静态地图。
- 正确启动 Isaac Sim、保存地图模式的 nvblox、AMCL 和 Nav2。
- 理解 `map -> odom -> base_link` TF 链。
- 根据场景选择 ground-truth、AMCL 自动、固定或手动 Initial Pose。
- 在 RViz 中检查 Map、Costmap、LaserScan 和机器人位置。
- 安全发送 `2D Goal Pose` 并判断导航是否真正成功。
- 诊断 Goal 无响应、Costmap 99、Scan 不对齐和 AMCL 转弯跳动。
- 为后续真机迁移保留正确的定位和安全边界。

## 1. 项目导航链路

本项目的导航数据链如下：

```text
warehouse_v3.pgm + warehouse_v3.yaml
  -> nav2_map_server
  -> /map
  -> Global Costmap static_layer
  -> Navfn 全局规划器

/front_3d_lidar/lidar_points
  -> pointcloud_to_laserscan
  -> /scan
  -> AMCL + Global/Local Costmap obstacle_layer

/chassis/odom
  -> topic_tools relay
  -> /odom
  -> Nav2 控制器与 TF

定位模式
  -> ground_truth: 静态 identity map -> odom
  -> AMCL: 根据 /map、/scan 和 /odom 动态发布 map -> odom
```

项目同时加载 `warehouse_v3.nvblx`，用于恢复 nvblox 的原生 3D/2.5D 表示和 RViz 检查。Nav2 静态规划真正使用的是 `warehouse_v3.pgm` 和 `warehouse_v3.yaml`。

不要混淆下面两组文件：

```text
maps/nvblox/warehouse_v3.nvblx  nvblox 原生地图
maps/nvblox/warehouse_v3.ply    导出的三维点云

maps/2d/warehouse_v3.pgm        Nav2 二维栅格图像
maps/2d/warehouse_v3.yaml       Nav2 地图元数据和阈值
```

## 2. warehouse_v3 已验证基准

当前案例的正确结果是：

```text
尺寸:       417 x 424
分辨率:     0.05 m/cell
原点:       (-14.4, -7.6, 0)
unknown:    117,570
free:       52,618
occupied:   6,620
```

`warehouse_v3.yaml` 必须包含：

```yaml
mode: trinary
occupied_thresh: 0.65
free_thresh: 0.196
```

灰度 `205` 在该 PGM 中表示未知格，其换算值约为 `0.196078`。如果写成 `free_thresh: 0.25`，Nav2 会错误地把未知区域加载成自由区域，规划器可能穿过未建图空间。

与地图安全有关的当前配置：

```text
robot_radius:                   0.35 m
inflation_radius:               0.45 m
GridBased.allow_unknown:        false
global_costmap.track_unknown_space: true
xy_goal_tolerance:              0.10 m
yaw_goal_tolerance:             0.25 rad
```

## 3. 容器与 ROS 环境

项目使用三个容器：

```text
Isaac Sim:           isaac-sim
Isaac ROS / nvblox:  isaac-ros-nvblox
ROS 2 Humble / Nav2: ros2-dev-humble
```

先检查：

```bash
sudo docker ps --format '{{.Names}}\t{{.Status}}'
```

进入 Humble 容器：

```bash
sudo docker exec -it ros2-dev-humble bash
```

初始化环境：

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/local_setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
```

进入 nvblox 容器时使用 `admin`：

```bash
sudo docker exec -it -u admin isaac-ros-nvblox bash
```

然后按顺序初始化：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/setup.bash
```

如果 ROS CLI daemon 报 `rclpy.ok()` 或 XMLRPC 错误，对支持的 `node`、`topic`、`service`、`param`、`lifecycle` 等子命令使用 `--no-daemon`，或者改用短生命周期的 rclpy 检查脚本。Humble 的 `ros2 action list` 不支持该选项，因此启动脚本通过 `ros2 service list --no-daemon --include-hidden-services` 检查 `ACTION/_action/send_goal`。这样可避免组件实际正常但脚本在启动 Navigation/RViz 前误判退出。

## 4. 一键启动当前仿真导航

仓库根目录的脚本当前默认执行：

```text
启动 Isaac Sim
等待 /clock、/chassis/odom 和 3D LiDAR 真正收到消息
加载 warehouse_v3.nvblx
等待 nvblox 服务、OccupancyGrid 和 0.5 m 最小量程
启动 localization_mode:=amcl
启动 amcl_initial_pose_mode:=odom_identity
启动 Nav2
自动启动 RViz
检查节点单实例、地图尺寸、lifecycle、Action、Scan、TF 和安全参数
```

执行：

```bash
cd /home/shenfq/projects/ros-humble
./start_nav_all.sh
```

脚本不再依靠固定几秒的等待来假定组件已经启动。每一阶段只有在实际 ROS 消息、服务或 lifecycle 状态就绪后才会继续；超时会明确失败，不会输出虚假的“全部完成”。

如果 Isaac Sim、nvblox、Nav2、AMCL 或 RViz 已经存在，脚本会拒绝再次启动，防止同名节点、重复 TF 和多个 `/cmd_vel` 控制链。此时只检查当前状态：

```bash
./start_nav_all.sh --health-check
```

正常启动末尾也会自动执行同一套健康检查。只有看到：

```text
[ OK ] All startup health checks passed.
```

才表示当前栈通过了自动检查。检查内容包括 `warehouse_v3` 的 `417 x 424` 地图、nvblox `0.5 m` 最小量程、未知空间禁止规划、TF、`/scan`、Nav2 lifecycle 以及 `/navigate_to_pose`、`/spin`、`/backup`。

该默认值只适用于当前 Isaac 仿真，因为 `warehouse_v3` 与 Isaac 的 odom 使用相同坐标系。它不适用于任意真机起点。

临时切换 ground-truth 验证：

```bash
LOCALIZATION_MODE=ground_truth ./start_nav_all.sh
```

真机或任意地图位置需要手动 Initial Pose 时：

```bash
LOCALIZATION_MODE=amcl \
AMCL_INITIAL_POSE_MODE=manual \
./start_nav_all.sh
```

使用测量好的固定地图坐标：

```bash
LOCALIZATION_MODE=amcl \
AMCL_INITIAL_POSE_MODE=fixed \
AMCL_INITIAL_X=1.0 \
AMCL_INITIAL_Y=2.0 \
AMCL_INITIAL_YAW=0.0 \
./start_nav_all.sh
```

无图形界面的诊断场景可设置 `START_RVIZ=0`；正常桌面导航保持默认值 `1`。

完整停止并确认三个项目容器已经关闭：

```bash
./stop_nav_all.sh
```

一键脚本通过 detached Docker exec 在后台启动 Isaac Sim、nvblox、Navigation 和 RViz，不会再为四个进程分别打开日志终端。它调用项目内 `isaac_sim/auto_play_warehouse.py`，以 `headless=True` 打开 warehouse USD 并自动 Play。桌面上只显示 RViz2；执行 `start_nav_all.sh` 的原终端继续显示就绪进度与最终健康检查。不要在脚本已经启动 Navigation 后，再手工启动第二套 `nav_stack.launch.py`。如果启动中途超时，先阅读文件日志，再执行 `./stop_nav_all.sh`，不要直接重跑启动脚本。

需要同时显示 Isaac Sim WebRTC UI 和 RViz 时，使用统一 streaming 模式；它仍只启动一个 Isaac Sim，并自动打开同一 warehouse USD 和 Play：

```bash
ISAAC_WEBRTC=1 ./start_nav_all.sh
/home/shenfq/桌面/myApps/isaacsim-webrtc-client.AppImage
```

等待启动器输出 `Isaac WebRTC : ready-at-127.0.0.1` 后，再让客户端连接 `127.0.0.1`。默认不设置 `ISAAC_WEBRTC` 时保持原来的无 WebRTC 后台模式。两种完整导航模式以及单独 `runheadless.sh` 模式互斥，切换前都先执行 `./stop_nav_all.sh`。

脚本启动 RViz 时会显式设置 `use_sim_time:=true`，并使用 `Fixed Frame: map`。两项都不可缺少：如果 RViz 使用系统墙钟，Goal 会因时间位于 TF 未来而失败；如果使用 `odom` 发送 Goal，周期重规划在 TF 缓存淘汰原始时间后会报告 `Lookup would require extrapolation into the past`。健康检查会验证 RViz 时间源和配置文件的 Fixed Frame，避免地图显示正常但 Goal 中途失败的假健康状态。

后台模式不再依赖宿主 `gnome-terminal`，因此也不会受到 Snap 版本 VS Code 注入 GTK/GIO 路径的影响。

## 5. 手动分步启动

当需要调试日志或只重启某一部分时，使用分步启动。

### 5.1 启动 Isaac Sim

进入 `isaac-sim` 后使用项目脚本：

```bash
cd /isaac-sim
export isaac_sim_package_path=/isaac-sim
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib
export ROS_DOMAIN_ID=0
./python.sh auto_play_warehouse.py
```

要求至少发布：

```text
/clock
/front_3d_lidar/lidar_points
/chassis/odom
/tf
/tf_static
```

并订阅 `/cmd_vel`。

### 5.2 加载保存的 nvblox 地图

在 nvblox 容器中执行：

```bash
ros2 launch \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nvblox_with_map.launch.py
```

该 wrapper 当前加载：

```text
maps/nvblox/warehouse_v3.nvblx
```

要求 ROS graph 中只有：

```text
1 x /nvblox_node
1 x /nvblox_container
1 x /pointcloud_padder
```

不要同时运行纯空白建图 `xt32_nvblox.launch.py` 和保存地图 wrapper。

### 5.3 启动 ground-truth 导航

在 Humble 容器中执行：

```bash
ros2 launch \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nav_stack.launch.py \
  localization_mode:=ground_truth
```

该模式发布静态 identity `map -> odom`。它用于验证地图、规划器和控制器，不测试 AMCL。

在该模式下：

- 不需要点击 RViz `2D Pose Estimate`。
- 不应同时存在 AMCL 发布的 `map -> odom`。
- 只有当地图本来就是在同一个 odom 坐标中生成时，identity 才成立。

### 5.4 启动 AMCL 仿真自动定位

```bash
ros2 launch \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nav_stack.launch.py \
  localization_mode:=amcl \
  amcl_initial_pose_mode:=odom_identity
```

`odom_identity` 会启动一次性 `amcl_pose_initializer`：

1. 等待 AMCL lifecycle 进入 active。
2. 等待 `/chassis/odom` 且机器人连续静止。
3. 把当前 odom 位姿作为 map 中的 Initial Pose。
4. 发布三次 `/initialpose`。
5. 等待连续 AMCL 响应。
6. 确认成功后干净退出。

正常日志应包含：

```text
Published Initial Pose 1/3
Published Initial Pose 2/3
Published Initial Pose 3/3
AMCL Initial Pose confirmed
process has finished cleanly
```

初始化成功后，`amcl_pose_initializer` 不应继续运行。AMCL 和 Nav2 应继续运行。

### 5.5 AMCL 固定起点

只在机器人每次从已经测量好的地图停靠位启动时使用：

```bash
ros2 launch \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nav_stack.launch.py \
  localization_mode:=amcl \
  amcl_initial_pose_mode:=fixed \
  amcl_initial_x:=1.20 \
  amcl_initial_y:=-0.80 \
  amcl_initial_yaw:=1.57
```

这些值是 `map` 坐标，不是 odom 坐标。示例数字不能直接用于其他场景。

### 5.6 AMCL 手动起点

机器人从任意真实位置启动时：

```bash
ros2 launch \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nav_stack.launch.py \
  localization_mode:=amcl \
  amcl_initial_pose_mode:=manual
```

随后在 RViz 中：

1. 选择 `2D Pose Estimate`。
2. 在二维地图中机器人真实所在位置按下鼠标。
3. 拖动箭头指向机器人真实朝向。
4. 观察 `/scan` 是否落到墙、柱子等地图结构上。
5. Scan 不对齐时重新设置，不要先发送 Goal。

“真实位置”是机器人在物理场景中对应到静态地图的坐标，不是把 RViz 图标随意拖到看起来空旷的地方。

## 6. 定位模式选择

| 使用场景 | `localization_mode` | Initial Pose 模式 | 是否点击 RViz Initial Pose |
|---|---|---|---|
| Isaac 地图/规划快速验证 | `ground_truth` | 不适用 | 否 |
| 当前 odom 对齐的 Isaac v3 | `amcl` | `odom_identity` | 否 |
| 真机固定测量停靠位 | `amcl` | `fixed` | 否 |
| 真机任意启动位置 | `amcl` | `manual` | 是 |

绝对不要在真机上默认使用 `odom_identity`。真机每次开机的 odom 原点通常只是“本次启动位置”，并不代表地图坐标原点。

## 7. TF 链与 RViz Fixed Frame

正常 TF 链为：

```text
map -> odom -> base_link -> lidar frames
```

- `odom -> base_link` 来自底盘里程计或状态估计。
- `map -> odom` 来自 ground-truth 静态变换或 AMCL，两者只能存在一个。
- `base_link -> LiDAR` 必须来自正确的 URDF/static TF。

RViz 正常导航推荐：

```text
Fixed Frame: map
```

这样地图保持静止，机器人在地图中移动。诊断 AMCL 的 `map -> odom` 修正或检查 nvblox 原生 odom 地图时，可以临时使用：

```text
Fixed Frame: odom
```

当前启动脚本使用的 `3d_lidar_amr_ws_1.rviz` 已包含 `/scan` 显示，并固定使用 `map` 作为导航 Fixed Frame。`_3.rviz` 仍可用于以 `odom` 为中心的建图诊断视图。

启动 RViz 示例：

```bash
rviz2 -d \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/configs/rviz/3d_lidar_amr_ws_1.rviz
```

至少显示：

```text
Map:            /map
Global Costmap: /global_costmap/costmap
Local Costmap:  /local_costmap/costmap
LaserScan:      /scan
TF
RobotModel 或机器人坐标轴
Global Plan / Local Plan（如已配置）
```

LaserScan QoS 必须为：

```text
Reliability: Best Effort
Durability:  Volatile
```

推荐可视化参数：

```text
Style:      Points
Size (m):   0.03
Decay Time: 0.2 s 左右
```

不要把 `Size (m)` 误设成数米。`Status: Ok` 只代表 RViz 收到了消息，不代表 Scan 已和静态地图对齐。

## 8. 启动后的必要检查

发送 Goal 前依次确认。

### 8.1 lifecycle 节点

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /behavior_server
ros2 lifecycle get /bt_navigator
```

使用中的节点必须返回：

```text
active [3]
```

ground-truth 模式没有 AMCL，其他导航节点仍须 active。

### 8.2 Action 服务器

```bash
ros2 action info /navigate_to_pose
ros2 action info /compute_path_to_pose
ros2 action info /spin
ros2 action info /backup
```

`/navigate_to_pose`、`/spin` 和 `/backup` 必须有 Action Server。默认 Nav2 行为树需要 spin/backup；删除行为插件可能导致 `bt_navigator` 激活失败。

### 8.3 单实例

要求：

```text
1 x AMCL（AMCL 模式）
1 x /chassis/odom -> /odom relay
0 x ground_truth_map_to_odom（AMCL 模式）
0 x amcl_pose_initializer（初始化成功之后）
1 x /nvblox_node
1 x /nvblox_container
1 x /pointcloud_padder
```

检查进程：

```bash
ps -eo pid,ppid,lstart,args | \
  grep -E 'nav_stack|amcl|relay|ground_truth_map_to_odom|nvblox' | \
  grep -v grep
```

同名 ROS 节点出现两套时，应停止父 launch 并重新启动，不能只忽略警告。

### 8.4 地图与未知区域

要求 `/map` 与 `warehouse_v3` 一致：

```text
417 x 424
0.05 m/cell
origin (-14.4, -7.6)
unknown 数量大于 0
```

确认配置：

```text
global_costmap.track_unknown_space = true
GridBased.allow_unknown = false
```

### 8.5 机器人中心安全

机器人中心在以下三个 OccupancyGrid 中都应为 0：

```text
/map
/global_costmap/costmap
/local_costmap/costmap
```

含义：

- `/map=0`：静态地图中心格是自由格。
- Global Costmap `0`：机器人不在静态/实时障碍膨胀区。
- Local Costmap `0`：机器人附近实时 Scan 没有把车体中心标成障碍。

中心格为自由还不代表整个 footprint 安全。圆形机器人半径为 `0.35 m`，目标附近还要保留膨胀与制动余量。

## 9. `/scan` 的项目参数

`nav_stack.launch.py` 把 3D 点云投影成二维 LaserScan：

```text
target_frame:    base_link
min_height:      0.10 m
max_height:      0.65 m
angle_min/max:   -3.14 / 3.14
angle_increment: 0.0174 rad
range_min:       0.5 m
range_max:       20.0 m
```

当前实测：

```text
361 rays
约 20.6 Hz
frame_id = base_link
无小于 0.5 m 的回波
```

高度切片只保留机器人会碰到的障碍带，避免把地面和高处货架全部投影进二维 Costmap。`range_min=0.5` 用于过滤约 `0.34 m` 的车体自身回波。

如果 `/scan` 看不到：

1. 先检查 `ros2 topic info /scan -v`。
2. 确认 pointcloud-to-laserscan 已有输入点云。
3. 将 RViz QoS 改成 Best Effort + Volatile。
4. 把点大小设为 `0.03 m` 左右。
5. 使用 TopDownOrtho 视角放大机器人附近。

## 10. AMCL 已验证参数

AMCL 配置文件：

```text
configs/amcl_params.yaml
```

关键参数：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `laser_model_type` | `likelihood_field_prob` | 概率似然场模型 |
| `do_beamskip` | `true` | 忽略与不完整地图明显不一致的部分束 |
| `max_beams` | `180` | 使用更多 3D 投影后的二维束 |
| `laser_min_range` | `0.5` | 与 `/scan` 自身过滤一致 |
| `laser_max_range` | `20.0` | 与 `/scan` 自身范围一致 |
| `update_min_d` | `0.10` | 每约 0.1 m 允许定位更新 |
| `update_min_a` | `0.10` | 每约 0.1 rad 允许定位更新 |
| `min_particles` | `800` | 粒子数下限 |
| `max_particles` | `3000` | 粒子数上限 |
| `alpha1/2/3/5` | `0.01` | 当前 Isaac 高质量 odom 的运动噪声 |
| `alpha4` | `0.005` | 限制原地旋转引入错误平移扩散 |

这些 alpha 是当前仿真标定结果，不是所有真机的通用值。真机轮胎打滑、编码器误差和 IMU 融合质量不同，必须重新测量。

## 11. 速度与 3D LiDAR 运动畸变

当前 `/scan` 是由旋转式 3D LiDAR 点云投影得到的，并未对每个点进行完整 deskew。机器人高速原地旋转时，一帧内不同时间采到的点会被当成同一时刻，造成墙面弯曲和 AMCL 临时修正。

当前稳定限制：

```text
behavior Spin max_rotational_vel:       0.35 rad/s
RPP rotate_to_heading_angular_vel:      0.35 rad/s
velocity_smoother angular min/max:      -0.35 / 0.35 rad/s
velocity_smoother angular acceleration: 0.4 rad/s^2
desired_linear_vel:                     0.3 m/s
```

这些值位于：

```text
configs/nav2_params.yaml
```

修改 `behavior_server.max_rotational_vel` 后必须重启 Navigation。该 Spin 插件在 lifecycle configure 时缓存参数，仅在运行时 `ros2 param set` 可能显示成功，但实际动作仍使用旧速度。

## 12. 在 RViz 中发送 Goal

第一次测试使用短距离、宽阔、已知自由区域。

步骤：

1. 确认 Initial Pose/自动初始化已经完成。
2. 确认 Scan 与静态墙、柱子结构基本重合。
3. 确认机器人中心在三张地图中都是 0。
4. 选择 RViz `2D Goal Pose`。
5. 在已知自由地面按下并拖出目标朝向。
6. 等待当前 action 到达终态，再发送下一个 Goal。

不要执行以下操作：

- 把 Goal 点到灰色未知区域。
- 把 Goal 点到墙边、货架边或地图边界。
- 在上一个 Goal 未结束时连续点击多个 Goal。
- 仅根据视觉上“地面空旷”忽略 Costmap 膨胀区。

规划器使用机器人中心规划，但碰撞检查使用完整 footprint。推荐目标中心至少离占用障碍 `0.55 m`，并离地图边界至少 `0.60 m`。

## 13. 分阶段导航验收

不要直接从冷启动跳到长距离复杂路线。按以下顺序测试。

### 阶段 A：非运动规划

调用 `/compute_path_to_pose`，确认：

- 起点和终点在地图内。
- 路径不穿过 unknown。
- 起点/终点不在高代价区。
- 此阶段不应产生 `/cmd_vel`。

### 阶段 B：ground-truth 短距离

使用 `localization_mode:=ground_truth` 测试地图、Costmap、规划器和控制器。该阶段不评估 AMCL。

当前项目 ground-truth 已完成五目标回归：

```text
5/5 SUCCEEDED
0 recoveries
最终 XY 误差约 0.090..0.096 m
无明显 TF 跳变
```

### 阶段 C：AMCL 冷启动

要求自动或手动 Initial Pose 成功，Scan 与地图对齐，`map -> odom` 稳定。

当前自动冷启动结果：

```text
Initial Pose 发布 3/3
AMCL 确认误差约 0.002 m / 0.004 rad
初始化节点正常退出
```

### 阶段 D：AMCL 短直线

优先选择 `0.6..1.0 m` 低曲率路径。检查：

- Action 返回 `SUCCEEDED`。
- 恢复次数为 0。
- 终点误差在 goal tolerance 内。
- 停车后线速度小于 `0.01 m/s`、角速度小于 `0.02 rad/s`。

当前冷启动验收：

```text
路径长度 0.709 m
0 recoveries
终点误差 0.098 m / 0.155 rad
最大 TF 修正 0.029 m
```

### 阶段 E：转向、返回与旋转

再测试 90° 转向、返回起点和低速完整旋转。不要用高速 Spin 作为第一次 AMCL 测试。

当前项目结果：

```text
0.841 m 转向 Goal: SUCCEEDED, 0 recoveries
0.771 m 返回 Goal: SUCCEEDED, 0 recoveries
低速完整旋转: SUCCEEDED
```

## 14. 如何理解“机器人跳动”

先确定跳的是物理车、RViz 机器人、地图，还是恢复动作。

### ground-truth 模式

`map -> odom` 是静态 identity。规划失败后的旋转或后退通常来自 Nav2 recovery，不是定位跳变。检查 action feedback 中的 `number_of_recoveries`，并检查 `spin`/`backup` 行为。

### AMCL 模式

如果机器人转向时 RViz 中地图或机器人位置瞬间修正，检查动态 `map -> odom`。本项目曾观察到默认 AMCL 参数在短导航中产生约 `0.168 m` 修正，原因是通用运动噪声、3D 点云投影畸变和较高角速度共同作用。

最终通过以下组合稳定：

- 降低 AMCL alpha，特别是 `alpha4`。
- 使用概率似然场和更多 LaserScan beams。
- 对不一致束启用 beamskip。
- 把转向速度限制到 `0.35 rad/s`。
- Initial Pose 后先短距离收敛，再测试复杂路线。

几厘米的 AMCL 修正并不等于物理小车跳跃。应同时观察 Isaac Sim 物理位置、`/chassis/odom` 和 `map -> odom`。

## 15. Costmap 常见值与隔离诊断

Nav2 OccupancyGrid 常见值：

```text
0:    自由
1..98: 膨胀代价
99:   内切膨胀障碍区附近
100:  致命障碍
-1:   未知
```

典型组合：

```text
/map = 0
global_costmap = 99
local_costmap = 0
```

这表示机器人中心的静态格本身是自由的，但 Global Costmap 认为完整 footprint 已靠近静态或实时障碍。可能原因：

- 静态地图中有散点/墙面。
- 机器人实际距离墙不足 `robot_radius`。
- Global obstacle layer 的 `/scan` 标记了障碍。
- Scan 与地图/定位未对齐。

隔离 Global obstacle layer：

```bash
ros2 param set /global_costmap/global_costmap \
  obstacle_layer.enabled false

ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap "{}"
```

检查结束后必须恢复：

```bash
ros2 param set /global_costmap/global_costmap \
  obstacle_layer.enabled true

ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap "{}"
```

如果关闭 obstacle layer 后仍为 99，优先检查静态地图和机器人到墙的实际距离；如果恢复为 0，检查 `/scan`、自身回波和 TF。

不要直接缩小 `robot_radius` 来掩盖问题。`0.35 m` 是当前机器人安全模型，随意缩小可能让规划路径穿过物理上无法通过的间隙。

## 16. 故障排查表

| 现象 | 主要原因 | 检查与处理 |
|---|---|---|
| 点击 Goal 没反应 | BT 未 active、Action 不存在、目标被立即拒绝 | 检查 lifecycle、`/navigate_to_pose`、BT 日志 |
| 自动启动的 RViz 点击 Goal 与后台测试结果不同 | RViz 未使用仿真时间，或 Fixed Frame 仍是 `odom` | 要求 `/rviz use_sim_time=true` 且导航 Fixed Frame 为 `map` |
| TF 报向未来外推 | RViz 使用系统墙钟给 Goal 加时间戳 | 用 `--ros-args -p use_sim_time:=true` 启动 RViz |
| TF 报向过去外推 | `odom` Goal 的原始时间戳已被 TF 缓存淘汰 | 导航 RViz 使用 Fixed Frame `map`，重新发送 Goal |
| Goal 一发出就失败 | 起点/终点高代价、unknown、地图外 | 检查三张栅格、规划器 `allow_unknown`、地图边界 |
| RViz 看不到 LaserScan | QoS 不兼容、点太小、输入点云缺失 | 设置 Best Effort + Volatile、Size `0.03 m`、检查 `/scan` |
| Scan 与墙平行但整体错位 | Initial Pose 错误或 `map -> odom` 错误 | 重新设置真实位置/朝向，检查 TF 发布者 |
| Scan 形状旋转时弯曲 | 3D 点云未 deskew、角速度过高 | 限速到 `0.35 rad/s`，真机增加点云 deskew |
| AMCL 初始化节点一直运行 | AMCL 未 active、车未静止、无 odom 或确认超时 | 检查日志、`/chassis/odom`、`/amcl_pose`、Initial Pose subscriber |
| AMCL 初始化节点退出 | 成功时这是正常行为 | 日志应包含 `AMCL Initial Pose confirmed` |
| AMCL 模式存在 ground-truth TF | 两个节点争抢 `map -> odom` | 停止错误模式并干净重启唯一 nav launch |
| Global Costmap 中心为 99 | footprint 靠近静态/实时障碍 | 隔离 obstacle layer，不要先缩机器人半径 |
| `worldToMap failed` | 目标或 footprint 超出地图范围 | 把目标移到地图内部并保留边界余量 |
| 地图未知区域可规划 | YAML 阈值或 `allow_unknown` 错误 | 使用 `free_thresh: 0.196`、`allow_unknown: false` |
| 导航中突然旋转/后退 | recovery 被触发 | 检查 action recovery 次数、局部规划和障碍层 |
| 同一节点名称警告 | 启动了两套 nvblox/Nav2/AMCL | 停止父 launch，确认单实例后重启 |
| ROS CLI 报 `rclpy.ok()` | 共享 daemon 状态损坏 | 使用支持的 `--no-daemon` 或短 rclpy 客户端 |

## 17. 日志位置

一键启动的聚合日志直接保存在宿主机：

```text
/home/shenfq/projects/ros-humble/logs/start_nav_all/isaac_sim.log
/home/shenfq/projects/ros-humble/logs/start_nav_all/nvblox.log
/home/shenfq/projects/ros-humble/logs/start_nav_all/navigation.log
/home/shenfq/projects/ros-humble/logs/start_nav_all/rviz.log
```

每次启动会把上一轮同名日志移动为 `.previous`，再创建本轮日志。实时查看 Navigation：

```bash
tail -f /home/shenfq/projects/ros-humble/logs/start_nav_all/navigation.log
```

`stop_nav_all.sh` 只停止进程和容器，不删除这些日志。

Humble 容器中的 Nav2 日志通常在：

```text
/root/.ros/log
```

查看最新日志：

```bash
f=$(ls -t /root/.ros/log/bt_navigator_*.log | head -1); tail -80 "$f"
f=$(ls -t /root/.ros/log/planner_server_*.log | head -1); tail -80 "$f"
f=$(ls -t /root/.ros/log/controller_server_*.log | head -1); tail -80 "$f"
f=$(ls -t /root/.ros/log/behavior_server_*.log | head -1); tail -80 "$f"
```

自动冷启动调试时，也可以把 launch 输出重定向到 `/tmp`。确认初始化成功时重点搜索：

```text
Published Initial Pose
AMCL Initial Pose confirmed
process has finished cleanly
```

## 18. 真机迁移

仿真导航完成不代表真机无需标定。真机至少完成以下工作。

### 18.1 地图与场景

`warehouse_v3` 来自当前 Isaac Sim 场景。只有真实环境几何结构、尺度和坐标基准一致时才可能直接使用。一般情况下应在真实场景重新生成地图。

### 18.2 时间与 TF

要求：

```text
use_sim_time = false
map -> odom -> base_link -> lidar
```

测量并验证 `base_link -> LiDAR` 的 x/y/z、roll/pitch/yaw。错误外参会直接造成 Scan 与墙不重合。

保证 LiDAR、IMU、轮速和主机时间同步。旋转式 3D LiDAR 推荐根据每点时间戳进行 deskew。

### 18.3 里程计

使用轮速+IMU 融合或 LiDAR/视觉惯性里程计提供连续 `odom -> base_link`。记录真实直行、转向和原地旋转误差，再调整 AMCL `alpha1..alpha4`。

### 18.4 Initial Pose

真机任意起点：

```text
amcl_initial_pose_mode:=manual
```

真机测量好的固定停靠点：

```text
amcl_initial_pose_mode:=fixed
```

真机不要使用：

```text
amcl_initial_pose_mode:=odom_identity
```

### 18.5 安全参数

重新实测：

- footprint 或 `robot_radius`。
- 膨胀半径。
- 最大线速度和角速度。
- 制动距离。
- LiDAR 盲区和车体自身过滤。
- 急停、速度看门狗与通信超时。

先架空轮子或使用低速安全区域验证 `/cmd_vel` 符号、里程计方向和 TF，再在真实场地执行短距离 Goal。

## 19. 每次关机前记录

至少记录：

- 当前加载的地图版本。
- 当前定位模式和 Initial Pose 模式。
- Isaac Sim、nvblox、Nav2、RViz 是否运行。
- AMCL 是否已初始化，initializer 是否已退出。
- Nav2 lifecycle 和 Action 状态。
- 机器人 map 位姿与停车速度。
- 三张栅格的机器人中心值。
- 最后一个 Goal 的 action 终态和恢复次数。
- 是否存在 teleop、explorer 或测试脚本等额外 `/cmd_vel` 来源。

## 20. 当前项目结论

截至 `warehouse_v3` 验收：

- nvblox 2.5D 地图已完成、保存并正确重新加载。
- unknown/free/occupied 阈值已经修正。
- ground-truth 五目标导航回归通过。
- AMCL 转向稳定性参数已经调整。
- AMCL 自动 Initial Pose 冷启动通过。
- 冷启动后的短距离导航通过，零恢复。
- 一键启动 RViz 的时间源和导航 Fixed Frame 已修正并加入健康检查。
- 一键启动默认使用无 WebRTC 的 headless 后端；设置 `ISAAC_WEBRTC=1` 后，同一个自动场景实例可同时供 WebRTC Client 和 RViz 使用。
- 严格安全区域的 AMCL 五目标回归通过 5/5，终点 TF 平移误差为 `0.085..0.098 m`。
- 当前仿真地图与导航可视为完成。
- 后续工作是实车外参、里程计、地图和 AMCL 的重新标定与安全验收。

### 20.1 五目标回归如何选择安全点

不要只看目标中心格是不是白色。当前机器人半径为 `0.35 m`，还存在膨胀代价和终点原地转向所需空间。自动验收使用以下更严格条件：

```text
目标到致命障碍的净距:  >= 0.80 m
目标到 unknown 的净距: >= 0.60 m
目标坐标系:             map
Goal 时间源:            Isaac /clock
```

这里的 `0.80/0.60 m` 只用于本次严格回归选点，不是 Nav2 的全局硬阈值，也没有写入 `nav2_params.yaml`。当前 Nav2 的永久几何配置是 `robot_radius=0.35 m`、`inflation_radius=0.45 m`；Frontier Explorer 默认采用障碍 `0.55 m`、unknown `0.40 m`、地图边界 `0.60 m`。其中 `min_goal_distance_m=0.80` 表示候选目标与机器人当前位置的最小间距，不是离障碍物的距离。机械臂预抓取、对接等近距离任务应使用任务专用预靠近点和低速最终接近策略，不能套用严格回归阈值。

2026-07-19 的严格安全回归结果：

| Goal | 路径长度 | 结果 | 恢复次数 | 最终 TF XY 误差 |
|---:|---:|---|---:|---:|
| 1 | 2.386 m | SUCCEEDED | 12 | 0.098 m |
| 2 | 1.873 m | SUCCEEDED | 0 | 0.085 m |
| 3 | 2.527 m | SUCCEEDED | 0 | 0.097 m |
| 4 | 3.027 m | SUCCEEDED | 0 | 0.095 m |
| 5 | 2.387 m | SUCCEEDED | 0 | 0.095 m |

第 1 点的恢复发生在离开上一轮遗留的临界起点时；脱困后的四段全部零恢复。另一组只有约 `0.50..0.57 m` 障碍净距、且要求大幅终点旋转的目标只通过 3/5，日志为碰撞预测和无法取得进展。这说明“坐标正确”与“目标对完整 footprint 安全”是两个不同条件。

## 相关文件

```text
launch/nvblox_with_map.launch.py
launch/nav_stack.launch.py
configs/nav2_params.yaml
configs/amcl_params.yaml
configs/rviz/3d_lidar_amr_ws_1.rviz
src/isaac_3d_lidar_bringup/isaac_3d_lidar_bringup/amcl_pose_initializer.py
start_nav_all.sh
docs/map_gen/README.md
```
