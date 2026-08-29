# nvblox 2.5D 地图构建、补图、保存与验证教程

本文面向需要在 Isaac Sim 中使用 3D LiDAR、nvblox 和 Nav2 构建移动机器人地图的开发者。教程使用当前仓库的 `warehouse_v3` 作为实际案例，覆盖环境启动、点云预处理、空白建图、手动或自动补图、地图验收、版本化保存以及导航验证。

完成本文后，你应当能够：

- 解释本项目生成的是 nvblox 2.5D 占用地图，而不是 SLAM Toolbox 地图。
- 正确启动 Isaac Sim、Isaac ROS nvblox 和 ROS 2 Humble/Nav2 环境。
- 判断 3D LiDAR 数据是否真正生成了可用的二维占用地图。
- 使用键盘或 Frontier Exploration 覆盖场景并补齐地图缺口。
- 导出 `.nvblx`、`.ply`、`.pgm` 和 `.yaml`，并验证保存结果。
- 使用 `ground_truth` 模式验证地图，再为 AMCL 或真机迁移做准备。

## 建图链路与适用范围

本教程在 `/home/shenfq/projects/ros-humble` 工作区中执行。

使用下面的权威建图链路：

```text
3D LiDAR
  -> 固定尺寸点云 /front_3d_lidar/lidar_points_nvblox
  -> nvblox TSDF/ESDF
  -> /nvblox_node/static_occupancy_grid
  -> Nav2 2D 地图
```

最终结果称为 nvblox 2.5D 占用地图，而不是 SLAM Toolbox 地图。创建新版本时不能加载旧 `.nvblx`。nvblox 依赖稳定的位姿来源，本身不负责解决大范围闭环。

项目的实时调试状态维护在 `skills/isaac-amr-project-state/SKILL.md`，容器和 DDS 调试细节维护在 `skills/ros-docker-debug/SKILL.md`。这些文件服务于项目续接和自动化调试；本文专注于面向使用者的完整建图方法。

## 教程案例：warehouse_v3

以下是本项目已经完成并通过目视检查的结果，可用于理解合格产物的形式。地图尺寸与原点由实际覆盖范围决定，不能作为其他场景的固定验收值。

- 原生地图：`maps/nvblox/warehouse_v3.nvblx`，约 `37 MB`。
- 点云导出：`maps/nvblox/warehouse_v3.ply`，约 `14 MB`。
- 2D 地图：`maps/2d/warehouse_v3.pgm`，`417 x 424`。
- 元数据：`maps/2d/warehouse_v3.yaml`。
- 分辨率：`0.05 m/pixel`。
- 原点：`[-14.4, -7.6, 0]`。
- Nav2 重新加载后的栅格统计：unknown `117,570`、free `52,618`、occupied `6,620`。
- `warehouse_v3.yaml` 必须使用 `free_thresh: 0.196`，不能使用会把灰度 205 未知格判成自由格的 `0.25`。
- 最终图像：外墙闭合、内部结构清楚、自由地面无密集盐胡椒障碍噪声。
- 保留 `warehouse_v1`、`warehouse_v2`，绝不覆盖它们。

## 1. 准备容器与环境

使用三个容器：

```text
Isaac Sim:           isaac-sim
Isaac ROS / nvblox:  isaac-ros-nvblox
ROS 2 Humble / Nav2: ros2-dev-humble
```

先检查容器：

```bash
sudo docker ps --format '{{.Names}}\t{{.Status}}'
```

进入 Humble 容器并初始化：

```bash
sudo docker exec -it ros2-dev-humble bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/local_setup.bash
```

进入 nvblox 容器时使用 `admin`：

```bash
sudo docker exec -it -u admin isaac-ros-nvblox bash
```

然后按顺序执行：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
source /workspaces/isaac_ros-dev/install/setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/setup.bash
```

进入并启动 Isaac Sim 前先检查 `start_nav_all.sh`：

```bash
sudo docker exec -it isaac-sim bash
cd /isaac-sim
export isaac_sim_package_path=/isaac-sim
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///workspace/ros-humble/cyclonedds_ros_local.xml
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib
export ROS_DOMAIN_ID=0
./python.sh auto_play_warehouse.py
```

仓库根目录的 `start_nav_all.sh` 当前默认加载保存好的 `warehouse_v3` 并启动 AMCL 导航，适合“使用现有地图导航”，不适合创建空白新地图。重新建图时必须按本教程单独启动纯 nvblox，不能让 `nvblox_with_map.launch.py` 同时运行。

启动组合：手动空白建图需要 Isaac Sim、纯 nvblox，并用 Humble 运行 RViz/teleop/保存工具；自动补图还需 Humble 中的 Nav2 与 frontier explorer；保存地图验证需要 Isaac Sim、加载保存地图的 nvblox 和 Humble Nav2。启动顺序始终为 Isaac Sim、nvblox、最后 Humble 工具/Nav2。要求 Isaac Sim 发布：

```text
/clock
/front_3d_lidar/lidar_points
/chassis/odom
/tf
/tf_static
```

要求 Isaac Sim 订阅 `/cmd_vel`。

## 2. 固定稀疏 3D LiDAR 点云尺寸

不要把 Isaac Sim 原始 RTX 点云直接送给当前 nvblox 配置。原始话题每帧只有约 `22k-25k` 个有效回波且长度变化，nvblox 的球面 LiDAR 模型要求固定 range image。

使用：

```text
raw:    /front_3d_lidar/lidar_points
padded: /front_3d_lidar/lidar_points_nvblox
shape:  1800 x 31 = 55,800 rays
```

让 `pointcloud_padder` 用 NaN 补齐缺失射线。检查实现与配置：

```text
src/isaac_3d_lidar_bringup/isaac_3d_lidar_bringup/pointcloud_padder.py
src/isaac_3d_lidar_bringup/launch/xt32_nvblox.launch.py
src/isaac_3d_lidar_bringup/config/nvblox/xt32_nvblox_base.yaml
```

修改后在 nvblox 容器中重建：

```bash
cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
source /workspaces/isaac_ros-dev/install/setup.bash
colcon build --packages-select isaac_3d_lidar_bringup --symlink-install
```

启动后要求以下实时参数成立：

```text
use_sim_time = true
lidar_width = 1800
lidar_height = 31
lidar_min_valid_range_m = 0.5
```

使用 `0.5 m` 过滤约 `0.34 m` 处的车体自身回波。不要为了获得更多点把它退回 `0.1 m`，否则地图可能出现机器人轨迹状散点。

## 3. 启动纯空白 nvblox

先停止所有 Navigation、AMCL、旧 nvblox 和加载地图的 wrapper。检查进程：

```bash
ps -eo pid,ppid,lstart,args | \
  grep -E 'nvblox|component_container|ros2 launch' | grep -v grep
```

不要调用 `/nvblox_node/load_map`。启动纯建图：

```bash
ros2 launch isaac_3d_lidar_bringup xt32_nvblox.launch.py
```

要求同时只存在：

```text
1 x /pointcloud_padder
1 x /nvblox_node
1 x /nvblox_container
```

要求 nvblox 只订阅 padded 话题，而不是原始可变长度话题。检查：

```bash
ros2 node list --no-daemon | grep -E 'nvblox|pointcloud_padder'
ros2 topic info --verbose \
  /front_3d_lidar/lidar_points_nvblox --no-daemon
ros2 param get /nvblox_node lidar_min_valid_range_m --no-daemon
```

如果共享 ROS CLI daemon 报 `rclpy.ok()`/XMLRPC 错误，对 `node`、`topic`、`param`、`lifecycle` 使用 `--no-daemon`；Action 命令在 Humble 不一定支持该选项，改用短 `rclpy.ActionClient` 检查。

## 4. 确认静止建图已经正常

等待静止扫描产生 `/nvblox_node/static_occupancy_grid`。同时检查：

- 宽高持续扩展。
- 自由格数量大于零。
- 占用格数量大于零。
- 地图分辨率为预期的 `0.05 m`。
- 点云回调存在不等于地图正确，必须检查 2D OccupancyGrid。

本次修复后的静止基准约为：

```text
256 x 249
free = 30,189
occupied = 2,237
```

下面的表现判定为失败：

```text
地图长期约 64 x 24
free 有增长但 occupied = 0
只有 ESDF 点云看起来很密，2D 图却没有墙
```

遇到失败时依次检查 padded 点云是否为 `1800 x 31`、nvblox 是否订阅 padded 话题、`use_sim_time` 是否从进程启动时就是 `true`。

## 5. 配置 RViz 检查视图

使用：

```text
Fixed Frame: odom
```

至少显示：

```text
/nvblox_node/static_occupancy_grid
/nvblox_node/static_esdf_pointcloud
/front_3d_lidar/lidar_points
/scan（启用 Nav2 时）
TF
RobotModel 或机器人 TF
```

配置 QoS：

```text
static_occupancy_grid: Reliable + Volatile
/scan:                Best Effort + Volatile
```

把地图、实时点云、Scan 和物理场景一起比较。不要因为单个 RViz Display 的点大小、Decay Time 或视角设置而判断地图缺失。

## 6. 覆盖场景

### 手动低速覆盖

只保留一个 `/cmd_vel` 控制源。使用键盘低速直行、转弯和原地旋转，进行有重叠的往返路线。不要沿墙高速掠过；在遮挡处和角落停留数秒。

### 自动前沿探索

使用包：

```text
src/isaac_3d_lidar_exploration
```

必要时在 Humble 容器中构建：

```bash
cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
colcon build --packages-select isaac_3d_lidar_exploration --symlink-install
source install/local_setup.bash
```

启动实时地图 Nav2 与前沿节点：

```bash
ros2 launch isaac_3d_lidar_exploration explore_nvblox.launch.py \
  start_enabled:=false
```

保持默认禁用，完成安全检查后再启用：

```bash
ros2 service call /frontier_explorer/set_enabled \
  std_srvs/srv/SetBool "{data: true}"
```

暂停并取消目标：

```bash
ros2 service call /frontier_explorer/set_enabled \
  std_srvs/srv/SetBool "{data: false}"
```

查看状态和安全候选点：

```text
/frontier_explorer/status
/frontier_explorer/frontiers
```

在启用前停止 `teleop_twist_keyboard`。探索节点会检测 `/cmd_vel` 上的意外发布者并拒绝启用。不要让 teleop、Nav2 或测试脚本同时直接控制底盘；需要同时在线时使用 `twist_mux`。

沿用以下保守阈值作为起点：

```text
robot_radius = 0.35 m
inflation_radius = 0.45 m
goal obstacle clearance = 0.55 m
goal unknown-space stand-off = 0.40 m
map boundary margin = 0.60 m
maximum automatic linear speed = 0.20 m/s
```

要求机器人中心在以下三层都为自由值：

```text
/nvblox_node/static_occupancy_grid
/global_costmap/costmap
/local_costmap/costmap
```

要求 `/navigate_to_pose` 就绪且 Nav2 lifecycle 节点为 `active`。探索节点只发送 `NavigateToPose`，不要让它直接发布 `/cmd_vel`。

## 7. 定点补齐地图缺口

收到用户在 RViz 选择的缺口坐标后，先暂停普通前沿探索。不要直接把原始点击坐标作为机器人中心目标。

依次执行：

1. 把世界坐标变换为 OccupancyGrid cell。
2. 检查目标格是否在地图内、是否为自由格。
3. 计算目标到占用格、未知格和地图边界的距离。
4. 在目标附近寻找满足 `0.55/0.40/0.60 m` 阈值的已知自由观察点。
5. 使用 `/compute_path_to_pose` 先规划，不要先运动后判断。
6. 只在两段路径都可达后依次发送 `NavigateToPose`。
7. 到达每个观察点后停留至少 5 秒，让 nvblox 融合多帧回波。
8. 再次检查缺口附近未知格和占用格是否发生合理变化。

本次实际案例：

```text
raw (-3.63,  0.562)
  obstacle clearance = 0.362 m，不安全
  safe viewpoint = (-3.725, 0.425)

raw (-3.72, -6.28)
  unknown clearance = 0.333 m，不安全
  safe viewpoint = (-3.775, -6.225)
```

预规划路径分别约 `3.92 m` 和 `10.31 m`，两段均无恢复成功到达。第二处未知空间间距从约 `0.33 m` 增加到 `0.75 m`。

如果机器人已到达且停留后墙仍不闭合，不要无限重复经过。检查 nvblox 2D slice 高度、墙面点云回波、遮挡、TF 和 LiDAR 最小量程。

## 8. 保存前验收

仅在以下条件都满足后保存：

- 用户确认主要区域和外轮廓闭合。
- 外墙、内部墙体、圆柱/货架等结构与物理场景一致。
- 已知自由区内部没有大块未探索空洞。
- 自由地面没有密集盐胡椒占用点。
- 地图不是只含自由格而没有占用格。
- 地图、Scan 和实时点云在 `odom` 下保持对齐。
- 机器人没有位于静态障碍、膨胀内切区或地图边界。

不要从 ESDF 点云“看起来很密”推断成功。检查最终 OccupancyGrid 的宽高、分辨率、原点、unknown/free/occupied 数量，并导出 PGM 实际打开查看。

## 9. 冻结并保存版本化产物

先暂停探索并等待底盘停止。要求：

```text
abs(linear.x) < 0.01 m/s
abs(angular.z) < 0.02 rad/s
```

检查目标版本文件尚不存在。绝不覆盖 v1/v2；若同名版本存在，先选择新版本名或得到用户明确授权。

在 nvblox 容器中保存原生地图与 PLY：

```bash
ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath \
  "{file_path: /workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/nvblox/warehouse_v3.nvblx}"

ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
  "{file_path: /workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/nvblox/warehouse_v3.ply}"
```

要求两个响应都是：

```text
success=True
```

在 Humble 容器中从 Volatile 地图话题导出 Nav2 地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /workspace/ros-humble/isaac_3d_lidar_amr_ws/maps/2d/warehouse_v3 \
  --ros-args -p map_subscribe_transient_local:=false \
  -r map:=/nvblox_node/static_occupancy_grid
```

检查导出的 YAML 阈值。当前 PGM 使用灰度 `205` 表示未知格，其换算占用概率约为 `0.196078`；如果 YAML 写成 `free_thresh: 0.25`，Nav2 会把未知格错误加载为自由格。本项目应使用：

```yaml
mode: trinary
occupied_thresh: 0.65
free_thresh: 0.196
```

重新加载后必须确认 `/map` 的 unknown 数量大于零，并与原生 nvblox OccupancyGrid 基本一致。不能只根据 RViz 中“出现地图”就判定导出成功。

验证：

```bash
ls -lh maps/nvblox/warehouse_v3.nvblx \
       maps/nvblox/warehouse_v3.ply \
       maps/2d/warehouse_v3.pgm \
       maps/2d/warehouse_v3.yaml

file maps/nvblox/warehouse_v3.nvblx \
     maps/nvblox/warehouse_v3.ply \
     maps/2d/warehouse_v3.pgm

sha256sum maps/nvblox/warehouse_v3.nvblx \
          maps/nvblox/warehouse_v3.ply \
          maps/2d/warehouse_v3.pgm \
          maps/2d/warehouse_v3.yaml
```

要求 `.nvblx` 可识别为 SQLite 数据库、PLY 非空、PGM 尺寸和 YAML 元数据一致。实际打开 PGM，拒绝噪声图。把 root 生成的 2D 文件归还给工作区用户，避免以后无法覆盖或编辑。

## 10. 切换到保存地图并验证导航

PGM 验收通过后，更新：

```text
launch/nvblox_with_map.launch.py -> warehouse_v3.nvblx
launch/nav_stack.launch.py       -> warehouse_v3.yaml
```

先干净停止纯建图 nvblox 和实时地图 Nav2。不要同时运行纯 nvblox 与 `nvblox_with_map.launch.py`。

加载保存地图：

```bash
ros2 launch \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nvblox_with_map.launch.py
```

使用同一次 `odom` 对齐地图进行仿真验证：

```bash
ros2 launch \
  /workspace/ros-humble/isaac_3d_lidar_amr_ws/launch/nav_stack.launch.py \
  localization_mode:=ground_truth
```

在 ground-truth 模式下要求 `map -> odom` 为 identity，不要点击 `2D Pose Estimate`。发送 Goal 前要求：

- 只有一个 nvblox 实例。
- `global_costmap.track_unknown_space: true` 且 `GridBased.allow_unknown: false`，未知区必须作为不可规划区域保留。
- `controller_server`、`planner_server`、`behavior_server`、`bt_navigator` 全部 active。
- `/navigate_to_pose`、`/spin`、`/backup` Action 可用。
- 机器人中心在静态地图、全局和局部 costmap 中都可通行。
- Scan、机器人和保存地图对齐。

ground-truth 导航通过后再测试 `localization_mode:=amcl`。

当前项目已经完成 ground-truth 五目标回归、AMCL 调参与自动 Initial Pose 冷启动验收。地图加载、AMCL 三种初始化模式、RViz Goal、参数说明和导航故障排查请继续阅读：

```text
docs/map_nav/README.md
```

## 11. 真机迁移

保留前沿算法和保存流程，替换接口与安全参数：

```text
use_sim_time = false
pointcloud_topic = 真机 3D LiDAR 话题
odom_topic = 真机融合里程计
global_frame = odom 或稳定 map
robot_base_frame = base_link
```

重新实测 footprint、速度、制动距离、点云自身过滤、LiDAR 盲区和高度切片。必须提供底盘急停与速度看门狗。

仅靠轮式里程计建立大图会累积漂移。为真机提供轮速+IMU、LiDAR/视觉惯性里程计，或带闭环的 SLAM/定位系统。前沿探索只依赖 OccupancyGrid 和 Nav2，可以更换 mapper；nvblox 本身不提供闭环。

## 故障判定表

| 现象 | 优先检查 |
|---|---|
| 地图约 `64 x 24` 且 occupied 为 0 | 点云是否补齐为 `1800 x 31`、nvblox 是否订阅 padded topic、启动时 `use_sim_time` |
| 地面出现机器人轨迹散点 | `lidar_min_valid_range_m` 是否错误回到 `0.1`、是否扫到车体 |
| LaserScan 与墙不对齐 | TF、定位、时间戳和 Fixed Frame；不要盲目移动 Initial Pose |
| Global Costmap 中心为 99 | 静态墙/散点或实时 obstacle layer 进入内切膨胀区；分别隔离图层 |
| 自动探索不移动 | explorer 是否 disabled、teleop 是否触发 `/cmd_vel` 互锁、Nav2 lifecycle/Action 是否就绪 |
| Goal 没反应 | `bt_navigator` 是否 active，`spin`/`backup`/`wait` 行为服务器是否存在 |
| ground-truth 模式下失败后跳动 | 检查 Nav2 recovery 的 spin/backup 和 action 恢复次数 |
| AMCL 转弯时地图或机器人跳动 | 检查 `map -> odom`、3D 点云投影运动畸变、AMCL alpha 和角速度限制；参见导航教程 |
| map saver 等不到地图 | 对 nvblox Volatile OccupancyGrid 设置 `map_subscribe_transient_local:=false` |
| 出现重复地图/节点 | 清理旧 launch，要求只有一个 `/nvblox_node` 和 `/nvblox_container` |
| ROS CLI 报 `rclpy.ok()` | 对支持的命令使用 `--no-daemon`，或使用短 rclpy 客户端 |

## 每次暂停或关机前的记录建议

为了下次能够从正确状态继续，应在项目状态记录中至少保存以下信息：

- 当前创建的是哪个地图版本。
- 使用纯空白建图还是加载保存地图。
- 当前运行/已停止的 Isaac Sim、nvblox、Nav2、teleop、explorer。
- `lidar_min_valid_range_m`、点云尺寸和 `use_sim_time` 实际值。
- 当前地图宽高、分辨率、原点和 cell 统计。
- 机器人位置、explorer enabled 状态和未完成目标。
- 已保存文件、大小、校验和与启动文件引用版本。
- 下一次必须执行的第一步。
