# Jetson 履带底盘通信链路

## USB 通信架构

```text
工作站 Nav2（ROS_DOMAIN_ID=0，Fast DDS/LAN）
  -> /cmd_vel geometry_msgs/msg/Twist
  -> Jetson micro-ROS Agent
  -> CP2102 USB-UART（921600 baud）
  -> ESP32 micro-ROS 履带差速控制
```

ESP32 不再通过 Wi-Fi 连接 Jetson；USB 串口只替换 ESP32 与 Agent 之间的
XRCE-DDS transport，ROS 话题、Domain 0 和 Jetson 到工作站的物理 LAN DDS 保持
不变。手机网页仍通过 Wi-Fi 访问 Jetson。Isaac 仿真也保留 Domain 0，但默认加载
`cyclonedds_ros_local.xml` 并只绑定 `lo`；真实机器人则显式执行
`scripts/real_robot_ros_env.sh`，切换到与 Jetson 硬件节点及 micro-ROS Agent 一致的
Fast DDS。不要在真实机器人环境中同时启动默认仿真栈。

Jetson Wi-Fi 已固定为 `192.168.1.109/24`，网卡 MAC 为
`50:2e:91:71:5a:9f`；NetworkManager 连接 `TP-LINK_652DC7` 已持久关闭 Wi-Fi
省电。MID-360 专网保持 `enP8p1s0=192.168.2.100/24`。2026-09-19 清理重复的
可靠点云订阅后，主机容器可持续接收 `odom -> livox_frame`，证明双网卡本身不会
破坏 TF；单路原始 PointCloud2 通过 Wi-Fi 仍仅约 `3.8 Hz`，是后续车端压缩或
Jetson 本地融合要解决的带宽限制。

## Jetson 单机 nvblox

2026-09-19 已完成 Jetson 本机部署与静态验收：

- 工作区：`/home/shenfq/Projects/isaac_ros-dev`。
- 镜像：`carbot-isaac-ros-nvblox:3.2`，基于官方 Isaac ROS
  `aarch64.ros2_humble`，apt nvblox 版本为 `3.2.5`。
- 运行时用源码 overlay 覆盖 `nvblox_ros`，固定到
  `shenfangqi123/isaac_ros_nvblox` 的 `fix-filepath-service-hang` 分支提交
  `3f77a90`。官方 apt 二进制的 `save_map`、`load_map` 和 `save_ply` 回调会
  挂起，不能退回只使用 `/opt/ros/humble` 中的版本。
- 容器必须使用 Isaac ROS 自带的 `rtps_udp_profile.xml`。宿主和容器虽能通过
  Fast DDS 发现端点，但默认共享内存通道曾导致容器只看到端点、收不到 TF/点云
  消息；UDP-only profile 已验证解决。
- 默认订阅 `/mid360/points_xyz`（约 4992 点/帧、10 Hz）。静态地图约
  `9.1 Hz`，容器约 `34% CPU / 388 MiB`。完整 `/livox/lidar` 可用，但地图约
  `6.7 Hz`、容器约 `79% CPU`，仅保留作诊断模式。
- 静态验收地图 `152 x 56 @ 0.05 m`；`save_map`、`save_ply`、`load_map`
  均返回成功。验收产物位于
  `/home/shenfq/Projects/isaac_ros-dev/maps/acceptance/`。

管理脚本部署到 Jetson 后使用：

```bash
scripts/jetson_nvblox_container.sh recreate compact
scripts/jetson_nvblox_container.sh status
scripts/jetson_nvblox_container.sh logs
scripts/jetson_nvblox_container.sh stop
```

容器采用 `unless-stopped` 自动恢复策略。脚本中的 `full` 模式仅用于性能诊断，
单机导航默认保持 `compact`，给定位和 Nav2 留出资源余量。

完整建图应使用 `mapping` 模式。停车结束建图后，一次保存 Nav2 二维占据栅格、
SLAM Toolbox 位姿图、nvblox 三维地图和 PLY 网格：

```bash
scripts/jetson_nvblox_container.sh recreate mapping
# 行驶建图完成并停车后：
scripts/jetson_save_maps.sh carbot_site_01
```

默认输出到 `/home/shenfq/Projects/isaac_ros-dev/maps/real/`。保存脚本会先核对四个
ROS 服务，并给每次调用设置 30 秒超时；只有二维地图、位姿图、`.nvblx` 和 `.ply`
全部返回成功才视为完成。由于官方 apt nvblox 的文件服务存在已知挂起问题，保存
必须通过当前带 `3f77a90` overlay 的容器执行。

ESP32 关闭时可以检查容器和启动文件，但 `odom -> base_footprint` 不再更新，不能
完成动态建图或定位验收。此时 nvblox/SLAM 报缺少该 TF 属于预期现象，也不得为了
消除日志而伪造里程计。

已保存地图的静态加载检查使用安全模式；地图必须位于 Jetson 工作区内：

```bash
scripts/jetson_nvblox_container.sh recreate navigation-safe \
  /home/shenfq/Projects/isaac_ros-dev/maps/real/carbot_site_01.yaml
```

该模式固定传入 `autostart:=false`，只创建 Nav2、AMCL 和点云投影节点，不激活
lifecycle 节点。它用于核对参数、地图和 ROS 接口，不代表允许导航，也不会自行
发布速度。切换模式必须使用 `recreate`；`start` 只会恢复容器上次创建时的模式。
容器保存模式和完整启动命令标签；若 `start` 所给模式或地图与既有容器不一致，
脚本会拒绝启动并要求使用 `recreate`，避免恢复错误的运行配置。

ESP32、MID-360 和 Jetson 都开机后，先运行只读前置验收：

```bash
scripts/jetson_nav_preflight.sh
```

它检查 systemd 服务、未激活的 Nav2 状态、`/cmd_vel` 拓扑与静默状态、唯一
`/odom` 发布者，以及 `/wheel_ticks`、`/odom`、点云、`/scan` 和两段 TF 是否能
在限时内收到新数据。任何一项失败都会返回非零状态并提示不得激活 Nav2；脚本
自身不发布速度、不改 lifecycle 状态。

正式加载保存地图时统一使用下面的无运动启动入口，不再手工逐条调用生命周期服务：

```bash
scripts/jetson_navigation_start.sh \
  /home/shenfq/Projects/isaac_ros-dev/maps/real/carbot_site_route_retry_20260919.yaml \
  INITIAL_X_M INITIAL_Y_M INITIAL_YAW_RAD navigation-safe
```

脚本固定执行“重建未激活容器 → 只读前置检查 → 激活 map_server/AMCL → 发布并
确认初始位姿和 `map -> odom` → 激活导航节点 → 核对生命周期、速度拓扑与静默”
顺序。任一步失败都会停止容器；成功时也不会发送导航目标。短生命周期 DDS 客户端
统一在一个 Python 初始化进程内执行，并预留发现稳定时间，避免此前偶发的服务发现
超时。若只调控制链，末尾改为 `navigation-diagnostic`，最终输出会被隔离到
`/cmd_vel_diagnostic`，真实 `/cmd_vel` 保持零发布者。

2026-09-19 的首次移动建图发现，原 `0.30 rad/s` 实车 Nav2 转向上限低于履带
可靠启动门槛：`+0.20 rad/s` 短脉冲产生 `0--0.8°`，`+0.30 rad/s × 2 s`
仅产生约 `3.9°`；`+0.40` 和 `+0.50 rad/s × 2 s` 分别稳定产生约 `13.6°`
和 `21.0°`，`-0.50 rad/s × 2 s` 产生约 `24.9°`。轮式里程计与 LiDAR SLAM
的角度增量一致。按 `0.254 m` 有效轮距计算，`0.40 rad/s` 对应每侧履带约
`0.0508 m/s`，与已测正向稳定死区吻合。因此实车初始 Nav2 转向范围改为
`0.40--0.50 rad/s`，线速度仍限制为 `0.10 m/s`；这只是启动门槛修正，后续
仍需闭环目标、停车尾段和左右不对称调优。

同日较长返回目标暴露出 RPP 与履带死区的闭环：原
`max_angular_accel=0.50 rad/s²` 在 20 Hz 下把大角度起始转向恒定限制为
`0.025 rad/s`，低于 `0.40 rad/s` 门槛，零 odom 又使限幅每周期重新开始。
隔离诊断模式把最终输出改接 `/cmd_vel_diagnostic`，确认真实 `/cmd_vel` 零发布者
时可稳定复现。RPP 内部限值现为 `10.00 rad/s²`，让其请求 `0.50 rad/s`；最终
velocity smoother 仍以 `±2.00 rad/s²` 将实际输出从 `0.10` 平滑到
`0.50 rad/s`，约 0.2 秒跨过门槛。该命令曲线已离线录包验证；随后一次新位置
综合验收中，连续建图路线约 `0.802 m`，Nav2 规划返回约 `0.633 m` 并返回
`SUCCEEDED`，最终位置/航向误差约 `5.7 cm/9.6°`，均在配置容差内。总平移约
`1.435 m`。该返回目标曾采用路径接近方向而非起始方向，随后补做 `/spin` 左转
`82°`，最终航向约 `99.2°`，与路线起始约 `99.0°` 相差约 `0.2°`；原地转向
位置漂移约 `3.4 cm`。无需追加更长距离测试。

## Jetson Agent

Jetson 项目：

```text
/home/shenfq/Projects/carbot-ros2
```

ESP32 开发板通过板载 CP2102 枚举为 `10c4:ea60`、USB serial `0001`。Agent 必须
使用稳定的 `by-id` 名称，不能写死 `/dev/ttyUSB0`，也不能使用会随 USB-C/USB-A
物理端口改变的 `by-path`：

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Jetson 用户必须属于 `dialout`，并在当前登录会话中实际获得该组权限：

```bash
id
test -r /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
test -w /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

部署新的 Agent 前先构建并安装 `carbot_hardware`。复制 unit 会覆盖现有 UDP unit，
所以先保存一份只用于回滚的副本；不要同时运行 UDP 和 serial Agent：

```bash
cd /home/shenfq/Projects/carbot-ros2
colcon build --packages-select carbot_msgs carbot_hardware
cp ~/.config/systemd/user/micro-ros-agent.service \
  ~/.config/systemd/user/micro-ros-agent.service.udp-backup
systemctl --user stop micro-ros-agent.service

# Agent 停止后才能通过同一个 CP2102 串口烧录 ESP32 USB transport 固件。

cp install/carbot_hardware/share/carbot_hardware/systemd/micro-ros-agent.service \
  ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now micro-ros-agent.service
```

unit 默认使用 `921600 8N1`。启动脚本会等待设备出现，USB 拔出或 Agent 退出后由
systemd 自动重启；插回 USB 后无需重启 Jetson。服务 active 只表示守护进程正在
运行或等待设备，不代表 ESP32 ROS session 已建立，因此仍必须检查 topic freshness：

```bash
systemctl --user status micro-ros-agent.service
journalctl --user -u micro-ros-agent.service -n 100 --no-pager
readlink -f /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
ss -lun | grep ':8888'  # 应无输出
ros2 topic hz /wheel_ticks
ros2 topic hz /imu/data_raw
```

当前生产固件使用 USB transport，不要同时启动旧 UDP Agent。更换 Jetson 物理 USB
端口无需修改服务配置；只要 CP2102 的 USB serial 保持为 `0001`，`by-id` 路径就保持
不变。服务 active 但 topic 不再更新时，不得发送运动命令；先检查 USB 数据面和
`/carbot/status` freshness。

## 手机网页遥控建图

手机控制网页现在应运行在 Jetson，而不是 ESP32。Jetson 节点把网页指令送入已经
标定的正式控制链：

```text
手机浏览器 -> Jetson :8080 -> /cmd_vel_command
  -> cmd_vel_compensator -> /cmd_vel -> ESP32 micro-ROS
```

节点默认使用 `0.10 m/s` 直线速度和 `0.40 rad/s` 转向速度，网页默认未使能。
使能时会检查 `/cmd_vel_command` 只有本节点一个发布者，且只有速度补偿节点一个
订阅者；运行中拓扑变化会自动停用。按住按钮时浏览器每 100 ms 续租一次，松手、
页面失焦或网络中断后，Jetson 最迟约 0.3 秒发布零速度。ESP32 自身的 500 ms
`/cmd_vel` 看门狗仍是下一层保护。网页停车和软件看门狗都不能替代物理急停。

部署到 `/home/shenfq/Projects/carbot-ros2` 并构建 `carbot_hardware` 后，安装用户服务：

```bash
mkdir -p ~/.config/systemd/user
cp install/carbot_hardware/share/carbot_hardware/systemd/carbot-web-teleop.service \
  ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now carbot-web-teleop.service
```

确认 `carbot-command-compensation.service`、micro-ROS Agent、轮式里程计、状态估计、
MID-360 和 mapping 容器均正常后，在同一可信 Wi-Fi 的手机打开：

```text
http://<Jetson-Wi-Fi-IP>:8080/
```

开始房间建图前使用 `scripts/jetson_nvblox_container.sh recreate mapping`。只允许
Web 手动控制这一个上游速度源；不要同时运行 Nav2、键盘遥控或其他
`/cmd_vel_command` 发布者。停车并在网页停用后，再执行
`scripts/jetson_save_maps.sh MAP_NAME` 保存地图。

Agent 与轮式里程计用户服务：

```bash
ssh isaac-jetson 'systemctl --user status micro-ros-agent.service'
ssh isaac-jetson 'systemctl --user status carbot-wheel-odometry.service'
ssh isaac-jetson 'journalctl --user -u micro-ros-agent.service -n 100 --no-pager'
```

2026-09-16 已在 Jetson 的 `/home/shenfq/Projects/carbot-ros2` 部署
`carbot_msgs` 与 `carbot_hardware`。`carbot-wheel-odometry.service` 是唯一
`/wheel_ticks -> /odom` 及 `odom -> base_footprint` owner，并在 ESP32
`time_synchronized=true` 后才发布。不要启动 `carbot_driver`，因为它会成为额外的
非零 `/cmd_vel` 发布者。

## 工作站真实机器人环境

在 `ros2-dev-humble` 等使用 host 网络的项目容器内：

```bash
source /opt/ros/humble/setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/install/local_setup.bash
source /workspace/ros-humble/isaac_3d_lidar_amr_ws/scripts/real_robot_ros_env.sh
```

当前 Nav2 的 `navigation_launch.py` 将 controller 输出重映射到
`/cmd_vel_nav`，再由 `velocity_smoother` 输出 `/cmd_vel_command`。Jetson 的
`cmd_vel_compensator` 对负 `angular.z` 应用规范参数 `0.896`，并输出标准
`/cmd_vel`；履带控制端仍只订阅最终的 `/cmd_vel`。该分层避免同话题回环，
也让 ESP32 固件保持不变。

## 启动真实 Nav2 前的硬门槛

阶段 E 已提供独立的 `launch/carbot_real.launch.py`、`configs/carbot/real.yaml`、实机
Nav2/AMCL 参数和系统时间配置，并已完成约 `1.435 m` 的受监护闭环路线验收。
不得用仿真启动器控制履带车。每次允许真实 Nav2 加载物理 LAN DDS 配置前，仍必须
逐项验证：

- ESP32 对 `/cmd_vel` 的履带差速换算、限速和指令超时停车已生效。
- 架空试验已验证外置 ESP32 人工断电开关可停车且重新上电不自启；该开关仅用于
  专人值守的低速标定。标准硬件急停仍未安装，自主导航验收前必须补齐。
- `carbot-wheel-odometry.service` active，物理 ROS 图中 `/odom` 发布者恰好一个，
  时间戳和 `frame_id` 正确。
- `odom -> base_footprint` 由该 Jetson 服务唯一发布，不能使用 Isaac
  `/chassis/odom` relay。
- Mid-360 提供真实点云或 `/scan`，并使用实测 `base_link -> front_3d_lidar` 外参。
- Nav2、AMCL、RViz 和传感器全部使用系统时间，即 `use_sim_time=false`。
- AMCL 使用 `manual` 或经过测量的 `fixed` Initial Pose，不使用仿真的 `odom_identity`。
- 物理 LAN Domain 0 中只有一个最终 `/cmd_vel` 发布链路。

在这些门槛完成前，只允许检查 Agent、ROS 节点和 Topic endpoint，不发送任何非零速度命令。

阶段 F 训练环境不会替代以上门槛。持续待办与每项所需证据记录在
`isaac_lab/carbot_env/hardware_calibration_backlog.yaml`；仿真结果不能把其中
任何项目标记为完成，真实测量值必须回填 Carbot 公共参数和域随机化范围。
