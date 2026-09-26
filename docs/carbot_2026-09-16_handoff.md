# Carbot 开发交接记录（2026-09-16）

> 当日真机续测更新：`carbot_msgs` 已解码，Jetson 唯一 wheel odometry 服务已部署；
> 架空 watchdog、四向符号、方向死区和静止 IMU 已取得原始证据。最新状态见
> `skills/isaac-amr-project-state/references/current-state.md` 的
> “Physical Carbot checkpoint: 2026-09-16”，原始数据见
> `calibration_data/2026-09-16_lifted/README.md`。车辆当前没有标准实体急停；手机网页红色
> “停止”不能压过持续 ROS 速度命令。架空测试中人工切断 ESP32 供电后，现场目视确认
> 履带停止，重新上电也未自启。该开关仅允许专人值守的低速受控标定；最终自主导航
> 验收仍需直接切断驱动动力或硬件使能的标准急停。

## 2026-09-18/19 真机续测更新

- ESP32 固件提交 `068071e` 已刷写，GitHub issues #1/#2/#3 对应的加速度
  重复重力换算、陀螺仪重复单位换算和过敏 Agent ping 重连均已修复。
- Jetson `CarbotStatus.msg` 已同步并重新编译；新增断连原因、连续 ping 失败数和
  session uptime 字段可正常解码。
- 固件刷新后静态加速度模长中位数约 `9.8037 m/s^2`。左右动态转向和完整一圈
  验证表明 ESP32 gyro 的方向、单位和量级正确；仍不计划把 ESP32 IMU 与
  MID-360 IMU 同时作为定位主 IMU。
- 木地板、空载条件下，`0.05 m/s` 正反向各 3 次约 1 m 有效重复已完成；六次
  合计候选有效半径约 `0.02189 m`，继续保留 `0.02175 m` 基线。
- 左右约 90°及左右 360°地面转向已完成。有效全转结果为：左转现场/IMU/轮
  编码器约 `353/356.29/361.50°`，右转约 `363/358.10/362.85°`；继续保留
  `0.254 m` 有效轮距基线。
- `0.10 m/s` 正反向各 3 次有效运动已完成。反向合计现场/编码器约
  `3.010/2.95276 m`；前进第 2、3 次现场距离比编码器高约 `9--12%`，与反向
  不一致，已作为外部真值/编码器信号待诊断项，禁止据此直接改轮半径。
- 所有固件刷新后的运动试验均保持 `reconnect_count` 不变，没有再次出现旧固件
  的数秒断流和 watchdog 中途停车；个别 ROS 接收空窗仍达到约 `0.33--0.46 s`。
- 30 分钟静态通信 bag 已完成：`reconnect_count=0`、零 ping 失败、设备采样空窗
  低于 `100 ms`；ROS 接收侧最大空窗约 `345 ms`，并有 6 个完全重复样本。
  固件 session 稳定性通过，接收抖动与重复过滤仍待处理。
- MID-360 已在 Jetson 有线网 `192.168.2.100/24` 上恢复连接，设备地址
  `192.168.2.202`；实测点云约 `10.00 Hz`、IMU 约 `200 Hz`，网口零错误/丢包。
  Livox ROS Driver 2 将加速度以 `g` 原样写入 `sensor_msgs/Imu`，不能直接用于
  通用 ROS 定位。新增 `mid360_imu_adapter` 将 `/livox/imu` 转换为符合 SI 单位的
  `/mid360/imu/data_raw`，并将 orientation 标记为不可用；Jetson 构建和实时
  `199.9 Hz` 验证已通过。
- MID-360 官方机械图确认外壳高 `0.060 m`、坐标原点 O 位于底面上方
  `0.047 m`。结合顶面离地 `0.222 m` 得 O 离地 `0.209 m`；装车点云木地板
  拟合得到约 `0.2066 m`，相差约 `2.4 mm`。参数、URDF 和 Isaac RTX 原点已
  统一修正。实车 M12 插头朝车尾，而官方顶视图中插头位于传感器 `-X` 侧，
  因此传感器 `+X` 朝车头、名义 yaw 为 `0 rad`；最终 XY、亚度级 RPY 与 IMU
  内部杠杆臂仍待精密外参验证。
- Jetson 已安装 `linuxptp 3.1.1`，系统服务 `carbot-mid360-ptp.service` 在
  `enP8p1s0` 上作为唯一软件时间戳 PTP 主时钟并已 enabled/active；网卡无 PHC，
  因此不运行 `phc2sys`。直接读取 MID-360 原始 UDP 协议头，点云和 IMU 各
  `200/200` 包均为 `time_type=1`。同步后 1000 个 IMU 样本无倒退，稳态接收
  延迟中位数约 `0.965 ms`、95% 约 `1.335 ms`。MID-360 到 Jetson 的同步通过，
  但 ESP32 只在 micro-ROS session 建立时同步一次 epoch，之后永久使用固定
  offset。当前轮速接收减 header 中位数已约 `64.84 ms`；历史 30 分钟 bag 的
  逐分钟中位数从 `8.76 ms` 经 `1.98 ms` 变化到 `19.01 ms`。固件必须增加
  保证 header 不倒退的周期重同步，之后再验收与 MID-360 的跨传感器时间。
- Jetson 已部署并启用用户服务 `carbot-mid360.service`，在同一个 systemd
  cgroup 内运行唯一 Livox 驱动和 SI IMU 适配器。主动重启验证中旧 PID 全部
  退出、新 PID 各一个，点云和两个 IMU 话题的发布者数量均为 1；服务当前
  active。该服务设置了 Livox SDK 的运行库路径，解决手动启动时缺少
  `liblivox_lidar_sdk_shared.so` 的问题。用户 linger 当前为 `no`，因此它随
  用户登录启动而非无人登录的系统启动。
- 真机 Livox 驱动已切换为原生 `sensor_msgs/msg/PointCloud2` 输出；话题仍为
  `/livox/lidar`，字段包含 `x/y/z/intensity/tag/line/timestamp`，从而与 Isaac
  Sim、nvblox、pointcloud-to-laserscan 和 KISS-ICP 的输入接口一致。
- Jetson Wi-Fi 已固定为 `192.168.1.109/24`，NetworkManager 连接
  `TP-LINK_652DC7` 已持久设置 `802-11-wireless.powersave=disable`。真机工作站
  环境改用与 Jetson/micro-ROS 一致的 Fast DDS；清除两套残留 nvblox 和两个
  时间探针后，双网卡环境中的 `odom -> livox_frame` 可稳定跨 LAN 接收。
- 原始 PointCloud2 经 Wi-Fi 单订阅仅约 `3.8 Hz`，并会使 TF 接收饿死。新增
  Jetson `mid360_pointcloud_xyz_relay`：仅保留 XYZ，Best Effort 发布，并用轮换
  stride 4 将每帧从约 `20k x 26 B` 降至约 `5k x 12 B`（约 `60 KB`）；本机
  实测 `10.03 Hz`。真机 nvblox 直接订阅 `/mid360/points_xyz`，不使用仿真的
  `1000 x 40` padder。
- 新增 `mid360_nvblox_real.launch.py` 与真机参数：系统时间、`odom` 全局帧、
  `livox_frame` 位姿帧、Sensor Data QoS。真实回波俯仰范围实测约
  `-7.95--+54.04°`，因此真机 FOV 校验边界取 `-10--+55°`；该修改不影响仿真。
  静止真机点云已生成 `152 x 56`、`0.05 m` OccupancyGrid，其中一次采样包含
  4258 unknown、3003 free、1251 occupied，发布约 `9.83 Hz`。持续建图时 TF
  连续可用，30 次 Ping 零丢包、平均约 `36.6 ms`；没有发送 `/cmd_vel`。
- 最新原始记录：`calibration_data/2026-09-17_postflash/README.md` 和
  `calibration_data/2026-09-18_ground/README.md`。

## 当前 Git 状态

- 分支：`codex/carbot-isaac-sim-adaptation`
- 最新已提交状态：`2ad24df feat: capture carbot hardware calibration state`
- 阶段 G 提交：`f8f911d test: complete carbot simulation validation`
- 2026-09-17/18 新增的固件后验证、ground bags、消息定义和标定记录尚未提交。
- `.codex_tmp/` 和 `docs/topology_build/node_modules` 是受保护的用户未跟踪目录，
  不得删除或加入提交。

## 已完成状态

阶段 A～G 的仿真基线已完成：

- 参数单一来源、Carbot URDF/Xacro 和本地 USD articulation 已建立。
- 场景中的唯一活动机器人是 `/World/Carbot`。
- articulation：`/World/Carbot/base_footprint`。
- RTX MID360：
  `/World/Carbot/base_footprint/base_link/lidar_link/mid360_rtx`。
- 旧 `/World/Robot`、Carter ROS2 图和旧 RTX 雷达图已停用。
- Isaac Sim 使用受限的理想履带差速运动学；Isaac Lab 使用相同的
  `[linear.x, angular.z]` 安全边界、加速度限制、耦合饱和和 watchdog。
- sim/real launch、时间源、DDS 和 odom 发布责任已经分开。
- 原 `warehouse_v3`、MID360、nvblox、RViz 和 Nav2 接口保持兼容。

## 已通过验证

- `carbot_description`：42 项测试通过，0 失败、0 跳过。
- Isaac Lab Phase F：8 项契约测试和 20 步环境冒烟测试通过。
- Isaac Lab G1～G7 全部通过：watchdog、正反直行、正负转向、六轮映射、
  耦合饱和、1 m、90°/360°、接触和稳定性。
- MID360、padded cloud 和 `/scan` 实测约 9.82 Hz；padded cloud 为
  `1000 x 40`。
- ROS 类型、QoS、frame、TF、约 50 Hz odom/joint/clock 均通过。
- Nav2 直线和带转向目标均成功，0 次恢复，停止速度为零。
- WebRTC、Isaac Sim、nvblox 和 Nav2 完整启动健康检查通过。
- 详细结果：`docs/carbot_phase_g_validation_report.md`。

## 已确认且不再属于缺失标定阻塞项

- 物理履带中心距：`0.225 m`。
- 有效半径：`0.02175 m`，确认基线。
- 有效轮距：`0.254 m`，确认基线。
- 1 m 和 90°/360° 真车测试仍需执行，但属于发布验收复核，不是重新标定
  前禁止使用的参数。

## 仍然阻塞真机/策略发布的项目

- 长时通信稳定性、ROS 接收空窗和紧凑点云移动建图压力测试；Jetson Wi-Fi
  power-save 已持久关闭。
- 履带侧滑及不同地面/载荷下的变化范围。
- 执行器延迟、死区、制动和左右不对称。
- 质量、重心、惯量和履带摩擦的真机证据。
- IMU 和 MID360 精确外参、Livox 原点 O 以及时间同步。
- 实体 watchdog、独立急停、观察量一致性和真机 AMCL/Nav2 验收。
- 权威清单：`isaac_lab/carbot_env/hardware_calibration_backlog.yaml`。

## 原始下一步计划（架空阶段已完成）

开始前必须关闭仿真 ROS 图，牢固架空两侧履带，并让人员持有实体急停。
不要在 watchdog 和急停验证前发送持续非零速度命令。

建议顺序：

1. 验证实体急停和 500 ms watchdog。
2. 分别低速驱动左右履带，确认方向、映射和编码器符号。
3. 标记轮子，复核 `/wheel_ticks` 50 Hz、重启/boot ID 和每圈 1560 counts。
4. 启动 Jetson 里程计，确认物理 ROS 图中恰好一个 `/odom` 和
   `odom -> base_footprint` 发布者。
5. 测量正反转无负载死区、命令到 RPM 响应延迟、不同档位稳态 RPM、PWM、
   电池电压和左右差异。
6. 静止测量 MID360 原点 O、支架 X/Y/Z/RPY，检查点云地面/墙面方向。
7. 测量 IMU 静止零偏、重力方向、坐标轴和时间同步状态。
8. 架空项目通过后，再制定低速落地的侧滑、制动、1 m 和 90°/360°
   验收计划。

架空阶段建议同步记录：

```text
/cmd_vel
/wheel_ticks
/odom
/tf
/tf_static
/imu/data_raw
左右目标 RPM
左右实际 RPM
左右 PWM
电池电压
```

原始 rosbag/CSV 不要预先平滑，并记录固件提交、Jetson 提交、电池电压、
测试载荷、时间同步模式和实体急停状态。

## 当前运行状态

2026-09-19 已在 Jetson 完成 Isaac ROS 3.2 / nvblox 单机部署和静态建图、
地图保存/加载验收。默认容器名 `carbot-nvblox`，镜像
`carbot-isaac-ros-nvblox:3.2`，使用紧凑点云和 UDP-only Fast DDS profile，
restart policy 为 `unless-stopped`。运行时必须加载提交 `3f77a90` 的源码 overlay，
否则 FilePath 服务会挂起。新增 `jetson_save_maps.sh`，一次保存 Nav2 PGM/YAML、
SLAM Toolbox 位姿图、nvblox `.nvblx` 和 PLY，并对服务调用设置超时和结果校验。
新增 `navigation-safe` 容器模式，强制 `autostart=false`；实测所有 Nav2 lifecycle
节点保持 `unconfigured`，`/cmd_vel` 为零发布者且无消息。

新增 `jetson_nav_preflight.sh` 只读验收。ESP32 关闭时实测系统服务、MID-360
点云、`/scan`、雷达静态 TF 和速度静默状态通过，只按预期报告
`/wheel_ticks`、`/odom`、`odom -> base_footprint` 三项无新数据。测试后
`carbot-nvblox` 已停止。实车 Nav2 速度、footprint、碰撞检测、默认未激活和唯一
最终速度输出路径已增加回归测试；Jetson 部分源码部署中为 4 passed、2 skipped、
0 failed（两项跳过为既有版权检查及缺少完整 `carbot_description` 源码树时的
跨包 footprint 对照）。尚未完成的是更大场地覆盖与几何真值、AMCL 初始位姿
及 Nav2 低速整车运动验收；这些步骤发送非零速度前仍需重新确认安全门禁。

2026-09-19 随后完成 Jetson 移动建图冒烟验收：`0.05 m/s × 3 s` 的里程计
位移约 `0.0937 m`，直行和左右转向期间轮式里程计与 LiDAR SLAM 位姿增量一致。
短脉冲证实 `0.20--0.30 rad/s` 位于原地转向死区附近，`0.40 rad/s` 开始可靠
启动；`±0.50 rad/s` 左右 2 秒分别约 `+21.0/-24.9°`。实车 Nav2 转向范围已从
无效的最大 `0.30 rad/s` 修正为 `0.40--0.50 rad/s`，Jetson 重建和回归测试为
4 passed、2 skipped、0 failed，运行时参数检查通过且节点未激活。移动冒烟地图
的 PGM/YAML、posegraph/data、nvblx 和 PLY 已完整保存到 Jetson
`maps/acceptance/moving_smoke/`；该小范围地图不作为最终导航地图。详细记录见
`calibration_data/2026-09-19_jetson_mapping/README.md`。

同日完成第一版正式地图候选：人工回正后将 odom/SLAM 从当前实际姿态重新置零，
以约 `0.05 m/s`、`0.50 rad/s` 分段走完约 `0.29 × 0.30 m` 小矩形并回到起点。
最终 SLAM闭合姿态约 `(-6.0 cm, -0.9 cm, -0.8°)`；现场观察确认
`map -> odom` 约 `(-4.6 cm, +1.6 cm, -14.4°)` 的方向与履带累计转向误差一致。
六项产物保存为 Jetson `maps/real/carbot_site_20260919.*`，二维 YAML 已通过
`navigation-safe` 的 map_server重新加载，控制节点保持未激活且无速度消息。
该地图是首个正式候选，仍需 AMCL重定位和 Nav2低速闭环验收；小范围路线没有
替代更大场地覆盖与外部几何真值。

最终候选随后通过静态 AMCL验收：`map_server` 与 `amcl` active，初值
`(-6.0 cm, -0.9 cm, -0.8°)` 收敛/输出到约
`(-5.6 cm, -0.9 cm, -0.91°)`，`/scan` 约 `10 Hz`；controller 与
velocity smoother 保持 `unconfigured`，`/cmd_vel` 零发布者。该结果验证静态
定位接口，不替代受控位移后的重定位精度和 Nav2闭环运动验收。测试后
`carbot-nvblox` 已停止。

随后完成首次 Nav2 低速闭环：可靠发布 AMCL 初始位姿后，确认两级代价地图、
导航动作服务与唯一 `/cmd_vel` 输出链路正常，再发送约 `0.20 m` 的前进目标。
动作约 7 秒成功；受 `0.10 m` 位置容差影响，TF 和现场均显示实际前进约
`0.106 m`。现场确认平稳直行、车头无明显偏转并完全停车，结束后 `/cmd_vel`
静默。该结果验证首次直行闭环，但 Nav2 原地转向、倒车恢复和更长路线仍待验收。

原地转向随后改用 Nav2 `/spin` 动作（同位置的纯姿态 `NavigateToPose` 不产生
速度并已安全取消）。原速度平滑器在动作成功后仍缓慢降速，使 `30°` 指令产生约
`5--8°` 超调。当次验收保留 controller/behavior 的 `0.50 rad/s²` 起步限制，同时把
速度平滑器角向跟随限值改为对称 `±2.00 rad/s²` 后，左/右 `30°` 实测分别约
`32.8°/33.9°`，最终回到约 `-1.2°`，现场与 TF 一致且每次结束后速度静默。
动作客户端还增加了 Fast DDS 端点发现后的 5 秒稳定期，避免短生命周期客户端的
目标响应竞态。当前低速直行与左右原地转向闭环通过，倒车恢复和更长路线仍待验收。

同日按总移动量 `1--2 m` 上限尝试一次集成路线。受控建图连续完成
`0.501 m + 左转 90° + 0.300 m`，并成功保存
`maps/real/carbot_site_route_20260919.*` 六项地图产物。新地图上的 AMCL 初值与
建图终点一致，只读规划可生成约 `0.55 m` 的返回路径；但自主返回目标虽被接受、
controller 也持续收到重规划路径，底盘/里程计/TF 均未运动，15 秒后触发
`Failed to make progress`，恢复重试仍无运动，30 秒硬超时后成功取消。未追加
距离或重复实车目标，容器已停止且 `/cmd_vel` 零发布者。后续先离线记录大角度
起始转向时三段速度话题和 progress checker 行为，查明后才允许一次最终复验。

随后在不连接真实 `/cmd_vel` 的 `navigation-diagnostic` 模式复现并录包，根因已
明确：RPP 以 `0.50 rad/s² × 0.05 s` 将每周期转向限制为恒定
`0.025 rad/s`；该值低于履带约 `0.40 rad/s` 的可靠门槛，实测 odom 持续为零，
又使下一周期继续限制在 `0.025`，形成死区闭环。将 RPP 内部
`max_angular_accel` 调为 `10.00 rad/s²` 后，隔离复测上游稳定请求
`0.500 rad/s`，最终 smoother 以 `0.100、0.200……0.500 rad/s` 在约 0.2 秒内
跨过门槛；真实 `/cmd_vel` 在两次诊断中始终零发布者。回归为 4 passed、2 skipped、
0 failed。当前只剩一次约 `0.55 m` 的真实返回复验，不再追加距离或分档测试。

操作者随后手工换位并从新位置重做一次集成路线：连续建图完成
`0.501 m + 左转 90.1° + 0.301 m`，新地图六项产物保存为
`maps/real/carbot_site_route_retry_20260919.*`。AMCL 在新终点建立后，全局规划
生成约 `0.633 m` 的返回路径；应用 RPP 死区修正的唯一真实目标返回
`SUCCEEDED`。最终位置误差约 `5.7 cm`、航向误差约 `9.6°`，均在配置容差内；
但该目标采用路径接近方向 `26.6°`，并非起始方向约 `99.0°`，所以当时只完成了
位置返回。随后以 `/spin` 左转 `82°`，最终 TF 航向约 `99.2°`，与起始方向差
约 `0.2°`；位置因履带原地转向漂移约 `3.4 cm`。至此位置和方向均完成验收。
结束后速度静默，容器已停止且 `/cmd_vel` 零发布者。建图与导航合计约
`1.435 m`，约定的 1--2 m 综合验收完成，不再追加距离测试。

交接记录创建时，`isaac-sim`、`isaac-ros-nvblox` 和 `ros2-dev-humble`
容器仍在运行，RViz2 已退出。开始真机 DDS/Jetson 联调前，必须运行
`/home/shenfq/projects/ros-humble/stop_nav_all.sh`，确认仿真 ROS 图完全关闭。

## 下一会话建议指令

```text
请读取 docs/carbot_2026-09-16_handoff.md、
calibration_data/2026-09-17_postflash/README.md、
calibration_data/2026-09-18_ground/README.md 和
isaac_lab/carbot_env/hardware_calibration_backlog.yaml，继续 Carbot 真机适配。
先确认 30 分钟静态通信 bag 的结果，再安排外部轨迹真值、其他地面/载荷、
MID-360 外参/时间和受控 AMCL/Nav2 验收；发送运动命令前仍需执行安全门禁。
```
