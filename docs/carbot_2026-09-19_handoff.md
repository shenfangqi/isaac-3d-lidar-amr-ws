# Carbot 会话交接记录（2026-09-19）

本文用于在新会话中继续 Carbot 的 Isaac Sim、Jetson、MID-360、nvblox 与
Nav2 工作。它汇总本会话已经解决的问题、仍有效的参数和证据、当前工作区状态，
以及下一步的优先顺序。

旧记录仍保留于 `docs/carbot_2026-09-16_handoff.md`，但开始新会话时应优先阅读
本文；需要追溯原始测量过程时再看旧记录和 `calibration_data/`。

## 1. 当前结论

- 真机基础适配、轮式里程计参数、MID-360 接入、Jetson 单机 nvblox、建图、
  AMCL 和一条总运动量约 1.435 m 的 Nav2 位置/方向返回验收已经完成。
- 真机与 Isaac Sim 使用同一套已确认的核心几何/运动学基线；仿真不是完整的
  履带物理复刻，地面、载荷、摩擦和侧滑仍需靠真机或外部轨迹真值继续辨识。
- Isaac Sim 中雷达点云只出现在车后方的问题已经修复并验收。
- Isaac Sim 中静态环境也不断改变全局路径的问题已经修复；固定路线已验证每个
  导航目标只生成一次初始 `/plan`，且可以正常到达。
- 上述“稳定全局路径并保留实时避障”的新方案目前只在仿真完成，尚未部署到
  Jetson 真机。真机待办已保存到仓库的 GitHub Issue #1。
- 本轮仿真修复、动态障碍验收与提交前检查已完成；提交号以当前分支 HEAD 和
  GitHub Issue #1 的记录为准。用户目录 `.codex_tmp/` 和
  `docs/topology_build/node_modules` 未纳入提交。

## 2. 安全与当前设备状态

- 本会话结束前，真车因电池报警已经关闭 ESP32，之后处于硬件调整和充电状态。
  本轮后半段只操作 Isaac Sim，没有向真车发送运动命令。
- 真实车辆没有独立标准实体急停。手机网页红色“停止”依赖 ESP32 仍在工作，
  不能视为独立的硬件急停；过去受控测试曾使用外接电源开关作为断电手段。
- 恢复真机测试时，必须先确认 ESP32、雷达和 Jetson 的实际供电状态、现场清空、
  线缆固定、人员在车旁，并确认停止手段立即可用。
- 2026-09-19 23:35（Asia/Tokyo）检查时，以下仿真容器仍在运行：
  `isaac-sim`、`isaac-ros-nvblox`、`ros2-dev-humble`。Nav2、nvblox、RViz、
  两级 costmap 和两个新 Scan filter 节点均存在；3 秒内未观察到 `/cmd_vel`
  消息，未发现持续运动指令。
- 在连接真机 ROS 图前必须先关闭仿真图，避免相同 ROS_DOMAIN_ID 下的
  `/cmd_vel`、TF、odom、点云或 Nav2 节点互相污染。既有关闭脚本位于
  `/home/shenfq/projects/ros-humble/stop_nav_all.sh`。
- 2026-09-20 动态障碍验收和最终软件检查结束后，`isaac-sim`、
  `isaac-ros-nvblox`、`ros2-dev-humble` 均已停止。

## 3. 已确认的真机参数与测量结果

### 3.1 底盘运动学

- 物理履带中心距：`0.225 m`。
- 继续采用的有效轮半径：`0.02175 m`。
- 继续采用的有效轮距：`0.254 m`。
- Nav2 footprint：
  `[[0.155, 0.133], [0.155, -0.133], [-0.130, -0.133], [-0.130, 0.133]]`。
- 木地板、空载条件完成了 `0.05 m/s` 和 `0.10 m/s` 正反向各 3 次约 1 m
  测试，以及左右 90°、左右完整一圈测试。
- 完整一圈的有效结果：
  - 左转现场/ESP32 IMU/轮编码器约 `353/356.29/361.50°`；
  - 右转现场/ESP32 IMU/轮编码器约 `363/358.10/362.85°`。
- `0.05 m/s` 六次直线测试得到候选有效半径约 `0.02189 m`，差异不足以替换
  `0.02175 m` 基线。
- `0.10 m/s` 前进第 2、3 次现场距离比编码器高约 9--12%，但反向不一致，
  更可能包含人工测距、地板/起止线或侧滑误差，禁止据此单独修改轮半径。
- 实车原地转向的可靠死区结论：`0.20--0.30 rad/s` 附近可能不启动，
  `0.40 rad/s` 开始可靠，`0.50 rad/s` 已实际验证。
- RPP 的低速转向卡死根因已经解决：原 `0.50 rad/s² × 0.05 s` 每周期只增加
  `0.025 rad/s`，始终跨不过履带死区。RPP 内部角加速度改为
  `10.00 rad/s²`，最终速度平滑器再用对称 `±2.00 rad/s²` 输出实际斜坡。

### 3.2 ESP32 固件与 IMU

- ESP32 固件提交 `068071e` 已刷写。
- ESP32 仓库 Issues #1/#2/#3 对应的三项问题已修复：
  - 加速度重复做重力换算；
  - 陀螺仪重复做单位换算；
  - Agent ping 过敏导致反复重连。
- Jetson 的 `CarbotStatus.msg` 已同步并重建，新状态字段能正确解码。
- 刷新后静态加速度模长中位数约 `9.8037 m/s²`，gyro 的方向、单位和量级
  已通过左右转及完整一圈验证。
- 30 分钟静态通信：`reconnect_count=0`、零 ping 失败、设备采样空窗小于
  `100 ms`；ROS 接收最大空窗约 `345 ms`，有 6 个完全重复样本。
- ESP32 仓库 Issue #4 的周期时间重同步修改已经完成并刷机，启动正常；仍建议
  后续用长时间 bag 验证 header 单调性及与 MID-360 的跨传感器时间差。
- 定位只使用一个统一的车体主 IMU。计划以 MID-360 IMU 为主；即使 ESP32 IMU
  已修复，也不应同时把两套 IMU 当作定位主观测量直接融合。

### 3.3 MID-360 几何、数据和时间同步

- Jetson 有线地址：`192.168.2.100/24`；MID-360：`192.168.2.202`。
- 点云约 `10 Hz`，IMU 约 `200 Hz`。
- M12 航空插头朝车尾，因此传感器 `+X` 朝车头，名义 yaw 为 `0 rad`。
- 传感器坐标原点 O 的实测安装高度：`0.209 m`；木地板点云拟合约
  `0.2066 m`，相差约 `2.4 mm`。
- Livox ROS Driver 2 的原始 IMU 加速度是 `g`；`mid360_imu_adapter` 已将
  `/livox/imu` 转成 SI 单位 `/mid360/imu/data_raw`，并正确标记 orientation
  不可用。
- 精确 X/Y 与亚度级 RPY、IMU 内部杠杆臂仍没有外部几何真值，属于精调项，
  不是当前启动建图的阻塞项。
- Jetson 已安装 `linuxptp 3.1.1`。`carbot-mid360-ptp.service` 在
  `enP8p1s0` 上作为软件时间戳 PTP 主时钟；网卡无 PHC，所以不运行
  `phc2sys`。
- MID-360 原始包的 `time_type=1`。1000 个 IMU 样本无时间倒退，接收延迟
  中位数约 `0.965 ms`、P95 约 `1.335 ms`。

## 4. Jetson 单机部署与真机验收

- Jetson Wi-Fi 固定为 `192.168.1.109/24`，连接名 `TP-LINK_652DC7`，
  NetworkManager 已持久关闭 powersave。
- 双网卡 DDS 已改用 Fast DDS 并验证。原始完整点云经 Wi-Fi 只能约 `3.8 Hz`
  且会挤压 TF；`mid360_pointcloud_xyz_relay` 用轮换 stride 4 输出
  `/mid360/points_xyz`，约 5k 点、60 KB/帧、`10.03 Hz`。
- `carbot-mid360.service` 管理唯一 Livox 驱动和 IMU 适配器，已验证重启后发布者
  数量均为 1。用户 linger 仍为 `no`，因此它随用户登录启动，不是无人登录启动。
- Jetson nvblox 容器为 `carbot-nvblox`，镜像
  `carbot-isaac-ros-nvblox:3.2`。运行时必须加载提交 `3f77a90` 的源码 overlay，
  否则 FilePath 服务会挂起。
- 真机 nvblox 使用 `/mid360/points_xyz`，不要使用仿真专用的点云 padder。
- `navigation-safe` 强制 `autostart=false` 已验证；`scripts/jetson_nav_preflight.sh`
  可在发送非零速度前做只读检查。
- 已完成静止建图、移动建图、地图保存/重载、AMCL 静态重定位、Nav2 短直线、
  左右 `/spin` 和一条综合返回路线。
- 关键正式地图产物：
  - `maps/real/carbot_site_20260919.*`；
  - `maps/real/carbot_site_route_retry_20260919.*`。
- 最终综合路线：建图运动约 `0.501 m + 左转 90.1° + 0.301 m`；返回规划约
  `0.633 m`，动作 `SUCCEEDED`。位置误差约 `5.7 cm`；追加 `/spin 82°` 后
  航向约 `99.2°`，相对起始 `99.0°` 约差 `0.2°`，原地转向位置漂移约
  `3.4 cm`。
- 综合建图和导航实际运动约 `1.435 m`，已满足双方约定的 1--2 m 验收范围，
  不再重复要求 5 m、10 m 或更多材料/距离的人工测试。

## 5. Isaac Sim 本轮修复

### 5.1 雷达回波只在车头后方

根因不是 MID-360 安装反了。仿真 RTX 代理是 10 Hz 旋转雷达，ROS 发布也是
10 Hz；当 `PublishPointCloud.inputs:fullScan=False` 时，两者发生相位锁定，每帧
都采到近似同一个后向方位，造成前方 `+X` 半球没有点。

修复：

- `isaac_sim/carbot_mid360.py` 的运行配置增加 `full_scan=True`；
- ROS RTX helper 使用 `PublishPointCloud.inputs:fullScan=True`；
- 对应测试和 `isaac_sim/MID360_SIM.md` 已更新。

验收结果：连续 5 帧每帧约 8743 点；前方约 4383、后方约 4360，四象限均有
回波，X 范围约 `-5.94 ... +5.94 m`，验收输出为
`FULL_SCAN_ACCEPTANCE_PASS`。

这项修复只属于 Isaac Sim 的旋转代理发布机制。真机 MID-360 使用 Livox 驱动和
非重复扫描模式，不要把 `fullScan=True` 复制到真机驱动配置。

### 5.2 静态环境中全局路径不停变化

最终架构：

```text
/scan ───────────────────────────────> local costmap
  │                                      （原始数据，立即停车/避障）
  └─> static_map_scan_filter
        └─> /scan_global_static_filtered
              └─> 5-frame temporal median
                    └─> /scan_global_filtered ─> global costmap

NavigateToPose:
初始 ComputePathToPose 一次 -> FollowPath
FollowPath 因持续障碍失败 -> 清两级 costmap -> 等 2 秒 -> 重规划一次
```

原因与实现：

- 仿真墙缘与静态地图之间长期存在约一至两格偏差，只有时间中值滤波无法消除，
  因为它是持续存在的回波。
- 新增 `static_map_scan_filter`，将落在静态占用栅格及 `0.12 m` 外扩遮罩内的
  回波从“全局规划 Scan”移除。
- 再用 `laser_filters` 对全局 Scan 做 5 帧中值；在约 10 Hz 下，新障碍至少
  持续 3 帧、约 0.3 秒后进入全局代价地图。
- local costmap 始终订阅原始 `/scan`，所以近距离停车和实时避障不等待滤波。
- 最终行为树是
  `configs/behavior_trees/navigate_to_pose_replan_after_controller_failure.xml`。
  它使用普通 memory `Sequence`，只在 controller 因持续障碍失败后重规划。

仿真静态路线验收：

- 原点到 `(4.34064, 2.39164)`：成功，恰好 1 条 `/plan`；
- `(4.34, 2.39)` 到 `(-1.35913, 1.99227)`：成功，恰好 1 条 `/plan`；
- RViz 已确认订阅 `/plan`；随后人工发送的多条目标也成功。
- 首个 global filtered scan 的一次记录移除了 `360/361` 个静态墙体回波，
  静态遮罩由 `417 x 424` 地图生成 14982 个遮罩栅格。

不要恢复以下失败方案：

- 仅做 5 帧中值：无法消除持续的地图/Scan 墙缘偏差，问题路线仍约 42 次传递
  新路径。
- `IsPathValid` 作为持续检查：ROS 2 Humble 在窄通道会拒绝 Navfn 刚生成、实际
  可行的路径，重新形成循环。
- `PipelineSequence` 或 `GoalUpdatedController` 版本：会在 `FollowPath` 运行时
  反复 tick 之前的规划节点；一次失败试验产生 3822 条 `/plan`，车辆偏离后目标
  aborted。

## 6. 动态障碍闭环（2026-09-20 补充）

- 已在 Isaac Sim 加入测试专用 `/isaac_sim/dynamic_obstacle` 控制入口，并以真实
  USD cube、RTX Mid-360、原始 `/scan` 和完整 Nav2 链路自动验收。
- 短暂障碍：最高 52 条 Scan ray 命中，local controller 停车；移除后沿原路径
  继续，动作 `SUCCEEDED`，`/plan=1`，结束后 odom 与 `/cmd_vel` 均为零。
- 持续堵路可绕行：最高 61 条 Scan ray 命中，先停车，再受控发布第 2 条路径并
  成功绕行，动作 `SUCCEEDED`，`/plan=2`，结束速度为零。
- 持续堵路不可绕行：四个加厚 cube 在车周围形成封闭环，最高 238 条 Scan ray
  命中；动作 `ABORTED`，`/plan=2`，结束速度为零，没有路径风暴。
- 验收过程发现并修复：
  - RPP 遇到碰撞会立即令 `FollowPath` 失败，因此行为树先等待 2 秒并用原路径
    重试一次；短暂障碍移开后不会重规划。
  - 暂停阶段不得清 local costmap，以免障碍重新标记前产生短暂无障碍窗口。
  - 持续失败只允许一次 global replan；移除最外层 6 轮 spin/wait/backup recovery。
  - Navfn `tolerance` 改为 `0.0`，不接受目标被占据时的部分路径。
- 自动验收脚本：`scripts/validate_dynamic_obstacle.py`。Isaac WebRTC 模式会预建
  四个隐藏的测试 cube；正常运行时它们位于场景下方，不影响点云或导航。

## 6.1 尚未覆盖的内容

- 当前 5 帧和 `0.12 m` 是仿真值，不是可直接宣称适合真机的最终参数。
- 真机配置 `configs/nav2_params_real.yaml` 尚未接入本轮行为树与全局 Scan 链路。
- Jetson 可能需要安装 `ros-humble-laser-filters`，并重新构建
  `isaac_3d_lidar_bringup`。
- 真机要根据 MID-360 的噪声、运动畸变和实际地图分辨率重新决定静态遮罩半径
  与时间窗口；local costmap 必须继续保留原始低延迟 Scan。
- 真机部署和验收清单已写入：
  <https://github.com/shenfangqi/isaac-3d-lidar-amr-ws/issues/1>
  （标题：`[真机适配] 稳定全局路径并保留 MID-360 实时避障与堵路改道`）。

## 7. 本轮 Git 提交范围

- 分支：`codex/carbot-isaac-sim-adaptation`。
- 本轮提交基于 `a823511 feat: finalize Carbot real robot adaptation`；最终提交号
  见当前分支 HEAD 与 GitHub Issue #1。
- `git diff --check` 已通过。
- 已修改：
  - `configs/carbot/common.yaml`
  - `configs/nav2_params_sim.yaml`
  - `docs/map_nav/README.md`
  - `isaac_sim/MID360_SIM.md`
  - `isaac_sim/carbot_mid360.py`
  - `isaac_sim/streaming_carbot.py`
  - `isaac_sim/usd/carbot.usd`
  - `isaac_sim/usd/configuration/carbot_base.usd`
  - `launch/carbot_navigation.py`
  - `src/carbot_description/test/test_carbot_mid360.py`
  - `src/carbot_description/test/test_carbot_phase_e.py`
  - `src/isaac_3d_lidar_bringup/package.xml`
  - `src/isaac_3d_lidar_bringup/setup.py`
- 新增且应纳入本次提交：
  - `configs/behavior_trees/navigate_to_pose_replan_after_controller_failure.xml`
  - `configs/laser_filters_global_sim.yaml`
  - `scripts/validate_dynamic_obstacle.py`
  - `src/isaac_3d_lidar_bringup/isaac_3d_lidar_bringup/static_map_scan_filter.py`
  - `src/isaac_3d_lidar_bringup/test/test_static_map_scan_filter.py`
  - 本交接文档。
- 用户目录，禁止删除或提交：
  - `.codex_tmp/`
  - `docs/topology_build/node_modules`
- 两个 USD 二进制文件是本轮 canonical alignment/rebuild 产生的有效修改，不要在
  未检查内容来源前随意回滚。

## 8. 已完成的软件验证

- `src/isaac_3d_lidar_bringup/test/test_static_map_scan_filter.py` 与
  `src/carbot_description/test` 合计最近一次结果：`34 passed, 8 skipped`。
- 新行为树 XML 可解析。
- Python/flake8 检查通过。
- `git diff --check` 通过。
- `ISAAC_WEBRTC=1` 的全栈启动健康检查通过。
- 仿真雷达前后半球覆盖验收通过。
- 两条代表性静态导航路线均成功且每个目标只发布一次初始全局路径。
- 三类动态障碍闭环验收通过：短暂障碍 `/plan=1`、持续可绕 `/plan=2`、持续
  不可绕 `/plan=2` 且安全 `ABORTED`。

新会话开始时仍应重新跑一次与提交范围相符的测试，并先检查工作区，避免把运行中
生成的文件误纳入提交。

## 9. 下一步优先顺序

### P0：当前仿真改动（已完成）

1. 动态障碍三类闭环已完成。
2. 单元测试、XML 解析、flake8 和 `git diff --check` 已通过。
3. 提交范围已检查，用户目录未纳入。
4. 提交号和动态障碍结果已补充到 GitHub Issue #1。

### P1：真车恢复供电后的部署

1. 先关闭全部仿真 ROS 图，再执行 Jetson preflight 和安全门禁；不要直接发送目标。
2. 在 Jetson 安装/确认 `laser_filters`，同步新节点、入口点、行为树和依赖后重建。
3. 为 `configs/nav2_params_real.yaml` 增加独立的真机全局 Scan 链路；不要让 local
   costmap 使用滤波后 Scan。
4. 静止时先验证话题频率、TF、QoS、静态墙体遮罩和 CPU/内存负载。
5. 再以一次受控短路线依次验收：无遮挡、临时障碍、持续堵路可绕行、持续堵路
   无法绕行。每一类先做一次，不展开无限重复或增加路线长度。
6. 将真机窗口和遮罩参数、验收 bag 及结果回写 Issue #1 和本文后续交接。

### P2：不阻塞当前功能的后续精调

- 用 LiDAR SLAM、视觉或其他外部轨迹真值辨识不同地面/载荷下的完整侧滑模型。
- 精确测 MID-360 X/Y/亚度级 RPY 和 IMU 杠杆臂。
- 验证 ESP32 周期时间重同步刷机后的长时 header 单调性和跨传感器时间误差。
- 补齐质量、重心、惯量、履带摩擦、执行器延迟、制动和左右不对称真值。
- 最终自主运行前增加真正独立于 ESP32/网络的软件与硬件急停链路。

## 10. 新会话可直接粘贴的指令

```text
请先完整读取
/home/shenfq/projects/ros-humble/isaac_3d_lidar_amr_ws/docs/carbot_2026-09-19_handoff.md，
再检查 git status 和当前容器/ROS 图，不要清理或回滚现有未提交改动，也不要处理
.codex_tmp/ 与 docs/topology_build/node_modules。

继续 Carbot 工作：P0 仿真修复、三类动态障碍闭环、测试、提交与 Issue #1 更新
均已完成。下一步按 P1 准备真机部署，但真车目前可能仍在充电/调整；除非我重新
明确确认供电、现场清空、线缆固定和停止手段，否则不要启动真机 ROS 图、部署会
触发运动的配置，或向真车发送任何运动命令。获得确认后先关闭仿真 ROS 图，再做
Jetson 只读 preflight、依赖安装、构建和静止话题/TF/QoS/负载检查。
```
