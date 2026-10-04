# Issue #9：有界脱困交接

## 当前状态

代码位于三个新包：`carbot_nav_recovery`（Python）、`carbot_recovery_interfaces`（接口）、`carbot_recovery_plugins`（C++ 插件）。实车 bringup 仅增加默认关闭的开关；没有向真机部署或发送运动命令。

包含双向旋转、短距前后平移、历史轨迹原路回退；候选检查完整 footprint、当前地图、制动余量和驶离路径。历史回退保留弯道中间点。恢复成功由行为树重新规划到原目标。

**当前已通过本地构建、77 项回归和 C++ 插件加载检查；尚未通过实车验收。** 回归在回环 DDS 域 73 运行，Action 使用合成协调服务和 `/issue9_test/cmd_vel`。覆盖正常完成、取消、协调器超时、非法速度全部清零；不代表完整 Nav2+传感器+底盘闭环已验证。

### 2026-10-03 收口状态

- 本地干净 overlay 回归更新为 77 项通过；Jetson Nav2 1.1.19/aarch64 独立目录 `/tmp/issue9-target-build-v3-20261003` 构建成功，核心 39 项通过，插件 smoke 通过。运行安装层未覆盖，恢复执行仍关闭。
- 实车开阔区域完成 15 次实际起步并停稳：正向 3、反向 3、左转 4、右转 4、smoother watchdog 1。7 个安全门拒绝均在起步前，另有一个空参数拒绝文件。统一解析入口为 `scripts/analyze_issue9_motion_evidence.py`。
- 车载里程计观测的最差停后平移为 0.01683 m，最差停后余转为 0.06634 rad。watchdog 断流后 `/cmd_vel_command` 首个零速为 664.1 ms，`/cmd_vel` 为 666.6 ms。这些不是外部尺量，不能将 `physical_acceptance_complete` 改为 true。
- 修正了 padded footprint 真机必拒、behavior_server 多 DDS endpoint 拓扑、扫描源时间 TF 健康门、定位急停快速退出、人工定位等待时限、重复恢复计数和候选计算分片。
- Jetson 几何基准：单个最坏旋转双地图 p99 44.6 ms，短移含制动 14.1 ms，终点单方向 180° 检查 64.9 ms；均低于单周期 100 ms。8 方向整批 p99 210.9 ms，因此禁止单周期批量计算。
- 最终只读审计 `20261003_final_readonly_audit.json`：定位 `READY`、Nav2 active、scan map score 0.775、无速度消息、TF 正常。普通导航仍使用原执行链；实验包未部署。
- 下一步必须由现场人员布置外部尺量标记和可移开的软障碍。完成外部制动距离/余转、左右与短移后旋转场景各 3 次、取消/失联边界和无解拒绝验收后，才能生成有效 acceptance profile 并进行默认关闭的完整 BT 闭环试验。

### 2026-10-04 实车续测

- 当前分支已快进到 `origin/main` 的 PR #8 合并提交 `ceb8e6b`，Issue #9 未提交改动完整保留。普通 `./start_real_nav.sh` 现在默认执行受保护的自动全局定位并在通过后激活 Nav2；人工定位保留为显式 `--manual` 恢复选项。
- 自动定位在重启后连续成功，最近一次全局最佳分数 0.869、次优 0.525，最终状态 `READY`。没有发送导航目标。
- 找到扫描空窗根因链：`/fast_lio/cloud_registered_body` 连续，但标成动态 `fast_lio_imu` 后，投影依赖的可靠 `/tf` 曾成批延迟 2.84 秒，`/scan` 最大空窗 2.54 秒。将两个投影器队列由 30 收紧为 3，并把本体点云按其真实物理原点标为 `imu_link`，使投影只依赖 URDF 静态 `imu_link -> base_footprint` 标定。部署后 20 秒样本中 `/scan` 最大到达间隔降至 0.395 秒。运动门仍拒绝源时间年龄超过 0.5 秒的样本。
- 外部直线量测：前进脉冲最终位移 35 mm（里程计 34.77 mm）；等参数回退后停在原胶带前方 2 mm，外部回退约 33 mm。证据及操作员原话在 `calibration_data/issue9_physical/20261003_external_measurements.json`。
- 实际脱困角速度 `0.40 rad/s`、0.4 秒的单次旋转已双向完成并稳定停车。外部观察右转约 4°（里程计 4.58°），左转约 5°（里程计 5.45°）。低速 `0.16 rad/s` 右转落入履带死区，轮增量 `+107/-20`；实际脱困速度下轮增量恢复近似对称，不能用低速微脉冲结果标定恢复旋转。
- 用户在 RViz 指定并到达开阔区域参考目标后，完成 3 轮 `+0.05 m/s × 1 s`、停车、`-0.05 m/s × 1 s`、停车的往返复测。相对初始地图位姿的返回误差依次为 2.16/3.92/7.00 mm，朝向误差绝对值为 0.36/0.55/0.70°，3/3 满足本次复测的 10 mm、1°判据。位姿均取 20 个定位样本的中值，原始日志及计算结果位于 `calibration_data/issue9_physical/20261004_reference_roundtrip_summary.json`。该结果支持低速前后运动的重复性，不替代独立外部制动距离测量。
- 往返后的 10 秒只读审计仍收到 85 帧 `/scan`、94 帧 `/odom`，自动定位状态为 `READY` 且 Nav2 已激活。最新 costmap 时间戳的精确 TF 查询因约 6.8 ms 的未来外推差失败；每次运动前后的实时安全门均通过。原始审计为 `calibration_data/issue9_physical/20261004_post_roundtrip_runtime_audit.json`。
- 使用同一实体标记完成 3 轮右转再左转复位。用户目视确认位置回到原点、最终车头向左约 1°；地图定位最终误差 10.46 mm、1.15°，与目视方向一致。该组未通过 10 mm、1°严格判据，旋转制动余量不能省略。
- 修复只读预览未订阅自动定位状态、真机永久显示 `LOCALIZATION_INVALID` 的问题。预览现在读取新鲜 `READY` 状态，并把 `/scan` 有限光束自由空间加入每个旋转扫掠结果。
- 发现并修复执行协调器的安全缺口：旋转候选和每周期命令此前只检查 costmap，现与平移和原路回退一样要求近期传感器自由空间。可见性计算限制在 0.20 m 恢复预算覆盖的局部窗口，窗口外保持未知；Jetson 单候选只读耗时 14.4–26.6 ms。
- 真机开阔地证实：8 个双向旋转候选的 costmap 几何全部为 `OK`，有限 `/scan` 证据全部为 `UNKNOWN_SPACE`。MID-360 的 0.5 m 近场盲区覆盖车身旋转扫掠范围；在增加近场观测前，软障碍运动验收和恢复执行必须继续关闭。证据为 `20261004_open_space_sensor_rotation_preview_summary.json` 与 `20261004_open_space_sensor_rotation_preview_roi_single_summary.json`。
- 后续 Intel RealSense D455 接入、部分视场融合和软障碍续验收由 GitHub Issue #10 跟踪：`https://github.com/shenfangqi/isaac-3d-lidar-amr-ws/issues/10`。接口已预留为 `carbot_recovery_interfaces/msg/NearFieldEvidence.msg`，计划话题 `/carbot_nav_recovery/nearfield_evidence`；相机接入前协调器不消费该话题。
- 恢复执行和 acceptance profile 仍保持关闭/无效。外部正反向与左右制动量测尚未达到每方向 3 次，软障碍选择、短移后旋转、无解/动态失效和原目标闭环仍未验收。

## 本轮修复

- 异常服务响应强制清零，禁止输出部分计算完成的速度。
- 取消、失联和失效同时撤销未完成的规划请求，包括延迟到达的规划接受响应。
- C++ 行为等待协调器最多 150 ms；超时、取消和生命周期停止发送零速度。成功必须完成停止握手。
- 同一原目标累计最多两次、0.20 m、30 s，运动前额外预留通信/制动额度。
- 检查理想路径及实际输出速度形成的弧线；对 ±π 朝向跳变进行连续展开。
- 光束覆盖检查包含栅格角跨度内的全部回波；不能仅凭四个角点清空中间未知区域。
- 只读采集器复用参数客户端，避免反复发现未启动服务造成客户端累积。

## 真机只读阶段

车辆先放在开阔平地，保持遥控未解锁、不给导航目标。此阶段不是狭窄处脱困测试。

1. 确认 Jetson 上实际 Nav2/RPP 版本。开发容器是 1.1.20，之前远端记录为 1.1.19；部署前必须重新核实并在目标 SDK 上构建。
2. 使用 `scripts/carbot_recovery_runtime_audit.py` 在已有 ROS 环境只读采集。它不会发速度、目标、参数写入或生命周期请求。
3. 核实实测 footprint 加 padding、`/scan` 最小量程、源时间 TF、真实地图更新频率与全部速度发布者。
4. 保留原始 JSON、bag、软件版本和配置散列。无法确认数据来源或更新时间时，不启动新行为。

## 现场标定与运动验收

### 2026-10-03 现场进展

#### 再次继续后的最新状态

目标SDK验证已完成：三个新包在Jetson `carbot-nvblox`内、独立目录`/tmp/issue9-target-build-v2-20261003`按实际Nav2 1.1.19和aarch64构建成功；隔离ROS域73的`recovery_plugin_smoke`通过（controller/layer/behavior/BT均加载，禁用生命周期无速度发布）。没有覆盖`/workspaces/isaac_ros-dev/install`，没有启用恢复行为。

目标核对修正两项真机必拒问题：协调器现在按Nav2规则使用加1cm padding的costmap footprint（默认由`[0.155,0.133]...`变为`[0.165,0.143]...`）；命令拓扑允许`behavior_server`为多个行为创建同名DDS发布端点，同时仍要求smoother和compensator各自恰好一个发布者。修正后本地75项回归全部通过。

已完成前进脉冲细化：命令窗口1.035秒，`/cmd_vel_command`和`/cmd_vel`在停止请求后最后非零分别约171.6/172.9ms；FAST-LIO净前移约4.27cm，停止请求后净纵向约1.41cm，最后明显线速度约263.8ms后消失。轮tick增量左436/右446。以上不是外部真值，不能据此填写physical acceptance profile；仍需反向、小角度旋转及外部量测。

AMCL `transform_tolerance` 动态对照已完成并撤回：0.5秒样本稳定段252帧中map TF成功240帧（95.24%）；1.5秒样本235帧中成功219帧（93.19%）。1.5秒未改善本次静止样本，真机参数已确认恢复0.5秒。不要依据工作区旧注释把1.5秒直接部署。TF过滤器的“earlier than all data”日志对多类future/timeout异常使用同一文本，不能单凭日志文字判定时钟倒退。

连接恢复后的维护健康检查确认RViz未运行；直接生命周期检查确认`map_server=unconfigured`。当前按安全的非导航状态处理，Web与command compensation服务active。Nav2进程存在不等于生命周期active；不得复用之前READY结论。下一步可在独立临时目录进行目标SDK编译，不覆盖真机install、不启用恢复执行。

用户已确认“有搬过小车”。本轮速度异常可能由搬动触发，不能据此认定估计器自身跳变。已通过维护脚本再次启动静止人工定位会话；预检通过：150样本、10 Hz、位置范围0.007825 m、yaw范围0.126680度。12秒只读诊断91帧源时间odom TF全部通过，扫描到达延迟最大0.589秒；未定位时map TF不存在属预期。证据`20261003_tf_after_moved_restart.json`。目前继续等待此会话进入/完成第7步人工定位；不得重复启动。

同时修正`jetson_navigation_wait.py`：订阅持久化定位急停，收到true立即以代码4失败而非等待传感器超时。域73隔离实际ROS进程测试验证通过；16项既有安全回归仍通过。已更新Jetson临时helper。未更改定位算法、速度保护阈值或AMCL配置。

重新运行维护启动（仍为静止人工定位模式），预检通过，但随后真机适配器在 ROS 时间 `1790993793.348` 报 `LOCALIZATION FAULT: planar speed 0.502 m/s` 并锁止。本轮无非零命令。读取已部署适配器确认：锁止后停止发布里程计/TF，随后点云投影队列积压。故本轮队列故障是定位锁止的后果；上次 costmap 丢帧的根因尚未证实相同。已终止启动并运行维护停止，禁止绕过锁止。已询问用户本轮是否推/搬/碰车，待回复；当前不用再进行人工定位。

新增只读 `scripts/carbot_scan_tf_audit.py`。故障前采到152帧源时间 odom TF全部通过，尚未人工定位因此map TF不存在；原始扫描到达延迟最大2.30秒。证据 `20261003_tf_restart_before_pose.json`、`20261003_restart_localization_fault.log`。真机AMCL配置0.5秒，本地配置1.5秒，有部署差异，但尚不能认定是故障原因，未修改真机AMCL。

修改 `scripts/jetson_navigation_control.py` 的扫描健康检查：必须取得5帧源时间新鲜且map TF可用的扫描，以前仅收到一帧即通过。仅语法检查和16项现有安全回归通过，不能当作新增逻辑完整动态测试。修改已复制到Jetson临时helper，未部署其他恢复代码。真机适配器含本地尚无的锁止保护，禁止全量覆盖部署。

**最新覆盖记录：** 用户确认现场无障碍并提供 RViz 截图。不能把近点读数称为已确认的实体障碍。原始点云经源时间 TF 对齐的 78 帧里有 42 个近点，车体坐标约 x=-0.074..-0.063、y=-0.540..-0.497、z=0.101..0.349 m，来源尚未确定。之前把自动定位整圈旋转的 0.55 m 全周门槛套到短直线标定不合理，已向用户更正。

监督标定改用 0.40 m 外包络检查（车体含 padding 外接半径 <0.22 m、单次直线 <=0.05 m、预留0.13 m）。扫描盲区为0.5 m，该检查不能证明近场无物，必须依靠本轮明确的现场清空确认；严禁迁移到自主脱困验收，也未修改自动定位的0.55 m保护。

已完成一次0.05 m/s、1 s前进脉冲，`PULSE_COMPLETE, settled=true`，LiDAR里程计净位移约4.3 cm，未发生故障；不能替代外部尺量。后退三次均在运动前拒绝（2 Hz状态消息的阈值适配、DDS发现、扫描时间新鲜度），无后退及旋转动作。记录在`20261003_forward_pulse_02.jsonl`及`20261003_reverse_pulse_*.jsonl`。

随后零速度复查仍发现扫描新鲜度失败，容器日志显示局部/全局costmap持续因TF缓存时间关系丢弃扫描，见`20261003_costmap_tf_drop.log`。已执行维护停止脚本，确认Nav2/RViz关闭、Web恢复为未解锁；不得假定仍为READY。根因尚未定论，下一步先诊断AMCL/map->odom与扫描时间链，禁止继续非零试验。标定脚本新增源时间scan->costmap TF检查，尚未真机复验。状态消息阈值改为0.75 s（实测2 Hz），扫描和里程计仍0.5 s，DDS启动发现等待8 s。

- 用户明确确认现场看护、急停可用，并授权短距离前后移动和小角度旋转标定；无需重复索要这项授权。
- 静止验证曾达到 `CANDIDATE_READY`，但该模式持续发布零速度且要求静止，不能直接叠加标定速度。
- 已用维护脚本停止该会话，再运行 `./start_real_nav.sh --stationary-validation --automatic-activate`。该组合使用 `prepare_stationary`，不执行自动整圈旋转；人工定位通过后才激活 Nav2。
- 新会话硬件预检通过，FAST-LIO 141 个样本、10 Hz、静止位置范围 0.008374 m、朝向范围 0.150888 度；定位急停为 false，速度输出静默。
- 当前到第 7/8 步，启动进程正在等待 RViz `2D Pose Estimate`。已请用户重新标真实位置/朝向，不发导航目标。没有发送任何非零运动指令。
- 恢复时先查看仍在等待的启动进程及实时状态，不要重复启动。该等待有超时，超时失败由维护脚本清理；不能假定 Nav2 已激活。
- 切换前只读证据保存在 `calibration_data/issue9_preflight/20261003_before_calibration_transition.json`。旧探针直接发布 `/cmd_vel`，不能用于宣称完整速度链的制动验收；后续标定仍需建立有界运动与记录流程。
- 用户随后完成定位；实时状态已达到 `READY`、`navigation_activated=true`、`validation_only=false`，自动旋转累计为 0。以上“等待人工定位”阶段已结束。
- 新增 `scripts/carbot_bounded_ground_pulse.py`：在 Jetson 本地计时，经过 `/cmd_vel_nav -> smoother -> compensation -> /cmd_vel`，单次最多 1 秒、0.05 m/s 或 0.20 rad/s。检查定位、急停、活跃 Action、命令所有权、扫描/里程计新鲜度和行程上限，finally 显式清零并观测停止；不会解除锁止。仅适用于明确授权的现场监督标定，不是自主避障器。
- 第一次仅零速度预演被 0.55 m 开阔区域门槛拒绝，`started=false, settled=true`，没有任何非零命令。连续扫描最近距离 0.500～0.652 m，后续快照最近点为 0.535 m、方位 -98 度（车体右侧略偏后）。已请现场人员检查/清开该侧近物，保持断电开关可触及；等待回复后再读扫描，不得放宽门槛。
- 原始证据：`calibration_data/issue9_preflight/20261003_zero_pulse.jsonl` 和 `20261003_motion_gate.json`。本轮没有取得运动或制动标定数据。新脚本通过语法解析及真机零速度预演，非零路径尚未验证。

要进入这一阶段，必须有现场人员看护及可靠急停。需要分别测量正反方向的低速响应、履带死区、完整速度链制动距离/角度、定位/跟踪误差、取消与下游 watchdog 停止行为。先开阔区域，再使用可移开的软障碍。

`config/acceptance_pending.json` 故意是不可用模板：验收标志为 false，物理值为空。禁止填写猜测值并改为 true。即使文件通过格式检查，它也不是自动生成的安全证明。

MID-360 近场盲区可能让所有平移和回退候选因缺少自由空间证据被拒绝。此时应保留拒绝状态；不能把静态地图自由单元当作后方传感器证据。若现有传感器无法覆盖需要的区域，需要另行解决近场观测。

## 实验范围

实验执行只接受 `NavigateToPose`，不接受多航点 `NavigateThroughPoses`。默认关闭。不要把实验 overlay 替换为普通导航默认配置。控制器补丁基于 Apache-2.0 的 Navigation2 1.1.20 RPP `computeVelocityCommands`，保留原版权声明。

启用新执行器的必要条件包括：目标 SDK 构建和运行验证、真实 footprint 一致、可信传感器覆盖、实测物理参数、控制权和停车验收。当前交接记录不宣称这些条件已经满足。

## 本地验证复现

在开发容器内，先 source `/opt/ros/humble/setup.bash` 和 `/tmp/codex-issue9-install/setup.bash`，工作目录为挂载的本仓库。使用 `ROS_DOMAIN_ID=73`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 和项目 `cyclonedds_ros_local.xml`。不要同时设置 `ROS_LOCALHOST_ONLY=1`，该 CycloneDDS 配置已绑定 lo，重复选择会导致节点创建失败。

回归入口为 `src/carbot_nav_recovery/test`、`src/isaac_3d_lidar_bringup/test/test_real_nav_safety.py` 和 `src/isaac_3d_lidar_bringup/test/test_automatic_localization_manager.py`。插件检查入口为 `ros2 run carbot_recovery_plugins recovery_plugin_smoke`。必须保持域 73 和测试输出话题隔离；这些程序不应运行在实车 DDS 域中。
