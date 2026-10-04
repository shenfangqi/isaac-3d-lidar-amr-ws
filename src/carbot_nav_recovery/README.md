# carbot_nav_recovery — Issues #9 and #12

当前包含只读诊断、Python 几何与执行协调器，以及 C++ 控制器证据、地图更新证据、有界行为和行为树插件。**默认配置不启用动作；实验执行尚未经过真机验收。**

## 原路回退选项

`PoseHistory` 记录同一导航目标、同一定位 epoch 下实际走过的位置和朝向，保留弯道上的中间点。`evaluate_trace_retreat` 从当前位置沿这些点倒序构造候选，不以直线连接远处的历史位置。

- 定位无效、换目标、时间倒退、采样断档、位姿跳变或非前进轨迹会清空历史；取消、人工接管、抬车和定位重置须由未来的运行适配器显式调用 `invalidate`。
- 历史默认保留 20 秒，最多 512 点，采样间隔不得超过 0.5 秒。
- 每条回退路线连同停车超程，都重新检查当前局部代价地图、静态地图和近期传感器确认的自由空间。旧路线通过过，不代表现在仍然安全。
- 终点至少要有一个方向通过 180° 完整车身旋转检查。这样选出的仅是几何上可转身的候选，仍需验证到原目标的新路径。
- 缺少后方自由空间观测时返回 `NO_OBSERVED_REAR_CLEARANCE`。不能用静态地图自由区域替代 MID-360 近场盲区的观测。
- `recovery_options` 优先提供已检查驶离路径的旋转候选，再提供距离较近的原路回退候选；全部 `motion_eligible=False`。

`RecoveryBudget` 是所有脱困策略共用的单目标账本：最多 2 次、累计移动 0.20 米、累计 30 秒。检查、等待和制动时间均计入；记录实际累计路程而非起终点净位移。一次成功不重置同一目标的预算。调用方需在评估前 `begin`，逐周期 `observe`，确认实际停止后 `finish`。账本本身不控制停车。

## D455 近场证据预留接口

`carbot_recovery_interfaces/msg/NearFieldEvidence` 已作为后续 D455 接入的固定类型化边界，计划话题为 `/carbot_nav_recovery/nearfield_evidence`。当前没有生产节点，协调器也不会消费该话题，恢复执行继续关闭。

- `header.stamp` 必须保留原始深度帧采集时间；网格位于固定导航坐标系，转换必须使用该源时间。
- `observed_space` 只有数值恰好为 `0` 的单元能证明自由。占用、无效深度、遮挡、相机视场外和未更新区域均为未知，不能清空。
- `producer_instance_id` 与单调 `update_sequence` 用于拒绝重启前缓存、倒序和重复证据；`sensor_id`、`sensor_frame` 只用于核对，不能单独授权运动。
- 生产节点必须报告实际距离与碰撞高度过滤范围。接收端还要核对唯一发布者、D455 序列号、TF、新鲜度、深度有效率和 accepted profile。
- 一台朝前安装的 D455 通常不能覆盖车身两侧和后方。融合后仍按每条候选的完整扫掠区域逐格检查；未覆盖方向继续返回 `UNKNOWN_SPACE`，不得用 MID-360 静态地图或插值补齐。
- 接上相机后先只读发布和 RViz 验证，再做软障碍拒绝测试；完成独立物理验收前不得把 `physical_acceptance_complete` 改为 true。

回退速度上限 0.10 m/s、角速度上限 0.50 rad/s，参考值为 0.05 m/s 和 0.40 rad/s。停车距离必须由调用方提供；当前没有真机标定值。候选预计时间不包含运行时所有开销，执行必须服从账本的实际耗时。

## 扫掠检查

`check_swept_path` 检查完整多边形内部及边缘。采样间距限制为顶点最大移动距离不超过半个栅格，并按采样间运动上界补足间隙。朝向使用连续展开角，保留旋转方向和跨越 ±π 的历史轨迹。

未知 255、致命障碍 254、地图外区域均拒绝。253 在机器人中心拒绝，在车身其他位置作为膨胀代价值处理；1–252 也用于排序，避免把膨胀区再次当作实体障碍向外扩张。仍须对照实际 Nav2 配置校验该策略。无驶离路径的旋转候选返回 `NO_DEPARTURE_PATH`。

`SnapshotFreshness` 检查源时间、接收时间、TF 和调用方提供的定位状态。调用方必须保证时间字段确实代表数据更新。Humble 原始 costmap 的 header 是发布时间，不能证明底层传感器或图层已更新；因此预览明确报告 `costmap_update_verified=False`。

## 只读 RViz 预览

实车导航启动参数 `recovery_validation_preview:=true` 可启动可选预览，默认关闭。节点订阅 `/local_costmap/costmap_raw`（Nav2 原始 0–255 代价，不能换成 OccupancyGrid `/costmap`）。

- RViz MarkerArray：`/carbot_nav_recovery/markers`。
- JSON 状态：`/carbot_nav_recovery/status`。
- Trigger 服务：`/carbot_nav_recovery_validation/save_snapshot`，保存最近一次完整诊断快照，默认目录 `/tmp/carbot_recovery`。
- 黄色表示当前旋转几何检查通过，红色表示阻挡；任何颜色都不表示允许运动。标记会过期。
- 预览同时检查 `/scan` 有限光束证明的近期自由空间；costmap 几何通过但扫掠区域落在传感器盲区时显示 `UNKNOWN_SPACE`，不得据此旋转。
- 自动定位状态从 `/automatic_localization/status` 动态读取并检查接收新鲜度；启动参数不再把真机预览永久锁在 `LOCALIZATION_INVALID`。
- 同一开关还启动 `recovery_runtime_observer`，采集当前目标下的临时历史轨迹。Marker `/carbot_nav_recovery/measured_trace` 显示实际轨迹，**不表示整段仍然可回退**。

## 运行状态与历史轨迹

观察器订阅 `NavigateToPose` / `NavigateThroughPoses` 的状态与反馈、`/odom`、`/cmd_vel_nav`、定位状态和 TF。目标 UUID 绑定到轨迹；取消、终止、多目标重叠、换目标、既有 Nav2 recovery、反向指令、定位轮次变化、位姿跳变或数据过期会丢弃历史。它不发送 Action、不修改生命周期，也不发布速度。

- JSON 状态：`/carbot_nav_recovery/runtime_status`。
- Trigger `~/get_trace`：重新检查数据时效后返回目标上下文和实际轨迹点；不足两点时 `success=false`。
- Trigger `~/invalidate_history`：供人工接管等外部事件使轨迹失效。尚未自动对接遥控所有权，不能依靠此观察器取得运动权限。
- 当前仅接受自动定位管理器的新状态格式：含进程 `instance_id`、`source_stamp_ns` 和 `evidence_epoch_ns`。旧版或手工定位链路缺少这些证据时不记录轨迹。
- 有运动指令、里程计仍静止，且持续 3 秒未越过 2 cm / 0.05 rad 的进展阈值时报告 `NO_PROGRESS_OBSERVED`。这是待诊断信号，**不是已确认近障死锁**。正常旋转、没有运动指令或数据中断不会被该规则认定为死锁。
- 所有输出仍为 `trace_provisional=true`、`deadlock_verified=false`、`motion_eligible=false`。Humble 的 FollowPath 空结果不能提供碰撞失败原因；还需控制器/BT 内部结构化证据。

观察器已将当前历史直接接入原路回退几何评估，每 0.5 秒检查一次，单次几何计算预算 0.10 秒。评估前再次检查运行数据时效；地图更换或无效静态地图也会使历史失效。诊断预览不消耗脱困尝试次数。

### 回退预览的输入与输出

- 输入 `/local_costmap/costmap_raw`、`/map`，以及独立的 `/carbot_nav_recovery/observed_free`（OccupancyGrid）。后者只有显式 `0` 单元被视作已观测自由，其他值一律不允许通行；**不得把 `/map` 重映射成此话题**。
- 只读预览使用独立观测输入，缺失该输入时报告 `NO_OBSERVED_REAR_CLEARANCE`。即使外部提供了诊断掩码，仍报告 `visibility_provenance_verified=false`。
- 原路路线、停车超程和终点旋转范围都需要近期自由空间覆盖。
- JSON `/carbot_nav_recovery/retreat_preview` 包含候选轨迹、检查结果、阻挡来源和当前目标上下文。
- RViz MarkerArray `/carbot_nav_recovery/retreat_candidates`：黄色为几何通过，红色为拒绝；标记短时过期，每轮清除旧候选。
- `preview_stopping_distance_m=0.02` 仅为预览假设，输出明确标记 `braking_calibrated=false`；不是车辆实际制动距离。
- 坐标系不一致、旋转地图原点、非法数组或超过一百万单元的网格均拒绝。静态占用图只将 `0` 视作自由，未知保留未知，其余占用概率按阻挡处理。

### 控制器失败证据接口

`ControllerFailureEvidence` 要求原导航上下文、准确的 FollowPath 请求 ID、控制器 ID、结构化原因和源/接收时间。`failure_gate` 仅对 0.5 秒内、同一目标和请求、且已做碰撞检查的 `COLLISION_PREDICTED` 或 `ROTATION_BLOCKED` 放行**评估**。单独 `NO_PROGRESS`、TF 错误、无效路径、旧目标或过期证据不能放行。

Python 参考接口用于离线分析；C++ `EvidenceController` 已在实际碰撞分支输出带路径指纹的类型化证据，行为树携带同一指纹请求执行。只读回退预览仍明确报告 `NO_STRUCTURED_CONTROLLER_FAILURE`。不能通过日志字符串、空 Action 结果或任意 JSON 消息冒充已确认碰撞。此接口不授权运动。

## 实验执行链（默认关闭）

`carbot_recovery_interfaces` 提供类型化协议；`carbot_recovery_plugins` 提供：

- `EvidenceController`：继承 Humble RPP，保留原算法和碰撞检查，仅在明确碰撞分支输出结构化证据。失败绑定目标、定位轮次与输入路径指纹，其他异常不能触发脱困。
- `EvidenceLayer`：放在局部代价地图最后一个图层，在实际 `updateCosts` 回调中输出完整地图、footprint、更新序号和图层状态。不会修改代价。
- `BoundedRecovery`：行为服务器中的 C++ Action，唯一运动出口仍为 `/cmd_vel_nav`。逐周期请求协调器，150 ms 无响应即发零速度；取消、异常、生命周期停止同样发零速度。执行未启用时不创建速度发布器。
- `CarbotBoundedRecovery`：BT 插件携带原目标和失败路径指纹。恢复成功后重新调用规划器规划到原目标，最多两次恢复；失败返回导航失败。

`recovery_coordinator` 复用运行观察器，收到明确碰撞证据后保留当时轨迹。先等待实际静止，再评估双向旋转；旋转不可用时检查短距前后移动与历史原路回退。旋转、平移和原路回退都必须通过当前地图与近期有限光束自由空间检查，候选还需要新规划的驶离路径。执行期间检查更新地图、源时间、扫描、定位、里程计、控制器结束状态和速度话题发布者；实际累计移动量与耗时受同目标预算约束。理想路线与即将输出的速度弧线均进行扫掠检查。

协调器从 `/scan` 源时间 TF 生成保守自由区域。只有相邻有效有限回波之间、完整落在可见区域内的栅格可视作自由；盲区、无效回波和无限距离不清空。当前 MID-360 的 0.5 m 近场盲区可能导致平移和原路回退全部拒绝，这是预期的安全结果，不能用静态地图覆盖它。

可见性只计算单目标 0.20 m 恢复预算及完整车身扫掠所需的局部窗口，窗口外保持未知。Jetson 真机单候选只读测量为 14.4–26.6 ms，满足协调器 100 ms 单周期预算；一次批量显示 8 个候选仍可能超过 100 ms，因此执行器继续逐候选分片。

启动参数 `bounded_recovery_enabled` 默认 `false`。开启还必须给出 `recovery_acceptance_profile` 文件且使用自动定位状态链。配置文件至少包含：

- `physical_acceptance_complete`：必须为 true；没有实测证据不得设置。
- `evidence_directory`：容器内真实存在的绝对目录；用于保存实测记录。
- `braking_distance_m`、`braking_yaw_rad`：包含完整速度链的制动超程。
- `position_margin_m`：定位、跟踪和 footprint 不确定性余量。
- `max_odom_gap_sec`：验收允许的里程计间隔，最多 0.5 秒。

仓库没有提供伪造“已验收”的参数文件，正常启动行为保持不变。

## 复杂弯路速度诊断（Issue #12，默认关闭）

`complex_route_advisor` 对当前全局路径的局部窗口做完整 footprint 扫掠检查，并分别计算曲率速度上限与制动距离速度上限。它只发布建议和证据，不发布 `Twist`、不修改控制器参数，也不会自动重发目标。

维护的一键入口可用下列方式显式启用：

```bash
./start_real_nav.sh --complex-route-validation
```

对应 launch 参数为 `complex_route_validation:=true`，默认 `false`。输出为：

- JSON：`/carbot_nav_recovery/complex_route_advisory`
- RViz MarkerArray：`/carbot_nav_recovery/complex_route_markers`
- Trigger：`/carbot_complex_route_advisor/save_snapshot`
- 快照目录：`/tmp/carbot_complex_route`

没有通过物理验收的制动配置时，节点仍会输出路径曲率和距首个不安全扫掠位置的距离，但顶层原因为 `CALIBRATION_REQUIRED`，且所有数值速度建议为空。配置必须是严格 JSON，只允许以下字段：

```json
{
  "physical_acceptance_complete": true,
  "evidence_directory": "/absolute/path/to/real/evidence",
  "minimum_deceleration_mps2": 0.0,
  "command_latency_sec": 0.0,
  "position_margin_m": 0.0,
  "max_linear_speed_mps": 0.0,
  "max_angular_speed_radps": 0.0
}
```

上面的零值仅表示字段结构，不能加载，也不能作为实车参数。仓库不提供猜测的验收配置。必须用真实底盘、完整速度链和实际载荷测得保守的最小减速度、停车指令延迟、定位/跟踪余量及速度上限，并保存原始证据后，才能设置 `physical_acceptance_complete=true`。这里必须取验收条件下最慢的制动能力，不能填峰值或最佳减速度。

保存一帧一致证据：

```bash
ros2 service call /carbot_complex_route_advisor/save_snapshot \
  std_srvs/srv/Trigger '{}'
```

离线汇总一个或多个快照/JSONL：

```bash
ros2 run carbot_nav_recovery analyze_complex_route_evidence \
  /tmp/carbot_complex_route/advisory_*.json
```

第一阶段只用于确认“弯道曲率、当前速度、完整车身扫掠和预测碰撞”之间的关系。把建议真正接入控制器、以及失败后的原目标自动续航，必须等真机制动参数、代表性复杂路线证据和 Issue #9/#10 的近场安全边界完成验收后再启用。

## 当前验证范围及剩余工作

四个相关包已完成本地构建。最新隔离 ROS 域 73 回归共 77 项通过，覆盖几何、轨迹、预算、证据、传感器盲区、预览、定位管理器回归及 C++ Action 正常完成/取消/超时/非法命令停车。Jetson 的 Nav2 1.1.19/aarch64 核心回归 39 项通过。插件加载检查确认控制器、图层、行为插件和 BT 注册可加载，默认关闭时无速度发布器。Action 故障注入使用合成协调服务和独立 `/issue9_test/cmd_vel`，不接真机。它验证行为协议及停车出口，不等于完整 Nav2+传感器+底盘闭环或实车验收。

Jetson 几何基准使用 80×80、0.05 m 栅格、实车 padded footprint 和 0.02 m 位置余量。最坏单个 90° 候选双地图检查 p99 为 44.6 ms，短移含制动双地图为 14.1 ms，终点单方向 180° 检查为 64.9 ms。8 个旋转候选一次性计算的 p99 为 210.9 ms，因此协调器把每个候选分到独立心跳，单周期保持在 100 ms 预算内。

真机启用前还需要确认 SDK 版本、实际 footprint padding、控制发布者拓扑、取消及下游 watchdog、扫描覆盖、定位抖动、制动与履带死区。若后方无法取得可信近场自由空间，原路回退必须继续禁用。实际速度链的拓扑与物理参数不能由桌面环境推断。
