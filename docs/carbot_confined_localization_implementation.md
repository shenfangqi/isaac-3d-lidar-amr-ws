# MID-360 狭小可旋转空间自动定位：实施规格

版本：2026-10-05，设计状态：待实施/待验收。

代码基线：`2a51cd01fd582bc6430898ed7f88e9fb848d9af6`。本次编写时工作区无已有未提交改动。本文件中的“新增”接口、状态、参数和命令均是待实现规格，不是现有功能；不得据此直接操作实车。

## 1. 范围和交付定义

使用现有 MID-360、FAST-LIO、履带差速底盘、保存的二维地图；不增加 D455、测距阵列，不依赖 Issue #10。场景为实体车身可旋转但附近有墙/家具，当前 0.55 m 全周距离门槛可能拒绝启动。第一版只允许原地旋转，不执行平移或 Issue #9 脱困。

用户不承担批量尺量工作：读取已有标定/模型、自动采集和计算；仅需确认无新增突出物、在授权运动测试时看护、少量目视/视频交叉确认。禁止将车载估计精度直接写成外部真值精度。

交付两种明确能力：

1. `stationary_only`：完全不动的自动全图定位，无近场运动依赖。
2. `segmented_rotation`：静止定位不足时，证据充分才分段旋转并重新验证。

不承诺“所有实体可旋转位置都能无人确认启动”：传感器盲区、重复环境、地图错误可能使软件无法证明安全或定位唯一。缺少证据时输出具体拒绝原因，不能偷偷恢复旧距离门槛绕过拒绝。第一版不实现人工确认盲区的长期豁免开关。

## 2. 当前代码事实与改动入口

路径简写：`B=src/isaac_3d_lidar_bringup`，`P=B/isaac_3d_lidar_bringup`，`R=src/carbot_nav_recovery/carbot_nav_recovery`。简写只用于本文件。

| 当前位置 | 已核对行为 | 实施改动 |
| --- | --- | --- |
| `P/automatic_localization_manager.py::_tick` | START_LOCALIZATION 后通常进 ROTATE_AND_SCORE，再静止全图搜索 | 按策略分流；新策略先 COLLECT_STATIC |
| 同文件 `_on_prepare_request` | prepare_stationary 进入人工定位路径 | 保持兼容，不复用成新自动模式 |
| `_on_scan` | 只有 SEARCH_GLOBAL_POSE 收集静止搜索帧 | 改为采集带源时间位姿的 Keyframe，区分训练/验证 |
| `_on_safety_scan` | 汇总有限光束数量和最近距离 | 保留旧策略；新策略调用旋转安全门 |
| `_on_odom` | 按固定方向累计正向 yaw 增量 | 新策略分开记录有符号段进度、净位姿变化及绝对运动预算 |
| `_start_global_search` | 异步调用确定性全图搜索 | 增加 session/map generation、截止时间和多视角输入 |
| `_accept_global_search` | runner-up 主要以 0.75 m 位置距离区分；候选直接生成 AMCL seed | 分离候选选择、歧义判定和种子发布；加入 yaw 和退化检查 |
| `_quality_passes`、`_candidate_quality_passes` | 扫描、粒子、TF 与漂移校验 | 保留并增加独立留出帧门槛；不得用紧 seed 协方差证明唯一性 |
| `_publish_rotation` | 发布到配置的 cmd_vel_topic，当前 YAML 为 /cmd_vel_command | 新策略由独立 watchdog guard 发同一上游话题，manager 不竞争发布 |
| `P/automatic_localization_quality.py::deterministic_global_search` | 多帧用同一候选变换评分；默认 fine_seed_count=1 | 新增多视角版本；旧函数保留兼容包装 |
| 同文件 `scan_map_metrics` | 已有端点、覆盖、自由射线穿墙检查 | 复用评分，增加 per-view 聚合和方向可观测性，不重新实现第二套评分 |
| `R/swept_footprint.py` | 已有 check_swept_path/check_observed_free_path 和半栅格插值 | 复用纯几何接口；不调用要求已定位的 recovery coordinator |
| `R/sensor_visibility.py::scan_visibility` | 无效光束/近场盲区保留未知 | 保持语义；不得将 range_min 内栅格标 free |

启动脚本有 `--automatic` / `--automatic-activate` / `--stationary-validation`；其中 validation-only 并不等于禁止运动。新设计必须用独立 `motion_policy` 禁止运动，不改变这些已有参数的含义。

## 3. 模块与依赖

新增以下纯逻辑模块到 P：

- `localization_observations.py`：Keyframe、源时间对齐、不可变观测窗口。
- `localization_hypotheses.py`：多视角候选搜索、聚类、歧义和留出验证。
- `localization_rotation_policy.py`：旋转候选选择、进度和预算（不发布 ROS）。
- `localization_motion_guard.py`：独立 ROS 进程，唯一启动阶段速度执行者；检查新鲜度/扫掠/超时，独立于搜索存活。
- `localization_contracts.py`：数据模型、JSON 编解码、schema 验证。

依赖现有 carbot_nav_recovery 的纯几何库，确认 package.xml 声明单向依赖；不得让 recovery 反向依赖 bringup，避免循环。不导入 coordinator 或其 READY 前置条件。setup.py 注册 guard 入口；launch 仅在新策略时启动 guard，旧策略保持原行为。

## 4. 数据与接口契约

### 4.1 输入

| 输入 | 类型/坐标 | 规则 |
| --- | --- | --- |
| /scan_localization | LaserScan，使用实际 header.frame_id | 用于匹配，不用于全部车体避障；保留当前定位高度带 |
| /scan | LaserScan | 全导航高度安全证据；无效/无限值不能清空未知 |
| /fast_lio/cloud_registered_body | PointCloud2，按实际 header 与已验证外参 | 第一阶段只做覆盖审计；没有经验证的 3D 自由空间模型前只提供障碍存在证据，不凭稀疏点之间空白判 free |
| /livox/lidar | 启动审计识别实际消息类型 | 只读审计原始有效回波、tag、源时间；不假定始终为 PointCloud2，兼容记录的 CustomMsg |
| /odom、/fast_lio/imu_odom | Odometry | 前者局部运动/TF一致性；后者估计器健康，不当作两份独立真值 |
| /map | OccupancyGrid，map | 地图内容+origin+resolution 生成 hash，变更使搜索失效 |
| /tf、/tf_static | 源时间变换 | 不用最新 TF 代替缺失的源时间 TF；缺失进入有界等待，超时丢帧 |
| 已有定位急停/底盘状态 | 复用当前适配层的实际类型与话题 | PR0 清单记录精确类型与字段；缺失或锁止禁止运动，不新增解锁路径 |

所有输入分别保存 ROS source stamp 和本地 monotonic receipt；ROS 时间用于 TF/源年龄，monotonic 用于 watchdog/预算。时钟倒退、来源重启、odom reset 增加 session generation 并清空观测。

### 4.2 纯 Python 数据结构（新增 dataclass）

```python
Keyframe(id: int, session: str, stamp_ns: int, view_id: int,
         scan: LaserScan, T_odom_base: SE2, T_base_scan: SE2,
         receipt_mono: float, role: str)  # TRAIN or HOLDOUT
Hypothesis(x: float, y: float, yaw: float, score: float,
           coverage: float, conflict: float, cluster_id: int,
           per_view: tuple, support_bounds: tuple)
SearchResult(session: str, map_hash: str, complete: bool,
             hypotheses: tuple, evaluated: int, duration_s: float,
             reason: str)
RotationDecision(allowed: bool, delta_yaw: float, reason: str,
                 min_clearance_m: float | None,
                 unknown_cells: int, snapshot_stamp_ns: int)
MotionProfile(schema_version: int, geometry_hash: str,
              extrinsics_hash: str, control_chain_hash: str,
              evidence_ids: tuple, stop_tail_rad: float | None,
              center_drift_m: float | None, latency_s: float | None,
              externally_reviewed: bool, status: str)
```

Profile 缺值为 null 而不是 0；自动分析只生成 ESTIMATED。reviewed profile 必须保留外部交叉检查记录和适用地面/负载，不自动将状态设 ACCEPTED。新增模块拒绝非有限数值、不合法坐标和未知 schema。

### 4.3 ROS 新增接口（v1，全部待实现）

为缩小第一版接口包依赖，内部请求使用 String JSON 严格 schema；v1 禁止未声明字段及 NaN。异步结果不靠日志解析。

- `/automatic_localization/motion_request`，String，Reliable/Volatile/depth=1，10 Hz 租约刷新：`schema_version, session, sequence, operation(STOP|ROTATE|RELEASE), delta_yaw_rad, speed_rad_s, profile_hash`。guard 从首次接受开始固定目标角，重复 sequence 只续租、不重置里程/预算；旧序号拒绝。guard 必须先收到 STOP 建立会话才可接受 ROTATE，重启不能重放旧动作。RELEASE 仅在停稳且完成零速窗口后执行，销毁速度发布者并回报 RELEASED；未停稳时先 STOP，不提前释放。
- `/automatic_localization/motion_status`，String，Reliable/Volatile/depth=1，10 Hz：`session, sequence, state, reason, signed_progress_rad, abs_travel_rad, stopped, source_ages`。
- `/automatic_localization/cancel`，Trigger：幂等；触发停止并撤销搜索；响应只表示已接收。必须等待 motion_status.stopped，超时状态为 FAULT_STOPPED，不宣称已停稳。
- `/automatic_localization/markers`，MarkerArray，1 Hz 上限，显示候选、实际 footprint、扫掠、未知区域。
- 现有 `/automatic_localization/status` 保持旧字段，新增 `schema_version, strategy, motion_policy, search_complete, hypothesis_count, ambiguity_reason, motion_guard_state, unknown_sweep_cells, total_abs_yaw, map_hash`。

guard 以 monotonic 接收时间判断租约，超时清零；session 不提供网络认证，隔离 DDS 与现场控制权限仍由现有运行边界负责。零命令经补偿到底盘的停止保障必须与实机 watchdog 联合验收，单有此进程不构成硬件急停。

## 5. 多视角算法与函数签名

### 5.1 采集与坐标变换

新增 `make_keyframe(scan, tf_lookup, session, view_id, role) -> Keyframe | Reject`。只取停稳后的帧用于第一版搜索，避免对运动点云提出新的去畸变假设。

候选 H 表示参考帧 t0 的 map→base 位姿（记号 T_A_B 表示 B 到 A）：

`T_map_scan(ti) = H × inverse(T_odom_base(t0)) × T_odom_base(ti) × T_base_scan(ti)`。

用此变换调用 scan_map_metrics；不能将所有 scan 共用 H，也不能把旧时刻候选直接当作当前 AMCL seed。发布 seed 前计算 `H_now = H × inverse(T_odom_base(t0)) × T_odom_base(now)`，转换后的候选必须仍有效且车已停稳。

每个停稳视角先取 3 帧 TRAIN，随后取 3 帧 HOLDOUT；固定角色不能在同一次判定中混用。最多保留 8 个视角，超过则按视角覆盖抽样而非无限累积。相同视角多帧先做组内稳健聚合，再对视角等权，不能通过重复采样人为增加置信度。

新增 `collect_keyframes(buffer, role, max_views, max_age_s)`：缺 TF、过期、搬动/跳变、不同 session/map 均拒绝。后续一旦对 HOLDOUT 做参数调优，必须另取新的验证帧。

### 5.2 搜索、聚类和唯一性

新增：

```python
search_multiview(grid, train_frames, search_config,
                 deadline, cancel_token) -> SearchResult
cluster_hypotheses(candidates, xy_radius, yaw_radius) -> tuple
validate_hypotheses(result, holdout_frames, thresholds) -> QualityDecision
seed_pose_at_current_time(winner, reference_odom, current_odom) -> SE2
```

算法：

1. 复用当前粗搜步长 0.20 m/10°；正确处理 OccupancyGrid origin 的 yaw，不能仅加 origin.x/y。自由 cell 是候选中心条件，不是旋转安全证明。
2. 粗搜覆盖整个可搜索区域；取消/预算结束返回 complete=false，禁止接受暂时第一名。
3. 候选按位置距离和圆周 yaw 距离聚类。同一簇要求 xy、yaw 均接近；位置相同但朝向相反必须保留不同簇。
4. 初始使用 0.30 m/15° 聚类、最多精化 8 个独立簇；这是待离线校准的搜索配置，不是定位允许误差。边界簇不得吞掉相近的另一解。
5. 每簇同等精化预算，保留未精化替代解。若未精化簇仍可能竞争第一名，不得宣称唯一；预算不足报 SEARCH_INCOMPLETE/AMBIGUOUS_LOCATION。
6. 在 HOLDOUT 上重新评分最佳及竞争簇：初始复用 score≥0.65、coverage≥0.65、known≥30、conflict≤0.25、margin≥0.12；不在狭小模式降低标准。由标注正反例集校准后版本化。
7. 长走廊退化检查：对最佳附近 x/y/yaw 扰动采样评分，构造接近最佳（初始 score loss≤0.03）的连通支持域；边缘仍高分则扩大搜索或报 UNOBSERVABLE_AXIS。支持域超过允许位置/角度界限时拒绝；不能由小搜索窗口人为制造确定性。
8. 通过独立验证后才注入 AMCL，继续现有漂移/TF/新扫描校验。粒子集中是辅助条件，不是新证据。
9. 保存位姿先验（2026-10-09 增补）：地图上存在近似对称的“双胞胎”位置时，原地旋转无法区分两者（例如 (-0.7, 0.05) 与 (4.7, 6.2)，HOLDOUT 分差 0.03–0.07，谁高不固定）。规则如下：
   - **保存**：READY 期间，AMCL 标准差在 `max_amcl_*` 内时，每 5 s 把位姿原子写入 `saved_pose_path`（带 map_hash）。
   - **启用**：仅当本次启动加了操作员担保 `robot_not_moved`（“自上次保存后车没被移动”）才使用，不跨启动保存。保存的位姿沿里程计换算到参考关键帧。
   - **选择**：HOLDOUT 上与最佳分差小于 margin 的候选中，恰好一个在先验容差内（0.25 m/10°），且通过所有门槛和走廊检查，才选它。
   - **冲突**：明确的唯一解若不在先验处，按 AMBIGUOUS_LOCATION 拒绝、转人工，不再探测。
   - **边界**：先验不降低任何门槛。车被搬到双胞胎位置时担保即为假，因此必须由操作员逐次声明。

保留旧 deterministic_global_search 的参数与返回契约；旧测试不得因切换新入口而失效。旧策略不新增“跳过旋转”的隐式行为。

### 5.3 性能隔离

搜索放到独立 worker 进程而非 ROS timer 或纯 Python 大循环线程。每份请求带 session/map_hash；取消使结果失效，必要时终止并回收 worker。最多一个在途搜索，不积压队列。

地图距离场/射线索引可缓存，key 为 map_hash。先全图一次，后续使用候选复核；若所有候选被推翻或搜索裁剪不能保证保留替代解，重新全图搜索，未完成前不能接受。

初始每次搜索上限 120 s，整个会话 240 s（2026-10-09 实车四段探测用了 233 s，会话上限与放置担保有效期改为 360 s），旋转另有独立预算。Jetson 测 p95/p99、源时间年龄与扫描最大空窗；候选计算超时保持停止，不扩大传感器新鲜度阈值来“通过”。

## 6. 旋转安全与执行算法

### 6.1 覆盖审计，不要求用户逐点测量

新增 `scripts/audit_mid360_localization_visibility.py`：读取 bag，输出原始回波/FAST-LIO 点云/scan 的距离-方位-高度分布、被 range/height/self mask 删除比例、时间年龄和 TF 失败率；记录驱动实际类型和各过滤参数。

输出 UNKNOWN / OBSERVED_OCCUPIED / OBSERVED_FREE 分开统计。自然场景无近点只能说明没有观测，不能自动标定“最小可靠测距”。若现有 bag 不足，集中一次现场软障碍演示/视频，而不是要求用户按厘米移动并记录。若仍无法确认覆盖，明确阻塞自动旋转发布，不阻塞静止定位交付。

不全局调低 /scan.range_min。专用近场处理只有在原始有效数据和物理覆盖验证通过后才能纳入运动门；不通过虚构射线或扩大 self mask 删除外部障碍。

### 6.2 几何与未知区

新增 `evaluate_localization_rotation(snapshot, footprint, delta_yaw, profile, now) -> RotationDecision`：

1. 从现有几何配置取得 footprint，应用一次 padding，使用 geometry_hash 防止 profile 错配。优先复用现有实测参数，不要求重新量整车。
2. 以 odom 为检查坐标，不使用待确定的 map→odom；建立局部障碍与可见性快照，源时间 TF 对齐。
3. 构造目标角及制动尾段、可能中心漂移的扫掠；复用 check_swept_path 和 check_observed_free_path。采样角使最远顶点位移≤半栅格，边界/内部及插值误差均检查。
4. 未知检查对象为车体**外部新扫过的区域和保护余量**；当前车体内部不是外部自由空间，使用经过确认的实体 self geometry 掩膜处理，不把当前 footprint 周围 padding 也豁免。当前位置与真实障碍冲突立即拒绝。
5. 雷达测量从 sensor origin 出发，不能把 base 距离等同测距；近场盲区不能清空。2D 可见性仅在已确认对应高度覆盖车体碰撞范围的场景使用；悬垂/低矮障碍覆盖不足仍拒绝。
6. 完整一圈的几何包络与方向无关；分段可以选择左右，但不能声称换方向使同一完整圆周包络变小。

### 6.3 分段与停车

新增 `choose_probe(hypotheses, rotation_decisions, history) -> RotationDecision`：评估 ±30°、±60°、±90°；先安全、再避免重复视角、再比较候选预测扫描的差异。若无候选或近期两次新增视角仍无有效改善，退出，不无限探索。360°雷达旋转不保证增加信息。

guard 每 50 ms 检查源时间、心跳、急停、odom、控制权和剩余扫掠；安全计算每周期预算≤20 ms（初始目标，目标机超时就停车）。与搜索进程隔离。profile 未接受时禁止输出非零。

- 起转速度初始上限 0.40 rad/s，不能低于实际履带起转能力后仍按命令推断运动。
- 用有符号、连续展开 yaw 计算段净进度；用 abs(delta_yaw) 累计预算，禁止仅累加正向噪声。
- 预测停车角 `abs(omega_measured)*latency_bound + stop_tail_bound + yaw_margin`；剩余角小于该值即清零。profile 中 stop_tail 定义为下游开始制动后的尾角，避免与 latency 重复计数；若数据无法分离，使用端到端总余转上界替代整个公式。
- 已观测平移/横滑超过 profile.center_drift_m 停车；不让名义“原地转”掩盖实际平移。
- 停止确认复用 `_stopped` 的速度阈值与持续窗口，并要求新鲜里程计；无数据只能报停止请求已发出，不报物理停稳。
- 最大 6 段、累计绝对转角≤2π、总非零执行时间≤45 s，任一耗尽停止并人工回退。总会话时间包含搜索和停稳。

## 7. 状态机、控制权和异常

| 状态（新增者以 NEW 标注） | 进入/动作 | 正常退出 | 失败退出 |
| --- | --- | --- | --- |
| 现有 WAIT/START_LOCALIZATION | 复用 FAST-LIO 稳定和 localization lifecycle | 新策略 COLLECT_STATIC | SAFE_STOP |
| COLLECT_STATIC NEW | 零速、停稳、采集 TRAIN | SEARCH_MULTI_VIEW | 10 s 内无合格数据→SAFE_STOP |
| SEARCH_MULTI_VIEW NEW | worker 搜索，保持零速 | complete→VERIFY_HYPOTHESES | 超时/地图变更→拒绝或有界重试 |
| VERIFY_HYPOTHESES NEW | 采集新的 HOLDOUT，复核候选 | 唯一→现有 STOP_AND_VERIFY；歧义→PLAN_PROBE | 无有效候选/数据失效→SAFE_STOP |
| PLAN_PROBE NEW | 只读评估局部旋转，最多 2 s | 安全且允许运动→EXECUTE_PROBE | forbid/UNKNOWN/no profile→WAIT_MANUAL_POSE，经 SAFE_STOP |
| EXECUTE_PROBE NEW | guard 接管，manager 续租 | 达到角度→SETTLE_PROBE | 障碍/失联/预算/跳变→SAFE_STOP |
| SETTLE_PROBE NEW | 零速，1 s 停稳，最多 5 s | 新 view→COLLECT_STATIC | 不停稳→FAULT_STOPPED |
| 现有 STOP_AND_VERIFY | AMCL seed adoption、质量持续窗口 | CANDIDATE_READY 或 START_NAVIGATION | SAFE_STOP |
| 现有 START_NAVIGATION | guard RELEASE 前先 STOP/停稳；确认零速与无启动发布者 | 生命周期成功→READY，不发送目标 | 导航启动失败→停车清理 |

新增状态的任意取消使 session 失效、worker 结果丢弃、guard STOP。定位急停/搬起/odom 跳变在所有新状态生效，不能仅保留旧 ROTATE_AND_SCORE 判断。

控制链明确采用当前定位路径：`guard -> /cmd_vel_command -> 已有 compensation -> /cmd_vel -> ESP32`。Nav2 尚未 active，不假设 smoother 已可用。guard 自己做有界加减速；参数来自此链路标定，不套用 Issue #9 经 smoother 的尾角。运行前核对实际 remap、watchdog 和独占控制源。

manager 在新策略不得创建同话题速度发布者。guard STOP 并 RELEASE 销毁 publisher 后，manager 才激活 Nav2；新速度源先建立前不得复活 guard。退出/异常清零，进程崩溃依赖独立底盘 watchdog，必须故障注入验收。

本次不新增固定 map→odom 发布器。保持当前 AMCL/manual localization 的 TF 所有权，由 PR0 核对实机；候选搜索不广播正式 TF。待确定候选只用 Marker 显示，不能让 provisional AMCL pose 被误认为 READY。静态 map→odom 锁定属于另一项 TF 设计变更，不夹带实施。

## 8. 参数、启动和诊断

以下为拟新增参数；数值是测试初值，不是实车安全认证。

| 参数 | 初值/约束 |
| --- | --- |
| localization_strategy | legacy_full_rotation（开发期默认）、stationary_only、segmented_rotation |
| motion_policy | forbid 默认；guarded 必须显式启用且 profile 有效 |
| train_frames_per_view / holdout_frames_per_view | 3 / 3，正整数 |
| max_views / max_probe_segments | 8 / 6 |
| probe_angles_rad | ±π/6、±π/3、±π/2，非零且绝对值≤π/2 |
| max_total_probe_yaw_rad / probe_motion_timeout_sec | 2π / 45，monotonic 预算 |
| motion_request_timeout_sec | 0.30；实测停车距离必须覆盖该等待时间 |
| sensor_freshness_sec | 复用 0.5 s 上限；profile 需覆盖最坏数据年龄，不能称瞬时障碍保护 |
| search_timeout_sec / session_timeout_sec | 120 / 360（初始 240） |
| independent_cluster_xy_m / yaw_rad | 0.30 / π/12，离线验证后调整 |
| max_refined_clusters | 8；仍有竞争候选则不接受 |
| motion_profile_path | 空默认；空/错 hash/未 review 禁止 guarded |

在 `carbot_auto_localization_real.yaml` 配置并在 `_declare_parameters` 校验，启动后禁止动态开启运动或放宽安全参数。现有 validation_only 只控制是否激活导航，新的 motion_policy 控制是否允许转动，两者组合需全部测试。

维护入口拟增加（尚不能执行）：`./start_real_nav.sh --localization-strategy stationary_only` 和 `--localization-strategy segmented_rotation --localization-motion guarded`。参数沿 `start_real_robot_navigation_rviz.sh -> jetson_navigation_start.sh -> launch -> manager/guard` 传递；审查 `jetson_navigation_control.py`、`jetson_navigation_wait.py` 对新状态和 240 s 总期限的兼容性。保留原参数含义、单实例锁和失败清理。

拒绝原因枚举至少包含：SEARCH_INCOMPLETE、AMBIGUOUS_LOCATION、UNOBSERVABLE_AXIS、TF_AT_SOURCE_MISSING、SENSOR_STALE、MAP_CHANGED、UNKNOWN_SWEEP、OBSTACLE_IN_SWEEP、PROFILE_INVALID、CONTROL_CONFLICT、ODOM_JUMP、MOTION_BUDGET_EXHAUSTED、CANCELED。用户界面显示中文原因和是否可人工定位，不反复要求重新启动。

## 9. 自动分析与最小人工配合

新增 `scripts/analyze_localization_rotation_profile.py`，复用现有 `analyze_issue9_motion_evidence.py` 的记录解析思想，不能直接复制其不同速度链标定值。

输入：命令请求、补偿前后速度、odom、原始 IMU、轮增量（存在时）、guard 状态、geometry/extrinsics/control hashes。计算首次运动延迟、左右角速度响应、制动尾角、中心漂移、数据年龄、异常样本；输出 JSON 和 Markdown 报告。样本不足/轮滑/时间异常标 INSUFFICIENT，不自动生成假数值。

先分析已有 bag，只有缺失项才集中采一次。程序自动完成分段计时和统计，用户无需填写表格。左右至少各 3 次有效停止样本，现场独立视频/目视核对一次代表性动作与最坏余转；没有角度参考的视频只证明是否明显异常，不宣称厘米/角度精度。无法获得可用外部交叉证据则 profile 留 ESTIMATED，交付静止模式与监督开发结果，不宣称无人值守旋转已验收。

运动测试必须单独获授权、现场可断电/急停；不因写文档或运行采集器触发运动。历史技能 checkpoint 不代表当前机器人状态。

## 10. 测试规格（每项必须有断言）

新增纯逻辑测试到 B/test；guard ROS 测试在回环隔离 DDS 域和 `/test_confined/*` 话题，不接实车域。

| ID / 测试函数名 | 输入 | 必须断言 |
| --- | --- | --- |
| L01 test_stationary_never_commands_motion | stationary_only，唯一地图候选 | 非零 Twist=0；候选通过后按 validation_only 决定是否激活 |
| L02 test_multiview_transform | 同一点、0°与90°观测，已知 odom 与偏置 scan 外参 | 地图端点重合，数值误差<1e-6 m；错误共用 H 的对照不通过 |
| L03 test_seed_uses_current_pose | 参考帧后旋转30° | seed yaw 加30°，不是旧 H |
| L04 test_yaw_ambiguity_same_position | 同位置0°/180°等分候选 | 两簇、AMBIGUOUS_LOCATION、无 AMCL seed |
| L05 test_corridor_unobservable | 平行重复墙、纵向平坦得分 | UNOBSERVABLE_AXIS，无 READY |
| L06 test_runner_up_not_pruned | 次优簇粗分低但精分相同 | 保留并拒绝；计算预算不足 complete=false |
| L07 test_holdout_independent | TRAIN 高分、HOLDOUT 穿墙 | 拒绝；重复训练帧不能充当 holdout |
| L08 test_rotated_map_origin | map origin yaw=π/2 | world↔cell 和候选位置正确 |
| S01 test_near_but_outside_sweep | 0.55m 内障碍在扫掠外，外部扫掠全部有可信 free 证据 | 新几何门通过；不得仅凭最近距离拒绝 |
| S02 test_corner_collision_mid_arc | 起终点无碰撞，中途角点碰细障碍 | OBSTACLE_IN_SWEEP |
| S03 test_blind_zone_is_unknown | 0.5m scan 盲区、地图 free | UNKNOWN_SWEEP、零速；地图 free 不能覆盖未知 |
| S04 test_self_mask_not_padding | 车内未知、车外保护边缘未知 | 车内不要求外部 ray；车外未知仍拒绝 |
| S05 test_braking_sweep | 目标角安全、余转区域碰障 | 起步拒绝或提前停车，不越过安全边界 |
| S06 test_sparse_3d_not_free | 只有高处有限回波、低处无点 | 不能推出低处自由空间 |
| C01 test_yaw_wrap_and_noise | +π/-π跨越、正负噪声、反转 | 段净进度准确；预算累计绝对运动，不会假完成 |
| C02 test_search_worker_stall | worker卡死/超时 | guard 周期不被阻塞，搜索期间无非零，拒绝迟到结果 |
| C03 test_cancel_and_lease_loss | 转动中取消或manager退出 | 在租约期限+一次周期内发零，必须等待实际停稳反馈 |
| C04 test_guard_crash_watchdog | guard进程退出 | 隔离底盘模拟器按watchdog停车；实车另测，不能以模拟通过替代 |
| C05 test_stale_tf_odom_emergency | 每项单独注入 | 当周期拒绝/停止，不解锁、不重发旧目标 |
| C06 test_duplicate_request | 重复/乱序/跨session request | 不重置目标角/预算；重启须STOP握手 |
| C07 test_handoff_no_two_publishers | 定位完成→Nav2激活 | guard先停稳并释放；不存在两个有效运动源 |
| C08 test_profile_mismatch | 外参/几何/链路hash变化、null字段 | PROFILE_INVALID，零速 |
| C09 test_strategy_matrix | 3策略×2motion_policy×2validation_only | 参数冲突启动报错；forbid始终无非零 |
| C10 test_budget_and_clock_reset | 六段/2π/45s/240s、ROS倒退 | 对应预算拒绝；时钟重置不重放旧动作 |

保留并运行 `test_automatic_localization_manager.py`、`test_automatic_localization_quality.py`、`test_real_nav_safety.py`、recovery 几何/可见性测试。纯测试命令在配置好 PYTHONPATH 的开发环境执行 `pytest -q <上述测试路径>`；ROS测试必须经过现有隔离容器环境，不能直接在当前实车DDS执行。

### 实车最小验收矩阵

- 先只读覆盖预览，再开阔地标定，最后软障碍狭小场景；不拿人体/硬墙作为首次近障试验。
- 可辨识墙角、窄但可转空间、重复走廊，每类3个起始朝向、每个3次。独立实际位置/方向标签由固定参考照片/少量标记一次提供，后续自动统计；重复走廊允许正确拒绝，不计为错误定位。
- 可辨识数据集成功率目标≥90%；错误放行=0（仅限该测试集，不宣称普遍零风险）。具备足够外部参考时报告≤5cm/3°目标，否则只报告可验证的匹配/重复性结果，不杜撰绝对误差。
- 未知扫掠/过期/取消/无解样例全部停止；运动没有擦碰；实际余转/漂移在profile边界内。CPU压力下停车链路仍工作。
- 完成定位后不自动发导航目标。正常开阔自动定位、人工2D Pose、Issue #9、cumulative mapping 无回归。

## 11. PR 顺序与退出条件

| PR | 交付文件/重点 | 合入条件 | 依赖 |
| --- | --- | --- | --- |
| PR0 只读审计与契约 | audit脚本、输入/控制源清单、基线bag manifest | 原始类型/TF/过滤链有记录；盲区结论不伪装free；无运动 | 无 |
| PR1 静止自动定位 | observations/hypotheses、manager静止入口、配置/脚本透传 | L01–L08 + 旧回归通过，worker取消完整；不依赖近场能力 | PR0接口 |
| PR2 几何只读预览 | rotation_policy、扫掠/可见性适配、Marker | S01–S06通过；明确已覆盖与未知；没有速度发布者 | PR0；可与PR1分支开发 |
| PR3 执行保护及自动分析 | motion_guard、profile分析、租约/预算 | C01–C10与底盘watchdog隔离测试通过；运动默认forbid | PR1、PR2 |
| PR4 受控验收 | 一次集中标定、软障碍测试、版本化profile/报告 | 近场证据与停车边界合格；未合格只交付静止/预览 | PR3、单独实车授权 |
| PR5 启动收口 | 帮助信息、中文状态、回滚文档 | 一键流程与失败清理回归；仅经接受的profile能开guarded | PR4 |

推荐第一个实现任务：PR0只读审计 + PR1静止入口，不先改 0.55m、不先开启狭小空间运动。不因PR0发现真实盲区而停掉所有开发；静止自动定位可独立交付，但不可把它包装成旋转功能已经完成。

## 12. 回滚和证据管理

开发期保留 legacy_full_rotation；它保留原有保护，不能作为新策略拒绝后的自动运动回退。显式选择旧策略须重新启动并走维护流程。停用新策略后不加载guard，Nav2和旧定位参数恢复原配置；不改地图/固件/全局scan裁剪。

每次报告包含 commit、dirty patch hash、地图hash、profile、外参、参数、话题类型、源时间范围、运行域、成功/失败原因和证据路径。大bag留校准目录并遵循现有忽略规则；提交小manifest和报告，不提交凭据或私人场景视频。

文档验收不等于代码验收。本文件完成后，尚未实现以上新模式，也未验证当前实车运行状态。
