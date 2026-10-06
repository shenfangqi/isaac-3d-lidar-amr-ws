# Issue #13 PR3：旋转执行保护（motion guard）与标定分析

对应实施规格第 11 节 PR3。依赖 PR1（静止定位）和 PR2（扫掠判定）。**运动默认禁止**：只有 `localization_strategy=segmented_rotation` + `motion_policy=guarded` + 状态为 ACCEPTED 且三个哈希都匹配的运动标定配置，guard 才可能输出非零角速度。目前没有任何 ACCEPTED 配置，启动脚本也继续拒绝 `segmented_rotation`，所以本 PR 合入后机器人的行为不变。

## 组成

| 文件 | 作用 |
| --- | --- |
| `localization_motion_guard.py` | guard 状态机（纯逻辑）、`ProbeLink`（manager 侧租约协议）、`profile_hash` |
| `localization_motion_guard_node.py` | guard 的 ROS 外壳：50 ms 控制周期；扫掠评估在独立回调组异步执行；速度发布者只在允许运动且完成 STOP 握手后创建，RELEASE 时销毁，退出时先发零 |
| `localization_rotation_policy.py`（追加） | `RotationProgress`（yaw 连续展开，分开统计段净进度和绝对转角）、`ProbeBudget`、`predicted_stop_angle`、`choose_probe` |
| `automatic_localization_manager.py` | `segmented_rotation`：歧义时 PLAN_PROBE → EXECUTE_PROBE → SETTLE_PROBE → 新视角 COLLECT_STATIC；manager 不创建速度发布者 |
| `localization_profile_analysis.py` + `scripts/analyze_localization_rotation_profile.py` | 从 bag 估计运动标定配置（只产出 ESTIMATED/INSUFFICIENT） |
| `carbot_navigation_real.launch.py` | 新参数 `motion_policy`（默认 forbid）、`motion_profile_path`、`extrinsics_hash`、`control_chain_hash`、`operator_rotation_clear`；guard 只在 segmented + guarded 时启动 |

## guard 的规则

- 必须先收到 STOP 建立会话才接受 ROTATE；同序号只续租，不重置目标角和预算；旧序号丢弃；其他会话或同序号不同内容 → `CONTROL_CONFLICT`。
- 每个周期检查：租约（0.30 s）、里程计新鲜度（0.5 s）、扫掠判定新鲜并覆盖剩余角度、定位急停（未收到也算不安全）、底盘状态（`/carbot/status` 新鲜 ≤1.5 s、已连接、未锁止）、中心漂移、预算。任何一项失败立即输出零。
- 剩余角度小于 `|实测角速度| × 停止延迟 + 停车尾角 + 5°` 时提前清零。
- "已停稳"只在新鲜里程计连续满足阈值时报告。RELEASE 只在停稳后执行，之后不再接受任何请求。
- 里程计时间倒退或跳变：作废会话；同一会话 ID 不能再次握手（防止重启后重放旧动作）。
- 新增拒绝原因：`LOCALIZATION_FAULT`（定位急停）、`CHASSIS_BLOCKED`（底盘锁止/断开/状态过期）。

## manager 的 segmented_rotation 流程

搜索不完整、`AMBIGUOUS_LOCATION`、`UNOBSERVABLE_AXIS`、`NO_VALID_CANDIDATE` 进入 PLAN_PROBE（其他拒绝原因照旧）。PLAN_PROBE 用 PR2 判定模块只读评估 ±30°/±60°/±90°，`choose_probe` 只选放行且能带来新视角（与已有视角相差 ≥20°）的角度；`motion_policy=forbid` 直接 `PROFILE_INVALID`。guard 拒绝、不响应或超时都拒绝并发 STOP。停稳 1 s 后进入新视角：TRAIN 帧要求当前视角凑够，已用过的 HOLDOUT 帧丢弃。连续两次探测假设数没有减少则以 `AMBIGUOUS_LOCATION` 结束。接受候选时 RELEASE；激活 Nav2 前确认 guard 已报告 RELEASED 且 `/cmd_vel_command` 没有发布者。会话开始、换请求和失败时立即发送请求，不等下一个周期。

## 规格 C01–C10

| ID | 测试 |
| --- | --- |
| C01 | `test_yaw_wrap_and_noise` |
| C02 | `test_search_worker_stall_only_leases_stop`（guard 本身是独立进程） |
| C03 | `test_cancel_and_lease_loss`；ROS 层 `test_guard_round_trip_in_isolated_domain` |
| C04 | `test_guard_crash_watchdog`：用规范底盘模型 `isaac_sim/carbot_control.py`（`cmd_vel_timeout_s=0.50`）模拟；**ESP32 实车 watchdog 仍需单独验收** |
| C05 | `test_stale_tf_odom_emergency`、`test_unknown_emergency_state_forbids_motion`、`test_chassis_block_or_silence_stops`、`test_missing_chassis_status_forbids_motion` |
| C06 | `test_duplicate_request`、`test_conflicting_content_on_same_sequence_halts` |
| C07 | `test_handoff_no_two_publishers`、`test_guard_that_never_releases_fails_instead_of_activating` |
| C08 | `test_profile_mismatch`、`test_motion_permission`、`test_guard_without_carbot_msgs_never_permits_motion` |
| C09 | `test_strategy_matrix`（3 策略 × 2 motion_policy × 2 validation_only） |
| C10 | `test_budget_and_clock_reset`、`test_odometry_clock_reset_never_replays_old_actions` |

ROS 层测试在独立 DDS 域（87）和 `/test_confined/*` 话题上运行，不接触实车域和真实 `/cmd_vel_command`。

## 标定分析（2026-10-06）

`docs/evidence/issue13_pr3_rotation_profile_estimate.{json,md}`：`2026-09-23_cmd_comp_final/ground_turn_0p40_4pairs_10`（0.40 rad/s，左右各 4 次）全部有效，状态 **ESTIMATED**：停止延迟最大 0.138 s、停车尾角最大 0.069 rad、中心漂移最大 8 mm。0.20 rad/s 的 bag 标为 other speed，不混入。

这份估计**不能直接验收**：

- 09-23 时 FAST-LIO 还没接入，那时的 `/odom` 是轮速计/EKF，而 guard 使用 FAST-LIO 的 `/odom`；
- 停车前里程计角速度只有指令的约 60%（0.24 vs 0.40 rad/s），可能是打滑或该里程计低估；
- 只有里程计，没有外部参照。

用当前链路（FAST-LIO 里程计 + `/cmd_vel_command`）录的旋转数据目前一份都没有：09-30 的自动旋转 bag 都是旋转结束后才开始录的静止窗口。

## 验证

- 容器全套：`isaac_3d_lidar_bringup` 全部通过（含 flake8/pep257），`carbot_nav_recovery` 51 passed。
- Jetson 只读核对（2026-10-06）：`/localization/emergency_stop` 为 `std_msgs/Bool`，RELIABLE + TRANSIENT_LOCAL，由 `fast_lio_base_adapter` 发布；`/carbot/status` 为 `carbot_msgs/CarbotStatus`，BEST_EFFORT，2 Hz，由 ESP32 `carbot_base` 发布。

## PR4 之前必须完成

2026-10-06 进展（用户授权、现场有人看护）：

1. ~~导航容器编入 `carbot_msgs`~~：已完成，容器可解析 `/carbot/status`。
2. 用当前链路录制停车数据：已完成，左右各 4 次 0.40 rad/s、60°，得到 ESTIMATED（`docs/evidence/issue13_pr4_rotation_profile_2026-10-06.*`）。实测角速度为指令的约 64%，轮编码器与 FAST-LIO 一致，不是打滑。**还差**：一次外部交叉核对（视频或目视）才能标 REVIEWED；ACCEPTED 需要你单独决定。
3. ~~ESP32 watchdog 实车验收~~：4/4 通过，停发指令后 0.49–0.57 s 停下，多转 8.4–8.9°（`docs/evidence/issue13_pr4_esp32_watchdog_2026-10-06.md`）。
4. ~~guard 崩溃时的扫掠范围~~：已修复。扫掠余转改为 `指令角速度 × max(停止延迟, 底盘 watchdog 0.60 s) + 尾角 + 5°`，按当前配置约 0.33 rad，覆盖实测的崩溃后多转 0.15 rad。guard 自身的提前清零仍按正常延迟预测。
5. 启动脚本参数透传（规格第 8 节；可在 PR5 收口）。
6. 之后的每次实车运动测试仍须单独授权、现场可急停。
