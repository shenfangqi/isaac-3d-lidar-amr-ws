# Issue #13 PR1：静止自动定位（stationary_only）

对应实施规格第 11 节 PR1。依赖 PR0 契约（`7a0b8c0`）。本 PR 新增的策略**从不发运动命令，也不创建速度发布者**；默认策略仍是 `legacy_full_rotation`，旧流程不变。

## 使用

```bash
./start_real_nav.sh --automatic --localization-strategy stationary_only           # 停在 CANDIDATE_READY
./start_real_nav.sh --automatic-activate --localization-strategy stationary_only  # 通过后激活 Nav2，不发目标
```

参数沿 `start_real_robot_navigation_rviz.sh → jetson_nvblox_container.sh（第 6 个参数）→ launch localization_strategy → manager` 传递。启动脚本会用 `ros2 param get` 核对部署的 manager 实际使用的策略，旧部署没有该参数时直接失败。`--manual`/`--stationary-validation` 与该选项组合会报错；`segmented_rotation` 在 PR3 的 guard 完成前一律拒绝启动。旧的 `jetson_navigation_start.sh` 入口未改，保持 legacy 策略。

## 流程

`WAIT_SENSORS → START_LOCALIZATION → COLLECT_STATIC → SEARCH_MULTI_VIEW → VERIFY_HYPOTHESES → STOP_AND_VERIFY（现有）→ CANDIDATE_READY / START_NAVIGATION`

1. **COLLECT_STATIC**：只在 `_stopped()` 为真时采 3 帧 TRAIN。每帧按扫描**源时间**查 `odom←base_footprint` 和 `base_footprint←scan`；TF 缺失时最多等待 `sensor_freshness_sec`，超时丢帧。一旦检测到运动，丢弃已采的部分帧。10 s 内凑不齐 → `SENSOR_STALE`。
2. **SEARCH_MULTI_VIEW**：独立 worker 进程（spawn + 单向 Pipe，最多一个在途任务）做全图搜索。结果带 `(session, map_hash)` 令牌，令牌不符的结果一律丢弃。
3. **VERIFY_HYPOTHESES**：采 3 帧新的 HOLDOUT（时间戳必须晚于全部 TRAIN 帧），在 worker 里复核。
4. 通过后按 `H·inv(T_odom_base(t0))·T_odom_base(now)` 换算到当前时刻，再注入 AMCL。之后沿用现有的漂移、TF 和质量检查。

任何拒绝都会给出契约中的原因（附中文文案和"能否人工定位"标志），进入 `SAFE_STOP → WAIT_MANUAL_POSE`。其他失效条件：
- `/automatic_localization/cancel` 取消；
- 里程计跳变（`ODOM_JUMP`）；
- 地图 hash 变化（`MAP_CHANGED`）；
- 整个会话超过 240 s。

这些情况都会终止 worker，并使本次会话作废。

## 搜索算法要点（`localization_hypotheses.py`）

- **粗搜**：0.20 m / 10°，所有自由格中心都参与，候选中心的计算考虑了地图 origin 的 yaw。
- **聚类**：按 xy 0.30 m 加圆周 yaw 15° 聚类，只和簇种子比较，避免链式吞并；同一位置朝向相反的解属于不同簇。
- **精化**：前 8 个簇各分配相同预算——先在标准评分上做模式搜索，再用**地图距离场上的平滑拟合分**（σ = 0.05 m）抛光。原因是 0.15 m 的匹配容差让标准评分在真值附近形成约 0.3 m 宽的平台，只用标准评分时，合成数据的位置误差可达 0.13 m。精化后落在同一邻域内的簇合并为一个解。
- **完整性**：没精化的簇如果粗分接近胜者、又不在任何已精化解的邻域内，就判 `SEARCH_INCOMPLETE`。截止时间到或被取消，同样判为未完成。
- **验证**：只用 HOLDOUT 帧。门槛沿用现有参数，未放宽：score≥0.65、coverage≥0.65、known≥30、conflict≤0.25、与第二名差距≥0.12。
- **退化检查**：在 0°/45°/90°/135° 四个方向和 yaw 上测量平滑拟合分的近最优连通区域；区域无界或超过 0.20 m / 5°，判 `UNOBSERVABLE_AXIS`。
- **聚合**：同一视角的多帧取中位数，不同视角等权，所以重复采样不会抬高置信度。
- **兼容性**：旧的 `deterministic_global_search` 及其测试保持不变。

## 验证

| 项目 | 结果 |
| --- | --- |
| 纯逻辑测试（宿主机，pytest 替身） | manager 31、契约 63、假设搜索 16、审计 12，全部通过 |
| L02–L08 | `test_localization_hypotheses.py` 中同名测试；L01 和接线测试在 `test_automatic_localization_manager.py` |
| 真实地图合成扫描（168×256 格） | 工作站搜索 6.4–7.2 s（旧搜索 11.7 s）；4/4 接受，误差 0.2–0.9 cm / ≤0.3° |
| 真实 bag 离线回放（09-30 两个静止 bag 的 `/scan_localization`） | 两次都接受同一个解 (3.565, 2.125, −1.59)，HOLDOUT 0.73–0.78，第二名 ≤0.51。与当次人工 2D Pose 得到的 AMCL 位姿相差 0.17 m / 1°；新解在标准分、冲突率和尖锐分上都更好，但没有外部真值，不能宣称绝对精度 |

**尚未完成：**
- 容器内 `colcon build` 和正式 pytest，包括 `test_flake8`/`test_pep257`。宿主机只检查了行长、docstring 和未使用的 import；
- Jetson 上的耗时，按旧数据的 5.6 倍估算约 40 s，预算 120 s；
- 实车只读验证（`--automatic` 停在 CANDIDATE_READY）。

## 契约补充

- 新增拒绝原因 `NO_VALID_CANDIDATE`：所有候选都没通过 HOLDOUT 门槛。
- `/automatic_localization/cancel`（Trigger）：幂等，只表示请求已接收，实际停下要看 status。
- status 新增字段 `session`、`reject_reason_text`、`manual_pose_allowed`。`motion_guard_state` 和 `unknown_sweep_cells` 在 PR3 之前为 `null`。
