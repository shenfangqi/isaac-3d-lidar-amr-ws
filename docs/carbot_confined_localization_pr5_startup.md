# Issue #13 PR5：启动收口（受保护旋转的一键启动、中文状态、回滚）

对应实施规格第 11 节 PR5。依赖 PR4 已验收的运动配置（`docs/evidence/issue13_motion_profile_accepted_2026-10-06.json`）。

## 用法

```bash
# 静止自动定位（不转动），与之前相同
./start_real_nav.sh --automatic --localization-strategy stationary_only

# 分段旋转但禁止运动：有歧义时不转，转人工 2D Pose
./start_real_nav.sh --automatic --localization-strategy segmented_rotation

# 受保护旋转：有歧义时小车可能原地转动，现场必须有人看护
./start_real_nav.sh --automatic --localization-strategy segmented_rotation \
  --localization-motion guarded \
  --motion-profile docs/evidence/issue13_motion_profile_accepted_2026-10-06.json \
  --operator-rotation-clear --operator-present
```

`--automatic-activate` 同样可用（定位通过后激活 Nav2，不发目标）。

## 受保护旋转的启动检查

在动机器人之前，工作站上依次检查（任一失败直接退出，不连 Jetson）：

1. `--localization-motion guarded` 只能配 `--localization-strategy segmented_rotation`；
2. 必须给出存在的 `--motion-profile`；
3. 必须加 `--operator-present`；
4. `scripts/check_motion_profile.py`：配置必须严格符合格式、状态 ACCEPTED 且经过外部审核，三个哈希与当前 `carbot_parameters.yaml`（车体外形 + 0.05 m padding、`sensors.mid360`、`control`）重新计算的结果一致；
5. `--operator-rotation-clear`、`--motion-profile` 只能与 guarded 一起用。

之后把配置拷到 Jetson `~/Projects/isaac_ros-dev/motion_profiles/` 并核对 sha256，经 `jetson_nvblox_container.sh` 的第 7–11 个参数传给 launch。定位开始前再核对：manager 的 `motion_policy` 与请求一致；guarded 时 guard 节点在运行且日志为 `motion guard: permitted=True`；forbid 时不得有 guard。任何一项失败都按启动失败清理。manager 和 guard 在 Jetson 上还会各自重新检查配置和哈希。

`--operator-rotation-clear` 是本次启动的放置承诺：只覆盖传感器看不到的扫掠格子（MID-360 盲区），看到的障碍、配置、预算、急停和底盘状态照常检查；不跨启动保存。

## 状态说明

| 状态 | 含义 |
| --- | --- |
| `COLLECT_STATIC` | 静止采集训练帧 |
| `SEARCH_MULTI_VIEW` | 全图搜索（Jetson 约 52 s） |
| `VERIFY_HYPOTHESES` | 用新的验证帧复核候选 |
| `PLAN_PROBE` | 只读评估 ±30°/±60°/±90° 探测角并选择一个（仅 segmented_rotation） |
| `EXECUTE_PROBE` | guard 执行探测旋转，manager 续租 |
| `SETTLE_PROBE` | 零速、等待停稳，然后从新视角重新采集 |
| `STOP_AND_VERIFY` | 种子注入 AMCL 并持续复核 |
| `CANDIDATE_READY` | 候选通过（仅验证，Nav2 未激活） |
| `READY` | Nav2 已激活，不发目标 |
| `SAFE_STOP` → `WAIT_MANUAL_POSE` / `FAULT_STOPPED` | 拒绝后停车；前者可用 RViz 2D Pose 人工定位，后者需排查后重启 |

## 拒绝原因（`/automatic_localization/status` 的 `reject_reason_text`）

| 原因 | 中文说明 | 可否人工 2D Pose |
| --- | --- | --- |
| `SEARCH_INCOMPLETE` | 全图搜索未完成，不能接受暂时第一名 | 可以 |
| `AMBIGUOUS_LOCATION` | 存在多个相近位置/朝向候选，无法确定唯一位置 | 可以 |
| `UNOBSERVABLE_AXIS` | 环境在某个方向上退化（如长走廊），该方向不可观测 | 可以 |
| `TF_AT_SOURCE_MISSING` | 缺少传感器源时间对应的坐标变换 | 可以 |
| `SENSOR_STALE` | 传感器数据过期 | 可以 |
| `MAP_CHANGED` | 地图已变更，搜索结果作废 | 可以 |
| `UNKNOWN_SWEEP` | 旋转扫掠区域存在未观测（未知）区域，不能证明安全 | 可以 |
| `OBSTACLE_IN_SWEEP` | 旋转扫掠区域内有障碍 | 可以 |
| `PROFILE_INVALID` | 运动标定配置缺失、未验收或与当前车辆不匹配 | 可以 |
| `CONTROL_CONFLICT` | 存在其他速度控制源或控制权冲突 | 不可以，需排查后重启 |
| `ODOM_JUMP` | 里程计跳变或车辆被搬动 | 可以 |
| `MOTION_BUDGET_EXHAUSTED` | 旋转段数、角度或时间预算已用尽 | 可以 |
| `CANCELED` | 定位已被取消 | 可以 |
| `NO_VALID_CANDIDATE` | 没有候选通过独立验证帧的匹配门槛 | 可以 |
| `LOCALIZATION_FAULT` | 定位急停已触发或状态未知，需排查并重启后才能运动 | 不可以，需排查后重启 |
| `CHASSIS_BLOCKED` | 底盘报告运动锁止、连接中断，或底盘状态缺失/过期 | 可以 |

## 回滚

- **关闭受保护旋转**：去掉 `--localization-motion guarded`（默认 forbid）。guard 不会启动，`segmented_rotation` 退化为"有歧义转人工定位"。
- **完全回到静止或旧策略**：`--localization-strategy stationary_only` 或不加该参数（`legacy_full_rotation`）。
- **作废运动配置**：车体外形、`sensors.mid360` 或 `control` 任何一项改变，哈希即不匹配，启动前就会被拒绝；需要重新采集和审核。删除 Jetson `~/Projects/isaac_ros-dev/motion_profiles/` 下的文件不影响其他模式。
- **卸载**：PR5 只改了启动脚本、容器脚本和新增的 `check_motion_profile.py`；回退这三个文件即可恢复 PR4 状态。地图、固件、全局 scan 参数均未改动。

## 验证

- `test_issue13_startup.py`：校验工具只放行 ACCEPTED 且哈希匹配的配置（ESTIMATED、REVIEWED、哈希不符、格式错误都拒绝）；仓库里的 ACCEPTED 配置与当前参数仍匹配；启动脚本和容器脚本的组合规则、启动前校验顺序、sha256 核对、guard 核对都在。
- 离线运行启动脚本（不可达主机）确认五种错误组合都在连接 Jetson 前被拒绝。
- 第一次真正的受保护旋转需要单独授权、现场可急停。
