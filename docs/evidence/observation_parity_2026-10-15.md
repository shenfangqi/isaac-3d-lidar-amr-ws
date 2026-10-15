# observation_parity：30 分钟静止通信录制（2026-10-15 晚）

目的：完成 `hardware_calibration_backlog.yaml` 里 `observation_parity` 要求的 30 分钟实车验证，同时补上
`imu_extrinsics_and_time` 一直缺的“ESP32 周期重同步固件刷入后 30 分钟时钟漂移”验证。小车全程静止、通电，
没有启动导航栈，没有发送任何运动指令。

工具：`scripts/record_observation_parity.sh`（Jetson 上 `systemd-run` 录制，全部话题 best-effort QoS，
不录点云，不订阅 `/cmd_vel_command`）和 `scripts/analyze_observation_parity.py`。阈值用
`configs/carbot/observation_parity_acceptance.yaml`（2026-10-15 操作者确认）。PR #92。

录制话题：`/wheel_ticks`（ESP32，50 Hz）、`/carbot/status`（ESP32，2 Hz）、`/mid360/imu/data_raw`（MID-360，200 Hz）。
`/odom` 和 `/fast_lio/imu_odom` 没有发布者（导航栈没启动），所以没录。

## 两次录制

| 次 | 开始（JST） | 时长 | 结论 | 报告 |
|---|---|---|---|---|
| 01 | 21:00:34（Jetson 开机不到 1 分钟） | 1856.9 s | **未通过**：ESP32 时间戳偏差约 21.4 h | `docs/evidence/observation_parity_2026-10-15_run01.json` |
| 02 | 21:37（重启 micro-ROS 代理之后） | 1858.8 s | **全部通过** | `docs/evidence/observation_parity_2026-10-15_run02.json` |

原始 bag：Jetson `/home/shenfq/Projects/isaac_ros-dev/calibration_data/2026-10-15_observation_parity_0{1,2}`，
工作站 `calibration_data/` 下同名目录各有一份副本（含 `observation_parity_analysis.{json,md}`）。

## 第 2 次结果（时间正确时）

| 项目 | 实测 | 阈值 | 2026-09-18（旧固件）对照 |
|---|---|---|---|
| `/wheel_ticks` 频率 | 50.0 Hz | 50 ± 2% | 49.99 Hz |
| `/wheel_ticks` 接收间隔 最大 / p99.9 | 25.9 / 21.6 ms | ≤ 400 / ≤ 100 ms | 345.3 / 56.9 ms |
| 跳号 / 重复 / 乱序 | 0 / 0 / 0 | 跳号 ≤ 0.1%，乱序 0 | 20 / 6 / 0 |
| 设备时间间隔最大值 | 20.0 ms | ≤ 100 ms | 60.0 ms |
| 接收减 header 延迟，每分钟中位数 | 12.09–13.08 ms | 在 [−5, 20] ms 内 | 1.98–19.01 ms |
| 30 分钟内延迟中位数的变化幅度 | **0.99 ms** | ≤ 5 ms | **17.0 ms** |
| 单样本时钟修正最大值 | 0.02 ms | ≤ 2 ms | 0（固定 offset） |
| `/carbot/status` offset 更新次数 / 单次最大 / 总范围 | 42 / 0.5 ms / 7.3 ms | ≥ 20 / ≤ 2 ms | 0 / 0 / 0 |
| 重连 / ping 失败 / boot 变化 | 0 / 0 / 0 | 0 | 0 / 0 / 0 |
| `/carbot/status` 最大接收间隔 | 503.9 ms | ≤ 1000 ms（守护节点上限 1.5 s） | 737.7 ms |
| `/mid360/imu/data_raw` 频率 / 最大间隔 / p99.9 | 200.0 Hz / 24.0 ms / 7.6 ms | 200 ± 2% / ≤ 100 / ≤ 20 ms | 未录 |
| `/mid360/imu/data_raw` 延迟每分钟中位数 / 变化幅度 | 11.71–11.83 ms / 0.13 ms | [0, 20] ms / ≤ 2 ms | 未录 |
| 消费端过滤回放（`wheel_odometry.py` 规则） | 丢弃 0 = 重复 + 乱序 0 | 相等 | 丢弃 6 = 重复 6 |

仿真证据配置 `configs/carbot/evidence_degraded.yaml` 对编码器丢包（0.00022）、重复（6.7e-05）和最大接收空窗
（345 ms）的建模都能覆盖本次实测（0 / 0 / 25.9 ms），训练看到的传输劣化比实车更严重，不需要重新生成。

结论：
- 周期重同步固件（`ce613ed`，每 60 s 重新对时）在 30 分钟内把延迟漂移压在约 1 ms，旧固件是 17 ms。
- Jetson Wi-Fi 省电关闭后，ROS 接收端空窗从 345 ms 降到 26 ms，没有重复样本。
- `wheel_odometry` 已经按 sequence/设备时间戳丢弃重复和乱序样本（`wheel_odometry.py:161`），本次没有需要丢弃的样本。

## 第 1 次失败的原因：冷启动时钟顺序（#95）

Jetson 的 `systemd-timesyncd` 日志：

| Jetson 时钟 | 事件 |
|---|---|
| 刚通电 | RTC 没有电池，读出 1970 年 |
| 开机约 1 s | 恢复上次关机前保存的时间 **2026-10-14 23:37:37** |
| 显示 23:37:40 | `micro-ros-agent.service` 启动，ESP32 建立会话，**按这个旧时间对时** |
| 约 30 s 后，显示 21:00:09 | NTP 首次同步（`1.pool.ntp.org`），系统时钟向前跳约 21 h 23 min |

结果：第 1 次录制中 `/wheel_ticks` 的接收减 header 延迟为 76 922.3 s → 76 920.5 s。固件一直在修正
（offset 更新 3691 次，单次 ≤ 0.5 ms，每分钟约 −60 ms），照这个速度追平要约 890 天。除时钟以外的传输项全部通过
（最大接收空窗 28.3 ms，0 跳号/重复/重连，MID-360 IMU 全部通过）。

21:37 在 Jetson 时间已经正确后执行 `systemctl --user restart micro-ros-agent`。ESP32 没有重启（`boot_id` 3698288053
不变，`reconnect_count` 0 → 1），重新对时后 header 与 Jetson 时间一致（1792067853 对比 1792067853），轮速
0/0。重启前确认：没有导航容器，`/cmd_vel_command` 只有网页遥控（解锁前的待命状态）一个发布者，`/cmd_vel`
只有补偿器一个发布者。

影响：导航不依赖 ESP32 时间戳（`/odom` 来自 FAST-LIO；2026-10-14 冷启动后的自动定位和导航都正常），但
`/wheel_ticks`、`/wheel/odom` 的时间戳在冷启动后是错的。修复方案见 #95（代理等待首次 NTP 同步后再启动）。
根本解决是固件在偏差超过约 1 s 时直接跳到正确时间。#95 完成前，每次冷启动、Jetson 校时完成后重启一次
`micro-ros-agent`。

## 仍未覆盖

- `observation_parity` 的观测单位和归一化与训练的对比（本次只验证传输：频率、延迟、丢包、重复）。
- 冷启动时钟顺序（#95）。
- `/odom`（FAST-LIO）的传输统计：需要在导航栈运行时录制。
