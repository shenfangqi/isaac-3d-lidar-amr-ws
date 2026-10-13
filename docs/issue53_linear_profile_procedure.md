# Issue #53：直线运动标定的实车步骤

目的：得到一份直线（平移）运动 profile，交给平移守卫（`localization_translation_guard.LinearProbeGuard`）使用。做法和 2026-10-06 的旋转 profile 一样：实车采集 → 离线分析出 ESTIMATED → 外部核对一次停车 → 人工决定 ACCEPTED → `scripts/check_linear_profile.py` 通过。分析脚本**永远不会**自己写出 REVIEWED 或 ACCEPTED。

## 场地和人员

- 位置：客厅中间南北向的空地，地图上 x ≈ 2.8，y ≈ 1.1–6.9（导航地图里最长的一条直线，两侧至少 0.4 m 余量）。起点约 (2.8, 2.6)，**车头朝北**，前方约 4 m、后方约 1.5 m。旁边就是 10-09 activate_probe_07/08 的停车位置。
- 开始前清空这条线，两侧各留半米以上。脚本**看不到障碍物**，只按里程计限制距离。
- 操作员守在车旁，随时能切断 ESP32 电源。这台车没有标准的物理急停，网页遥控的停止按钮在连续指令下不起作用（`hardware_calibration_backlog.yaml`）。

## 前提（由我在 Jetson 上核对）

- 底盘、`cmd_vel_compensator`、FAST-LIO 和 `fast_lio_base_adapter` 在运行，所以 `/odom`、`/carbot/status`、`/wheel_ticks` 和锁存的 `/localization/emergency_stop`（false）都有数据。
- Nav2 没有激活，`/cmd_vel_command` 上没有别的发布者。脚本的预检会核对这一点，录包程序只订阅、不发布，不影响预检。
- 电量不低。

## 采集（在 Jetson 上）

先 `scp scripts/jetson_linear_profile_capture.py isaac-jetson:/tmp/`。每一步开始前先开录包，结束后再停：

```bash
ros2 bag record -o <日期>_issue53_linear_stops_01 /cmd_vel_command /cmd_vel /odom /wheel_ticks \
  /carbot/status /tf_static
python3 /tmp/jetson_linear_profile_capture.py --mode stops --operator-present
```

- `stops`：默认 0.08 m/s，前进 0.40 m → 停 3 s → 后退 0.40 m → 停 3 s，共 4 对。每对都先前进，所以后退时走的是刚走过的路。
- 然后另开一个录包做 `watchdog`：`--mode watchdog --operator-present`，默认 4 次，每次走 2 s（约 0.16 m）后完全停止发指令，测底盘自己多久停下。前进和后退交替。
- 脚本每 50 ms 检查一次，以下任一情况立即发 3 s 零速度并退出：急停信号、底盘断开或阻塞、里程计过期、偏离直线超过 0.10 m、航向变化超过 10°、比目标多走 0.20 m、离起点超过 0.80 m、累计路程超过 5 m。
- 参数有硬上限，超出就拒绝运行：速度 ≤ 0.10 m/s，单段 ≤ 0.60 m，每次 watchdog 试验走的距离 ≤ 0.30 m。
- 前进时低于约 0.05 m/s 车不能持续移动（死区），所以默认用 0.08 m/s。
- **不要在 Jetson 上录点云**：2026-10-13 在 Jetson 上录 `/fast_lio/cloud_registered_body` 时，`/odom` 出现了最长 1.9 s 的中断，采集中止，随后 FAST-LIO 锁定了故障，要重启整个栈。车动过的点云要换别的方式录，比如在工作站上通过网络录，这个还没验证。

## 分析（在工作站的容器里）

把录包拷回 `calibration_data/`，然后：

```bash
python3 scripts/analyze_linear_profile.py calibration_data/<stops 录包> calibration_data/<watchdog 录包> \
  --probe-speed 0.08 --output-json docs/evidence/issue53_linear_profile_<日期>.json \
  --output-md docs/evidence/issue53_linear_profile_<日期>.md
```

- 每次运动给出：反应时间（停发指令 → 开始减速）、停止尾程、watchdog 停车时间、偏离直线的距离、航向变化、实际速度和指令的比例。全部用里程计位置计算，不用 FAST-LIO 的速度字段（10-08 发现它在运动后会偏）。
- 前进和后退各至少 3 次有效停车、至少 3 次 watchdog 试验，才写出 ESTIMATED 的 profile，取最差值；否则只写报告，退出码为 1。
- "停稳"的门槛是 0.02 m/s（FAST-LIO 静止时的速度噪声有 0.007–0.013 m/s）。
- 合成数据的端到端检查：写出的 profile 哈希和本车参数一致；ESTIMATED 状态被 `check_linear_profile.py` 拒绝，改成 ACCEPTED 后通过（停车外延 0.177 m）。

## 验收

1. 外部核对：至少一次停车用卷尺或地面标记量实际距离，和里程计对比。
2. 用户看过报告后明确决定 ACCEPTED，再把 `status` 改成 ACCEPTED。
3. `python3 scripts/check_linear_profile.py <profile.json>` 通过。
4. 之后才能用 `--localization-translation guarded --linear-profile <json>` 启动带平移的定位。

## 2026-10-13 的结果

已验收：`docs/evidence/issue53_linear_profile_accepted_2026-10-13.json`，记录在 `docs/evidence/issue53_linear_profile_review_2026-10-13.md`。前进时每米右偏约 6°，是已知问题，验收时已记录。
