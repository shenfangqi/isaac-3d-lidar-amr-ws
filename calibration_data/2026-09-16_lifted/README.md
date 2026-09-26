# 2026-09-16 Carbot 架空实测数据

- 车体状态：两侧履带架空，由用户现场确认。
- 固件接口来源：`shenfangqi/2wheel-carbot-esp32`，提交
  `63dd45c7876212db8d8a387c045f25c2178eb2f0`。
- Jetson：`ubuntu`，ROS Domain 0，micro-ROS Agent UDP 8888。
- ESP32 boot ID：`549633487`。
- 时间同步：`/carbot/status.time_synchronized=true`。
- 初始电池读数：约 `7.776 V`。
- 实体急停：车辆未安装标准急停。架空持续 `0.05 m/s` 指令期间，人工切断
  ESP32 供电后 ROS 数据约在运动开始 1.3 秒后中断，现场目视确认履带停止。
  重新上电后现场确认履带未动，ROS 显示新 boot ID `2491872445`、命令源为 0、
  左右累计 ticks 均为 0。该开关只可作为低速受控试验的临时人工断电保护，
  最终安全验收仍需直接切断驱动动力或硬件使能的标准急停。
- 载荷与地面：架空测试不适用；落地试验必须另行填写。

## 已验证

- `/wheel_ticks` 约 50 Hz，静止 15 秒左右 tick 增量均为 0。
- 单次 `0.05 m/s` 后 watchdog 停车：首个 tick 变化约 73 ms，最后变化约
  586 ms，随后静止；已重复发送零指令。
- 正反直行与正负原地转向的 tick 符号均符合车辆坐标约定。
- Jetson 上唯一 `carbot_wheel_odometry` 发布 `/odom` 与
  `odom -> base_footprint`。
- 人工切断 ESP32 供电可停止架空履带；切断后 `/wheel_ticks` 离线，故停止结论
  来自现场目视而不是断电后的编码器数据。

## 主要发现

- 低速左右不对称显著。前进时 M3/左履带在 `0.01–0.03 m/s` 基本无法持续
  启动，到 `0.05 m/s` 才稳定；反向约从 `0.02 m/s` 起可持续转动。
- 静止 IMU 447 样本的加速度模长中位数为 `96.590896 m/s²`，约 `9.8495 g`。
  固件驱动已经输出 m/s²，ROS 发布层又乘一次标准重力；修复并重刷前不得融合。
- `forward_0p05_2s_bag_v2` 之前的正向 bag 含两个孤儿 odom 进程，不作为 odom
  验收证据；后续 `*_single`、deadband 正/反向 bag 使用单一 owner。
- `/wheel_ticks` 与 IMU 平均频率约 50 Hz，但存在成对的跳序/重复帧和间歇性发现；
  `/carbot/status.reconnect_count` 在本轮由 18 增至 103。累计 tick 去重后仍可积分，
  但在完成 Wi-Fi/micro-ROS 稳定性诊断前，不能通过 observation parity 门禁。

原始 CSV 与 rosbag 未平滑。快速混合方向的 `deadband_sweep_bag` 只用于诊断，
正式死区结论以拆分后的 `deadband_forward_*` 与 `deadband_reverse_*` 为准。


稳态 ESP32 遥测中位数如下；M1 正号、M3 负号对应车辆前进：

| cmd linear.x | M1 actual RPM | M3 actual RPM | M1 PWM | M3 PWM |
|---:|---:|---:|---:|---:|
| +0.01 m/s | +1.93 | 0.00 | +1.5 | -4.0 |
| +0.02 m/s | +7.69 | 0.00 | +1.0 | -8.0 |
| +0.03 m/s | +11.54 | 0.00 | +1.0 | -11.0 |
| +0.05 m/s | +19.23 | -15.38 | +2.0 | -6.0 |
| -0.01 m/s | -3.85 | 0.00 | 0.0 | +4.0 |
| -0.02 m/s | -7.69 | +7.69 | -1.0 | +1.0 |
| -0.03 m/s | -11.54 | +7.69 | -1.0 | +4.0 |
| -0.05 m/s | -19.23 | +15.38 | -2.0 | +6.0 |
