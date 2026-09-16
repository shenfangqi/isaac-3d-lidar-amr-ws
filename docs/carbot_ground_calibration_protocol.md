# Carbot 落地侧滑与有效运动学标定

## 前置门槛

落地前必须确认人工断电装置能停止驱动，并由专人在开关旁值守。手机网页的红色
“停止”按钮属于软件命令，不是独立急停，不能替代人工断电装置。当前车辆未安装
标准物理急停；架空试验已确认切断 ESP32 供电可使履带停止且重新上电不会自启，
因此该开关仅限本协议的低速、短距离、人工监督标定。开关必须始终触手可及，任何
异常立即断电。该临时措施不满足无人或自主导航验收，最终仍需直接切断驱动动力或
硬件使能的标准急停。只运行 Jetson 的
`micro-ros-agent.service` 与 `carbot-wheel-odometry.service`；物理 ROS 图中必须恰好
有一个 `/cmd_vel` 发布链路、一个 `/odom` 发布者和一个
`odom -> base_footprint` 发布者。不得启动 Isaac 仿真或 Nav2 自动导航。

## 试验矩阵

每个代表性地面与载荷至少重复三次：

1. 正向、反向各 1 m，速度 `0.05 m/s` 与 `0.10 m/s`。
2. 左、右原地转向各 90° 与 360°，角速度先用 `0.20 rad/s`。
3. 每次从完全静止开始，试验间等待履带和电机完全停止。
4. 直线用钢卷尺测 `base_footprint` 同一点的实际位移；转向用地面基准线或外部
   定位测实际 yaw。不要用 `/odom` 反过来充当真值。

同步记录：`/cmd_vel`、`/wheel_ticks`、`/odom`、`/tf`、`/carbot/status`、
`/imu/data_raw`、`/battery_state`，并保存 ESP32 `/telemetry` CSV。记录地面材料、
载荷、电池电压、轮胎/履带状态、固件提交和急停状态。

## 指标定义

- 直线纵向滑移：`1 - 实测距离 / 编码器预测距离`。正值表示编码器预测距离大于
  实际落地距离。
- 有效半径：`基线半径 × 实测距离 / 编码器预测距离`。
- 转向 yaw 增益：`实测 yaw / 编码器预测 yaw`。
- 转向滑移：`1 - abs(yaw 增益)`。
- 有效轮距：`abs(右履带距离 - 左履带距离) / abs(实测 yaw)`。

先生成不覆盖已有文件的 CSV 模板：

```bash
python3 scripts/analyze_carbot_ground_trials.py trial.csv --create-template
```

每次试验填入累计 tick 的起止差和外部实测值后计算：

```bash
python3 scripts/analyze_carbot_ground_trials.py trial.csv
```

侧向滑移不能由左右编码器和单个终点距离单独辨识；需要 LiDAR/视觉/全站仪等外部
轨迹真值。没有外部轨迹时，只把上面的纵向与转向指标回填到有效半径、有效轮距及
domain randomization，不声称已获得完整 lateral slip 模型。
