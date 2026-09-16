# Carbot 阶段 G 仿真验证报告

日期：2026-09-16
分支：`codex/carbot-isaac-sim-adaptation`

## 结论

Carbot 的 Isaac Sim、Isaac Lab、ROS 2、MID360、nvblox、保存地图和
Nav2 仿真基线通过阶段 G 自动验证。场景中唯一的活动机器人位于
`/World/Carbot`，articulation 和 RTX MID360 分别位于：

```text
/World/Carbot/base_footprint
/World/Carbot/base_footprint/base_link/lidar_link/mid360_rtx
```

旧 `/World/Robot`、`/World/ROS2_Carter_Graph` 和
`/World/ROS2_LidarRTX` 均已停用。仿真结果不代表真机动力学或安全验收
完成。

## 自动测试

- `carbot_description` 的 colcon 测试：42 项通过，0 错误、0 失败、0 跳过。
- Isaac Lab Phase F 契约测试：8 项通过。
- Isaac Lab 单环境 20 步加载、reset、action/observation 冒烟测试通过。
- `git diff --check` 通过。

## Isaac Lab 单体运动门禁

| 门禁 | 结果 | 关键结果 |
| --- | --- | --- |
| G1 零命令与 watchdog | 通过 | 零漂移 0 m；0.51 s 触发；最终速度 0 m/s |
| G2 正向/反向 | 通过 | +0.729/-0.729 m；回位误差 0 m；偏航漂移 0° |
| G3 正负角速度 | 通过 | +98.98/-98.96°；对称误差 0.02° |
| G4 轮组映射与饱和 | 通过 | 每侧六轮一致；曲率保持；饱和比例 0.64633 |
| G5 1 m 直行 | 通过 | 车体与有效半径推算距离均为 1.000 m |
| G6 转弯几何 | 通过 | 90.24° 和 360.16°；位置漂移 0 m |
| G7 接触与稳定性 | 通过 | 无穿透/悬空/侧翻；roll/pitch 最大值 0° |

G7 中接触解析的瞬时同侧轮速差只作信息记录。当前
`ideal_kinematic_tracked_differential` 是未完成真机标定前的理想运动学
基线，不能证明真实履带侧滑或摩擦已经校准。

## ROS、MID360 和导航回归

- `/odom`、`/joint_states`、TF 和 `/clock` 实测约 49～50 Hz。
- `/livox/lidar`、`/livox/lidar_nvblox` 和 `/scan` 实测 9.82 Hz。
- padded 点云保持 `1000 x 40`。
- `map -> odom -> base_footprint -> base_link -> lidar_link` 可解析。
- `warehouse_v3` 保持 `417 x 424`、`0.05 m` 分辨率及既有栅格统计。
- 直线 Nav2 目标成功：0 次恢复，终点误差 0.094 m，停止速度为零。
- 带转向 Nav2 目标成功：0 次恢复，终点误差 0.096 m、3.04°，停止速度为零。
- 最终 WebRTC、Isaac Sim、nvblox 和 Nav2 全栈启动健康检查全部通过。

## 未由仿真关闭的真机门禁

以下项目仍以
[`hardware_calibration_backlog.yaml`](../isaac_lab/carbot_env/hardware_calibration_backlog.yaml)
为准：Jetson wheel tick 解码与里程计唯一归属、履带侧滑、
执行器延迟/死区/制动、质量/重心/惯量、摩擦、IMU 与 MID360
外参和时间同步、实体 watchdog/急停、观察量一致性以及真机导航验收。
有效半径 `0.02175 m` 和有效轮距 `0.254 m` 是确认基线；1 m 与
90°/360° 真车试验属于发布验收复核，不是缺失标定阻塞项。

开始真机测试前必须关闭仿真 ROS 图，架空履带并确认实体急停；在真实
watchdog 和唯一 odom/TF 发布责任得到验证前不得发送非零速度命令。
