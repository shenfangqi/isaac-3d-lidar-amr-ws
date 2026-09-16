# Carbot 开发交接记录（2026-09-16）

## 当前 Git 状态

- 分支：`codex/carbot-isaac-sim-adaptation`
- 最新提交：`332cd9d docs: clarify confirmed kinematic baselines`
- 阶段 G 提交：`f8f911d test: complete carbot simulation validation`
- 工作树中的正式项目文件已提交。
- `.codex_tmp/` 和 `docs/topology_build/node_modules` 是受保护的用户未跟踪目录，
  不得删除或加入提交。

## 已完成状态

阶段 A～G 的仿真基线已完成：

- 参数单一来源、Carbot URDF/Xacro 和本地 USD articulation 已建立。
- 场景中的唯一活动机器人是 `/World/Carbot`。
- articulation：`/World/Carbot/base_footprint`。
- RTX MID360：
  `/World/Carbot/base_footprint/base_link/lidar_link/mid360_rtx`。
- 旧 `/World/Robot`、Carter ROS2 图和旧 RTX 雷达图已停用。
- Isaac Sim 使用受限的理想履带差速运动学；Isaac Lab 使用相同的
  `[linear.x, angular.z]` 安全边界、加速度限制、耦合饱和和 watchdog。
- sim/real launch、时间源、DDS 和 odom 发布责任已经分开。
- 原 `warehouse_v3`、MID360、nvblox、RViz 和 Nav2 接口保持兼容。

## 已通过验证

- `carbot_description`：42 项测试通过，0 失败、0 跳过。
- Isaac Lab Phase F：8 项契约测试和 20 步环境冒烟测试通过。
- Isaac Lab G1～G7 全部通过：watchdog、正反直行、正负转向、六轮映射、
  耦合饱和、1 m、90°/360°、接触和稳定性。
- MID360、padded cloud 和 `/scan` 实测约 9.82 Hz；padded cloud 为
  `1000 x 40`。
- ROS 类型、QoS、frame、TF、约 50 Hz odom/joint/clock 均通过。
- Nav2 直线和带转向目标均成功，0 次恢复，停止速度为零。
- WebRTC、Isaac Sim、nvblox 和 Nav2 完整启动健康检查通过。
- 详细结果：`docs/carbot_phase_g_validation_report.md`。

## 已确认且不再属于缺失标定阻塞项

- 物理履带中心距：`0.225 m`。
- 有效半径：`0.02175 m`，确认基线。
- 有效轮距：`0.254 m`，确认基线。
- 1 m 和 90°/360° 真车测试仍需执行，但属于发布验收复核，不是重新标定
  前禁止使用的参数。

## 仍然阻塞真机/策略发布的项目

- Jetson `/wheel_ticks` 解码和唯一 `/odom`、`odom -> base_footprint` 责任。
- 履带侧滑及不同地面/载荷下的变化范围。
- 执行器延迟、死区、制动和左右不对称。
- 质量、重心、惯量和履带摩擦的真机证据。
- IMU 和 MID360 精确外参、Livox 原点 O 以及时间同步。
- 实体 watchdog、独立急停、观察量一致性和真机 AMCL/Nav2 验收。
- 权威清单：`isaac_lab/carbot_env/hardware_calibration_backlog.yaml`。

## 下一步：先做架空测试

开始前必须关闭仿真 ROS 图，牢固架空两侧履带，并让人员持有实体急停。
不要在 watchdog 和急停验证前发送持续非零速度命令。

建议顺序：

1. 验证实体急停和 500 ms watchdog。
2. 分别低速驱动左右履带，确认方向、映射和编码器符号。
3. 标记轮子，复核 `/wheel_ticks` 50 Hz、重启/boot ID 和每圈 1560 counts。
4. 启动 Jetson 里程计，确认物理 ROS 图中恰好一个 `/odom` 和
   `odom -> base_footprint` 发布者。
5. 测量正反转无负载死区、命令到 RPM 响应延迟、不同档位稳态 RPM、PWM、
   电池电压和左右差异。
6. 静止测量 MID360 原点 O、支架 X/Y/Z/RPY，检查点云地面/墙面方向。
7. 测量 IMU 静止零偏、重力方向、坐标轴和时间同步状态。
8. 架空项目通过后，再制定低速落地的侧滑、制动、1 m 和 90°/360°
   验收计划。

架空阶段建议同步记录：

```text
/cmd_vel
/wheel_ticks
/odom
/tf
/tf_static
/imu/data_raw
左右目标 RPM
左右实际 RPM
左右 PWM
电池电压
```

原始 rosbag/CSV 不要预先平滑，并记录固件提交、Jetson 提交、电池电压、
测试载荷、时间同步模式和实体急停状态。

## 当前运行状态

交接记录创建时，`isaac-sim`、`isaac-ros-nvblox` 和 `ros2-dev-humble`
容器仍在运行，RViz2 已退出。开始真机 DDS/Jetson 联调前，必须运行
`/home/shenfq/projects/ros-humble/stop_nav_all.sh`，确认仿真 ROS 图完全关闭。

## 下一会话建议指令

```text
请读取 docs/carbot_2026-09-16_handoff.md 和
isaac_lab/carbot_env/hardware_calibration_backlog.yaml，继续 Carbot 真机架空联调。
先关闭并确认仿真 ROS 图，再只读检查 Jetson、micro-ROS Agent、ESP32 topic、
实体急停和 watchdog；在安全门槛通过前不要发送非零速度命令。
```
