# 降低 Jetson 导航 CPU：IMU 转换 C++、禁区节点、Nav2 组合进程（离线，2026-10-20）

## 背景

2026-10-19 实车（MAXN_SUPER，`cdf14cf`，绕障模式 READY 静止）：6 核占用 58%。新增负载的突发多次在 FAST-LIO "一帧位姿跳变" 故障之前出现。最大的几项：IMU 转换 44%、禁区节点 39%、Nav2 每个空闲进程 5–10%（约 0.6 核）。之后还要跑强化学习。

## 1. IMU 转换改为 C++（`carbot_hardware_cpp`）

- `mid360_imu_adapter`（C++）：节点名、参数、话题、偏置估计（窗口中位数）、协方差回退、时间戳修正与 Python 版相同；`imu_units.hpp` 对应 `imu_units.py`。
- `mid360_stack.launch.py` 默认启动 C++ 版；`imu_adapter:=python` 恢复 Python 版。
- 对拍：4000 条合成消息（偏置预热、窗口滚动、运动/静止切换、NaN、带协方差），两版输出**逐字段完全相同**（4000/4000）。
- CPU（工作站，200 Hz，20 s）：Python 1.24 CPU 秒，C++ 0.22 CPU 秒（1/5.6）。按比例，Jetson 上约 44% → 8%。
- gtest 6 个，carbot_hardware 测试 56 个通过。

## 2. 禁区节点（`novel_obstacle_keepoff.py`）

- `clear()`：按每条光束实际长度生成采样点，不再生成"光束数 × 10 m"的掩码（结果逐位相同）。
- `densify_beams()`：插入光束数量按这对光束实际到达的距离计算，保证该距离处间距 ≤ 2.5 cm（原来按 10 m 计算，室内多插了约 2.5 倍）。这一项改变了清除的细节：20 次渲染的禁区地图与旧版**完全相同**，命中计数相同，"见过为空"标记 39183 格中差 1 格。测试 `test_beams_between_returns_are_filled...` 的间距断言改为"到达距离处"。
- 发布：用 `array('b')` 代替 17.7 万元素的 Python 列表（8.7 ms → 0.03 ms，序列化结果相同）；地图没变化时不重发，最多每 `unchanged_republish_s`（5 s）重发一次，减少全局代价地图整图重算膨胀。
- 基准（工作站）：`clear()` 12.1 ms → 3.2 ms（3.7 倍）。Jetson 预计约 0.39 核 → 0.1 核左右，待实测。
- Isaac（`output/route_check_sim/s9`）：temporary（2.41 m 外停车、撤走后原路线到达）、detour（2.29 m 外停车、绕行、不后退）、blocked（等待后取消）全部 PASS。

## 3. Nav2 组合进程（`nav2_composition`，默认 true）

- map_server、AMCL、controller/planner/behavior/bt_navigator/waypoint_follower/velocity_smoother 和两个生命周期管理器，作为组件加载进一个 `component_container_isolated`（`nav2_container`）。分支逻辑（绕障、等待重试、后退脱困树、近场、自动/手动定位）与独立进程相同；`bounded_recovery_enabled` 仍用独立进程。`nav2_composition:=false` 恢复独立进程。
- **关键坑**：组件只读传给它的参数，而服务器内部创建的代价地图子节点读进程全局的 `--params-file`。只传一种时，代价地图会悄悄用默认值（footprint `[]`、膨胀 0.55）。现在两种都传，容器全局参数文件按变体选择（近场、禁区覆盖只在对应模式下加入）。
- 离线试跑实车启动文件（去掉 FAST-LIO 引入的副本，`output/cpu_work/real_launch_probe.sh`）：10 个组件全部加载；默认组合、绕障组合下，组合与独立进程的局部/全局代价地图（footprint、半径、膨胀、观测源、图层）、RPP、行为树、`bt_loop_duration`、velocity_smoother 速度上限与输出话题 `/cmd_vel_command`、AMCL、map_server、autostart **全部一致**。Nav2 相关进程 11 → 2。
- 试跑中发现并修正了两个会让 Nav2 在车上起不来的错误（OpaqueFunction 早于参数声明；PythonExpression 求值为 `'True'`）。
- CPU 收益只在实车上能测（预计 0.3 核左右，未验证）。

## 上车验证清单

1. 部署 bringup（`deploy_bringup_to_jetson.sh`），并在 Jetson 的 `carbot-ros2` 工作区放入 `carbot_hardware_cpp` 和更新后的 `carbot_hardware`，`colcon build --packages-select carbot_hardware_cpp carbot_hardware`，重启 `carbot-mid360.service`。
2. 确认 `/mid360/imu/data_raw` 200 Hz、FAST-LIO 正常、日志有 "gyro z bias ready"。
3. `./start_auto_nav.sh --keepoff-detour` 到 READY；确认 `nav2_container` 中 10 个组件、生命周期激活。
4. READY 静止采样 CPU，与 58% 对比；行驶中看 FAST-LIO 故障是否再出现。
5. 绕障场景：通道放箱子、人走过、沙发角。
回退：`imu_adapter:=python`；`nav2_composition:=false`。
