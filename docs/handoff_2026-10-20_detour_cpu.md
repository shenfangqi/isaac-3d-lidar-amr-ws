# 交接文档：绕障模式（#123）与 Jetson CPU（2026-10-20）

下一个会话从这里继续。用户规则、Jetson 连接、定位流程等通用内容见 `skills/isaac-amr-project-state/references/` 和以前的交接文档；这里只写 10-18 到 10-20 这三天的新内容。

## 0. 当前状态（一眼看）

| 项目 | 状态 |
|---|---|
| main | `7c92657`（#130 已合并） |
| Jetson bringup | `4e68df7`（#129），**落后 #130 一步，需要部署** |
| Jetson `carbot-ros2` | `carbot_hardware_cpp` + 更新后的 `carbot_hardware`（来自 `0780b8c`），手动复制并编译；`carbot-mid360.service` 已重启，C++ IMU 转换在运行 |
| Jetson 功耗模式 | MAXN_SUPER（`sudo nvpmodel -m 2`，用户执行，重启后保留） |
| Jetson | 用户已关机；开关机由用户操作 |
| Issue #123 | 仍 OPEN（实车验证大部分通过，还差沙发角无箱子一项） |

## 1. 下次开机后先做（按顺序）

1. **部署 #130**：`scripts/deploy_bringup_to_jetson.sh`（预览）→ 导航停着时 `--apply` → `--check`。只涉及 `isaac_3d_lidar_bringup`。导航若在跑，先 `scripts/stop_real_robot_navigation_rviz.sh`。
2. **复制更新后的录制脚本**：`output/issue1_robot/lio_rec/nav_rec.sh`（本地，未提交，已加 `/scan_near` 和 `/local_costmap/costmap_raw`）→ Jetson `~/Projects/isaac_ros-dev/tmp_issue1_bags/`。10-20 复制时 Jetson 已关机，没复制成功。
3. **提醒用户测沙发角（不放箱子）**——用户明确要求下次提醒。车从沙发西南角（约 (3.07, 3.12)）经过，就是 10-19 卡住的地方。目的：看 RPP 拒绝转弯时到底撞的是什么（近场点、局部代价地图）。
4. **在车上确认途中换目标**：车行驶或等待中在 RViz 发新目标，车应立即改去新目标（#130）。
5. 有时间再做：同一位置的新旧配置 CPU 对比（见 §4）。

## 2. 绕障模式现在的规则（`navigate_to_pose_detour_real.xml`，`--keepoff-detour` 才启用）

- 行驶中每次 tick 检查路线前方 4 m（`TruncatePathLocal` + `IsPathValid`）。全局代价地图在绕障模式用 `robot_radius: 0.130`（`carbot_obstacle_detour_real.yaml`），只看路线中心是否落在致命/内切格，静态墙不会触发。
- **路线被挡**：立即停车 → 等 3 s → 畅通则沿原路线继续；仍被挡则规划绕行（不后退）；规划不出就停着，每 3 s 检查原路线或再规划。**不计次数，不中止。**
- **RPP 拒绝**（路线畅通但"会撞"）：等 2 s 重试一次 → `reverse_escape` 后退 0.10 m → 等 2 s → 重新规划。**每个目标最多 3 次**；之后每 3 s 先重新规划（不后退，规划失败保留当前路线）再试。
- **目标变化**（途中发新目标）：`GlobalUpdatedGoal` 检测到后停下一切，从新规划开始（#130）。
- 初始规划失败仍中止；其余情况目标不会自己中止，只能到达或操作员取消。
- 取消当前目标的命令（车停着时才在 Jetson 上运行）：
  `docker exec carbot-nvblox bash -c 'source /opt/ros/humble/setup.bash; ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"'`

## 3. 10-20 实车测试结果（`4e68df7`，组合进程 + C++ IMU）

| 场景 | 结果 | bag（Jetson `tmp_issue1_bags/`） |
|---|---|---|
| 箱子在 1.1 m 宽过道中间 | 停车 10.7 s（绕不过去）→ 挪开后规划通行 → 人挡 3.2 s → 绕开 → 到达 | `route_check_06` |
| 箱子在沙发角路线上 | 停 3.0 s → 绕行 → 到达 | `sofa_corner_01` |
| 人慢速走过 | 停 2.9 s → 原路线（无新规划）→ 到达 | `person_cross_01` |
| 人快速走过 | 不停（禁区确认要约 0.5 s）→ 到达 | 同上 |
| 箱子离墙 0.8 m | 约 3 m 外停 → 一次"假畅通"多停 3 s → 绕行 → 到达 | `wall08_01` |
| 箱子在沙发角附近，绕行路线穿过箱子与沙发角之间的缝 | **卡住**：RPP 拒绝，3 次后退都重新规划出同一条缝 → 等待；新目标被忽略（#130 已修）；最终取消 | `detour_02` |

前 5 轮：后退 0 次，FAST-LIO 故障 0 次，位姿无 >5 cm 跳变，行驶时负载约 5–6。

**未解决：沙发角那条缝为什么 RPP 不让过。** 车停在 (3.15, 3.00)，朝向 123°；绕行路线经过 (2.98, 3.28)、(2.76, 3.68)，离沙发角约 0.27 m。可能撞的是近场看到的箱子本体，也可能是转弯扫掠。没录近场和局部代价地图，所以没定论。

**考虑过但没做：** 绕障全局代价地图半径改 0.21（车角扫掠半径）。否决原因：路线检查和规划都从车当前位置开始，车中心离墙 <0.21 m 时会永远"被挡"且规划失败（死锁，车也不会后退）；而这次路线离沙发角 0.27 m，改了也未必挡得住。

其他观察：箱子在 3–4 m（禁区标记上限 4 m 附近）时禁区时有时无，会多停一次；可以以后优化。

## 4. Jetson CPU

READY 静止，6 核占用（`top` 15–60 s）：

| 阶段 | 占用 | 说明 |
|---|---|---|
| 最初（25W） | 69% | |
| MAXN_SUPER | 60% | CPU 1.344 → 1.728 GHz |
| #127（定位节点 READY 后休眠） | 58% | |
| #128 + #129（C++ IMU、禁区优化、Nav2 组合进程；修崩溃） | 58–59% | 见下 |

#128 后各进程：IMU 44% → 9%，Nav2+AMCL+map_server 约 73% → 50%（11 进程 → 1 容器），禁区 39% → 24%，合计约省 0.75 核；但运动守护 11% → 28%、reverse_escape <7% → 13%（#129 改回不销毁订阅的代价），FAST-LIO 26% → 40%、雷达驱动 29% → 37%（没改代码，可能是测量位置/场景不同）。**总数没降**，负载从 10–13 降到 5–6，行驶中 FAST-LIO 没再出故障。

还能做（都需用户决定）：
- **同一位置新旧对比**：`imu_adapter:=python`（`carbot-mid360.service` 的 launch 参数）和 `nav2_composition:=false`（导航 launch 参数）可以分别回退，用来对比。
- **运动守护 28%**：释放后可以安全休眠或退出进程。注意：rclpy 多线程执行器里**不能** `destroy_subscription`（#127 就是这样崩的）。
- 定位管理器 READY 后仍 24%、near_field 19%、static_map_scan_filter 15%、wheel_odometry 14%、DDS 锚点 13%（都是 Python）。
- polkitd 尖峰来自每次 SSH 登录（约 0.26 s CPU/次）；建议在 `~/.ssh/config` 给 `isaac-jetson` 加 ControlMaster，**用户还没同意**。
- 重启 `carbot-mid360.service` 时旧雷达驱动退出时段错误（exit -11）一次，新驱动正常；以前是否也这样没查。

## 5. 部署方法备忘

- **bringup**：`scripts/deploy_bringup_to_jetson.sh [--apply|--check]`。
- **carbot_hardware / carbot_hardware_cpp**（部署脚本不管）：先比对 Jetson `~/Projects/carbot-ros2/src/carbot_hardware/` 要改的文件与改动前版本的 sha256（10-20 三个文件一致），备份到 `carbot-ros2/.deploy_backup/<名字>-<时间>/`，复制源码，`source /opt/ros/humble/setup.bash && colcon build --packages-select carbot_hardware_cpp carbot_hardware`，导航停着时 `systemctl --user restart carbot-mid360.service`，确认 `/mid360/imu/data_raw` 200 Hz、日志 "gyro z bias ready"。10-20 备份：`.deploy_backup/cpu128-20261020-214239/`。
- 停导航：`scripts/stop_real_robot_navigation_rviz.sh`（会恢复网页遥控为未解锁）。

## 6. 实车测试流程（必须遵守）

- **车行驶中不要在 Jetson 上启动任何新的 ROS 进程**（录制器、诊断脚本、`ros2` 命令）。10-19 三次 FAST-LIO "一帧位姿跳变" 故障都发生在新进程启动后 3–6 s。行驶中只看 `docker logs`。
- 录制：READY 后，先从日志确认 `automatic localization: READY`，再 `./nav_rec.sh start NAME`（轻量，不录原始雷达/IMU，正常优先级），静止 20–25 s 无告警后明确告诉用户"可以发目标"。用户常常提前发目标，要说清楚。`lio_rec.sh`（全量）会挤崩 FAST-LIO，导航时不要用。
- bag 只在车停着、录制停止后读（`docker exec -i carbot-nvblox ... python3 -`，rosbag2_py）。
- Jetson 刚开机时 SSH 第一次常报 "No route to host"，等几秒重试。

## 7. 仿真工具（`output/`，未提交）

- `output/route_check_sim/`：`nodes.sh start|stop`（禁区、近场、reverse_escape 三个节点，pid 文件）；`series.sh TAG 场景...`（环境变量 `BT` 指定行为树副本，`PASS_GAPS="1.0 0.8"`）；参数 `nav2_detour_near_stocktree.yaml`。启动：
  `ISAAC_WEBRTC=1 START_RVIZ=0 LOCALIZATION_MODE=ground_truth NAV2_PARAMS_FILE=/workspace/ros-humble/isaac_3d_lidar_amr_ws/output/route_check_sim/nav2_detour_near_stocktree.yaml ./start_nav_all.sh`（健康检查 "Carbot footprint" 那一项会失败，属预期）。
- **Humble bt_navigator 对同名行为树文件不重新加载**：仿真不重启时，每次改树都复制成新文件名。
- `output/route_check_sim/preempt.py`：途中换目标测试。
- `output/cpu_work/`：`imu_parity.sh`（Python/C++ IMU 对拍）、`keepoff_bench.py`（禁区新旧对比）、`real_launch_probe.sh` + `nav_real_nolio.launch.py`（离线跑实车 launch，去掉 FAST-LIO；改了 launch 要重新生成副本）。
- 验证脚本 `scripts/validate_dynamic_obstacle.py` 新增 `--behavior-tree`、`--route-check`、`--expect-pass`、`--cancel-after-stop-s`。

## 8. 这几天踩过的坑

- rclpy 多线程执行器里 `destroy_subscription` / `TransformListener.unregister()` 会和执行器线程竞争 → "cannot use Destroyable because destruction was requested"，节点崩溃（#127 → #129）。单线程执行器里没问题。
- Nav2 组合进程：组件只读传给它的参数，而服务器内部创建的代价地图子节点只读进程全局 `--params-file`；两种都要给，否则代价地图静默用默认值（footprint `[]`、膨胀 0.55）。
- launch 里 `PythonExpression` 求值成 `'True'/'False'`，不是 `'true'`；`OpaqueFunction` 要放在它用到的 `DeclareLaunchArgument` 之后。
- Humble：同一棵树的新目标只替换 `{goal}`，不重启树；自定义树必须自己检查目标变化。
- Smac2D 路线点朝向全是 0；`IsPathValid` 用多边形时按朝向查，会被墙误判。
- 每次 SSH 登录触发 polkitd 约 0.26 s CPU。

## 9. PR 一览

#124 路线检查提前停车 · #125 次数用完后退一次（已被 #126 取代）· #126 路线被挡不计次数、RPP 后退最多 3 次 · #127 READY 后定位节点休眠 · #128 C++ IMU、禁区优化、Nav2 组合进程 · #129 修多线程销毁订阅崩溃 · #130 途中换目标、次数用完后重新规划。全部已合并；#130 未部署。

## 10. 别忘了

- 下次换硬件时装 Jetson RTC 纽扣电池。
- 固件 PR shenfangqi/2wheel-carbot-esp32#13（PID 换向）仍待合并。
