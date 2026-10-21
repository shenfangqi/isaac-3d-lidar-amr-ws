# 交接文档：常驻事件录制、RPP 速度下限、IMU 间隔监测、RL 方向（2026-10-21）

下一个会话从这里继续。10-18 到 10-20 的内容（绕障规则、CPU 数据、部署方法、实车测试流程）见 `docs/handoff_2026-10-20_detour_cpu.md`，仍然有效；这里只写 10-21 的新内容和改变了的地方。

## 0. 当前状态（一眼看）

| 项目 | 状态 |
|---|---|
| main | `b249a54`（#137 已合并） |
| Jetson bringup | `e33f65b`（#136），`deploy_bringup_to_jetson.sh --check` = IN_SYNC（#137 不改 bringup） |
| Jetson `carbot-ros2` | `carbot_hardware_cpp` + `carbot_hardware` 的 IMU 相关 6 个文件 = main `b249a54`，手动部署；`carbot-mid360.service` 已重启，IMU 200 Hz，没有误报间隔。备份：`.deploy_backup/imugap137-20261021-233748/` |
| Jetson 导航 | 已停止；Jetson 关机由用户操作 |
| 常驻录制器 | 随导航启动（默认开），bag 在 Jetson `~/Projects/isaac_ros-dev/nav_events/` |
| 今天的数据 | `output/issue1_robot/nav_events/`（14 个事件 bag + 容器日志，未提交）；`output/issue1_robot/bags/preempt_01`（手动录制） |
| Issue #123 | 仍 OPEN；规则绕障开发基本冻结（见 §6） |
| Issue #132 | RL 局部避障提案，征求意见中，用户**还没最终决定** |

## 1. 下次开机后先做

1. 没有必须先做的部署，所有 PR 都已部署。
2. 正常用车即可。车停住、报错、卡住或定位故障时，录制器会自动写 bag；用户**不需要**再按场景手动录制（用户明确要求，见 §6）。
3. 如果再出现 FAST-LIO 静默或"卡住再追上"：先在对应的事件 bag 和日志里查有没有 `MID-360 IMU input gap` 这条 ERROR（§5）。
4. 每次用完车，把 `nav_events/` 同步回工作站：`rsync -a isaac-jetson:Projects/isaac_ros-dev/nav_events/ output/issue1_robot/nav_events/`。录制器超过 3 GB 或 200 个事件会删最旧的。

## 2. 今天合并的 PR

| PR | 内容 | 部署 |
|---|---|---|
| #130（10-20 合并） | 途中换目标立即生效；后退次数用完后定期重新规划 | 已部署，实车验证通过（`preempt_01`：换目标 0.2 s 后走新路线） |
| #133 | codex 提交 `57e602b` 把雷达外观尺寸写进了 `sensors.mid360`，导致 `extrinsics_hash` 变化，`start_auto_nav.sh` 报 `MOTION_PROFILE_REJECTED`。外观尺寸移到 `geometry.mid360_visual_proxy`，哈希恢复为 `7691c3c2…` | 只影响工作站上的校验 |
| #134 | 常驻事件录制器 `nav_event_recorder`（§3） | 已部署 |
| #135 | 录制器修复：重复过滤 30 s → 5 s；每个 bag 带上当时有效的 `/plan` 和目标状态；退出时写盘 | 已部署 |
| #136 | RPP `regulated_linear_scaling_min_speed` 和 `min_approach_linear_velocity` 从 0.04 改为 0.06 m/s（§4） | 已部署 |
| #137 | IMU 转换节点报告输入间隔（§5） | 已手动部署 |

另外：`output/issue1_robot/lio_rec/nav_rec.sh`（加了 `/scan_near`、`/local_costmap/costmap_raw`）已复制到 Jetson `tmp_issue1_bags/`，旧版是 `nav_rec.sh.bak-20261019`。有了常驻录制器，一般不再需要手动录制。

## 3. 常驻事件录制器

- 节点：`isaac_3d_lidar_bringup/nav_event_recorder.py`；参数：`config/nav2/carbot_event_recorder_real.yaml`；launch 参数 `nav_event_recorder:=true`（默认）。
- 平时只在内存里保留最近 30 s 的导航话题（序列化字节，不写盘）。话题和 `nav_rec.sh` 相同，另加 `/rosout` 和 `/navigate_to_pose/_action/status`；**不录原始雷达和 IMU**。
- 触发条件：
  - `/rosout` 里 ERROR 及以上的日志；
  - "Goal was canceled"（路线检查停车、途中换目标）；
  - 导航目标被中止（途中换目标时旧目标也算中止）；
  - 有目标但 20 s 内移动不到 5 cm；
  - `/localization/fault_reason` 非空，或 `/localization/emergency_stop` 变为 true。
- 每个 bag 覆盖第一个事件前 30 s 到最后一个事件后 10 s（最长到第一个事件后 60 s）。同一条日志 5 s 内只算一次。退出时未写完的窗口立即写盘。
- 输出：`nav_events/<UTC时间>_<原因>/bag/` + `event.json`（事件列表、各话题条数）。
- Jetson READY 静止时实测：单核 14.6%，内存 89 MB，系统总占用 59%（和之前的基线一样，在误差范围内）。用户决定先保持现状，暂不优化。
- 录制器的日志只在一个窗口的**第一个**事件时打印 `event ...`，后续事件合并进同一个窗口但不再打印。用 `docker logs` 过滤时，格式是 `[nav_event_recorder]: event ...`（`]` 后面有冒号）。
- 分析脚本（未提交，在 scratchpad 里写的，需要时重写）：在拒绝时刻把车身矩形放到 `costmap_raw` 上，模拟原地转 ±29° 和前进 5/10 cm，再用 `/scan_near`、`/scan`、禁区地图判断碰到的格子来自哪里。注意 `/rosout` 里录制器自己转述的日志会重复匹配。

## 4. 目标附近来回原地转（#136 已修）

- 现象（`20261021T133338Z_no_progress`）：车停在离目标 0.08–0.11 m 处（位置容差 0.10 m），原地 ±0.5 rad/s 来回转约 35 s，被进度检查中止。
- 原因：接近目标时 RPP 把速度降到 0.04 m/s（平滑后约 0.02 m/s），低于履带前进死区 0.05 m/s，车挪不进容差；原地转动时车中心挪动 1–2 cm，RPP 在"转向目标朝向"和"转向路线终点"之间来回切换。
- 修复：两个速度下限改为 0.06 m/s。`test_real_nav_safety.py` 要求两者都高于 `carbot_parameters.yaml` 里的 `hardware_response.track_deadband.forward_min_sustainable_mps`。
- 仿真 A/B（`output/route_check_sim/speed_floor_ab.py`，warehouse_v3，11 段随机路线）：0.04 和 0.06 都是 11/11 成功，0 次碰撞拒绝。仿真底盘没有死区，只能证明没有副作用。
- 部署后实车 3 个目标都正常到达。是否彻底修好，靠录制器以后有没有"目标附近 no_progress"事件来判断。
- 对 RL 的影响已经写在 #132 评论里：死区是"在控制链里补偿"还是"在仿真里建模"，是 RL 的待决问题。

## 5. FAST-LIO 静默故障（23:31，原因未定）

- bag：`20261021T143059Z_rosout_error`（合并了 11 个事件），日志 `output/issue1_robot/nav_events/carbot_nav_20261021_fault.log`。
- 当时车在一个地图上没有的障碍物旁边反复被 RPP 拒绝、后退（§6）。
- 原始雷达 `/scan_near` 一直连续；FAST-LIO 的输出（`/scan`、`/odom`）在 8 s 内卡了 4 次，每次约 2 s，每次都在 0.3 s 内补完积压（延迟从 1.9 s 降回 0.1 s）。第 2 次超过 2 s，`fast_lio_base_adapter` 判定静默，锁住运动、暂停 Nav2。FAST-LIO 进程一直活着，没有打印任何日志；系统 CPU 约 55% 空闲。
- 判断：补得这么快，不像算不过来，更像 FAST-LIO 在**等 IMU 输入**。10-18 在箱子旁转弯时也见过同样的"卡住再追上"（当时还是 Python IMU 转换）。
- #137 之后，IMU 转换节点在 `/livox/imu` 停顿超过 0.5 s 时打 ERROR：`MID-360 IMU input gap: X s without /livox/imu (header stamps advanced Y s)`。
  - 到达间隔长、时间戳间隔短：传输卡住（驱动到转换节点之间）；
  - 时间戳间隔长：源头丢数据；
  - FAST-LIO 卡住时**没有**这条日志：问题在转换节点之后（DDS 到 FAST-LIO，或 FAST-LIO 本身）。
- IMU 转换节点在主机上运行（`carbot-mid360.service`），它的 ERROR 通过 `/rosout` 进入容器里的录制器。主机的用户 journal 不持久，`systemctl --user status carbot-mid360.service -n 200` 只能看到最近的日志。

## 6. 用户的决定和工作方式（重要）

- **要泛化，不要按场景修**：用户明确反对"每个场景都让用户录一次数据再修"。修复要按几何和状态设计；验证用仿真批量场景；实车证据靠常驻录制器在正常使用中自动积累。只有仿真回答不了时才请用户专门录，并说明原因。
- **RL 方向（#132）**：保留 Nav2 的地图、定位和全局路线，避障、等待、绕行交给 RL 局部策略，外加不学习的硬否决层。issue 里有现状、资源、缺口和 P0–P5 阶段，并补充了两条评论：死区问题、"目标紧挨未知障碍物 → 反复摇摆"场景（含更正）。用户还没决定是否正式启动。
- **规则绕障基本冻结**：
  - "目标紧挨地图上没有的障碍物 → RPP 拒绝 → 后退 → 规划出同一条贴边路线 → 再被拒"这个循环，用户决定**不修**，留作 RL 场景。原因已查清：路线离障碍物格子约 0.17 m，车身左边缘只剩约 3 cm，RPP 的弧线检查认为左前角会蹭到，拒绝是正确的；规划器用 0.133 m 内切半径，和控制器标准不一致。
  - 10-20 交接里"沙发角（不放箱子）录制"不再单独做，由录制器自动积累。
- RPP 的停车距离、`footprint_padding`、碰撞时间窗口仍然只讨论、不改（`docs/handoff_2026-10-20_detour_cpu.md` 和以前的约定）。

## 7. 启动导航的注意事项（10-21 新发现）

- `start_real_nav.sh --automatic-activate --keepoff-detour` 只允许原地旋转定位：车旁 0.5 m 内有障碍物时会报 `obstacle remained too close for automatic rotation` 并退出。
- `start_auto_nav.sh --keepoff-detour` 允许前进 0.2–0.4 m，但要求现场有人输入 `y` 确认，**agent 不能替用户确认**。遇到上面的情况时请用户自己运行。
- 两个启动脚本共用一把锁：用户已经在启动时，agent 的启动会被拒绝（"Another real-navigation startup is already running"），这是正确的保护。
- 有一次硬件预检失败在 FAST-LIO 静止稳定性检查（`jetson_lio_stability_check.py`），而且这一项没有打印任何输出。原因没查（随后用户自己启动成功）。如果再出现，要先看这个脚本为什么没有输出。

## 8. 测试和工具备忘

- 描述包测试要在 `ros2-dev-humble` 里带 `PYTHONPATH=<workspace>` 运行；URDF 相关测试需要先 colcon 编译（`CARBOT_GENERATED_URDF` 由 CMake 提供），可以在 `/tmp` 里单独编译。
- bringup 全部测试：`PYTHONPATH=$W/src/carbot_nav_recovery:$W python3 -m pytest test`。
- 在 main 上本来就失败、与今天的改动无关的测试：
  - `carbot_description/test/test_carbot_phase_e.py::test_detour_parity_profile_matches_the_keepoff_overlay`
  - `isaac_3d_lidar_bringup/test/test_localization_motion_guard_node.py` 里 2 个隔离 domain 的往返测试（开发容器的 DDS 环境问题）
- `docker exec` 用 stdin 传脚本时必须加 `-i`，否则什么都不执行，也不报错。
- 仿真 A/B 脚本 `output/route_check_sim/speed_floor_ab.py`：`python3 speed_floor_ab.py OUT.json 0.04 0.06 --goals 10 --seed 7 --ros-args -p use_sim_time:=true`（在 `ros2-dev-humble` 里，仿真需先用 parity 配置启动）。

## 9. 别忘了

- 下次换硬件时装 Jetson RTC 纽扣电池。
- 固件 PR shenfangqi/2wheel-carbot-esp32#13（PID 换向）仍待合并。
- `~/.ssh/config` 给 `isaac-jetson` 加 ControlMaster（减少每次 SSH 登录触发的 polkitd 尖峰），用户还没同意。
- 8 块 VL53L5CX ToF 和 D455 还没装；仿真里的安装位置在 `sensors.vl53l5cx_sim_mount`、`sensors.d455_sim_mount`（codex `57e602b` 改成了低位布置），不参与运动配置的哈希。
