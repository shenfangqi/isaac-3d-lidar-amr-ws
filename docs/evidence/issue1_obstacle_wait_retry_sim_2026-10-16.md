# Issue #1：实车参数下的障碍等待重试与绕行仿真验收（2026-10-16）

目的：在 Isaac Sim warehouse_v3 中，用实车 Nav2 参数（parity 配置，只差仿真时间）验证 #1 的障碍处理行为。只用工作站，Jetson 未开机，没有任何实车运动。

结论：**交付范围缩小为 `obstacle_wait_retry`**：FollowPath 失败后等 2 s、沿原路径再试一次，再失败就停车 ABORT；不做全局绕行。PR1（#104）的绕行树在实车参数下 0/13 次成功，并有 2 次接触，原因见第 4 节。

逐次结果：`issue1_obstacle_wait_retry_sim_2026-10-16.json`。

## 1. 配置

- 交付配置：`configs/nav2_params_sim_real_parity_wait_retry.yaml`，由 `python3 scripts/build_sim_real_parity_nav2.py --wait-retry` 从 `carbot_navigation_real.yaml` 生成，和 parity 配置只差 `bt_navigator.default_nav_to_pose_bt_xml`（`navigate_to_pose_wait_and_retry_real.xml`）。
- 运行时读到的参数：新树已加载；global `obstacle_layer.enabled=False`；`FollowPath.max_allowed_time_to_collision_up_to_carrot=1.0`。
- 启动：`ISAAC_WEBRTC=1 START_RVIZ=0 LOCALIZATION_MODE=ground_truth NAV2_PARAMS_FILE=<上面的文件> ./start_nav_all.sh`。动态障碍只在 WebRTC 模式的 `streaming_carbot.py` 中可用。
- 场景：开阔带，起点 (-3, 4)、车头朝东，目标 (3, 4)，0.65×0.65×1.0 m 立方体放在初始路径 3.0 m 处（约 (0, 3.9)）；每次开跑前先导航回起点。四面封闭：4 面厚 0.8 m、长 4 m 的墙围在放置时车的位置周围（距离 1 m）。
- 工具：`scripts/validate_dynamic_obstacle.py --scenario {temporary,persistent,blocked,detour} --goal-x 3 --goal-y 4 --obstacle-distance 3 --trace-output ...`。

## 2. 交付配置验收（`obstacle_wait_retry`）

| 场景 | 次数 | Nav2 结果 | `/plan` | 车体到障碍最小净空 | 结束速度 |
|---|---|---|---|---|---|
| 临时障碍（停车后移开） | 3 | 3× SUCCEEDED | 1 | 0.073–0.081 m | 0 |
| 持续障碍 | 3 | 3× ABORTED | 1 | 0.079–0.081 m | 0 |
| 四面封闭 | 2 | 2× ABORTED | 1 | 0.039–0.041 m | 0 |

四面封闭最早的 2 次（blocked_01/02）作废：脚本把墙围在地图原点而不是车的位置，车直接到达了目标；修正后补跑为 blocked_03/04。

持续障碍时，车停住后等 2 s、重试一次、ABORT，没有原地转动或后退，`/plan` 只有初始的一条。

真实 Nav2 Humble `bt_navigator` 加桩服务器（FollowPath 依次失败 k 次）：k=0、1 SUCCEEDED，k=2 ABORTED，规划都只有 1 次。

## 3. 绕行对照（PR1 绕行树，不交付）

对照配置放在 `output/issue1_sim/control/`（不提交）：parity 配置 + main `d08d1f5` 的 `navigate_to_pose_replan_after_controller_failure_real.xml` + global 障碍层（`/scan_global_filtered` 标记、原始 `/scan` 清除）。同一开阔带场景 3 次：

| 次 | 结果 | `/plan` | 重规划时障碍框内致命栅格 | 绕行路径中心线到障碍框 | 最小净空 |
|---|---|---|---|---|---|
| 1 | ABORTED | 2 | 11 | 0.000 m | 0.077 m |
| 2 | ABORTED | 2 | 13 | 0.000 m | 0.085 m |
| 3 | ABORTED | 2 | 9 | 0.000 m | 0.079 m |

加上探索阶段的 10 次（不同位置、不同修正阶段，详见 JSON），绕行共 0/13 成功；接触 2 次：detour_02（1.0 s 碰撞时域）和 detour_10（临时 3.0 s 碰撞时域实验）。

## 4. 绕行失败的原因

1. **停得太近，转不了向。** RPP 只把当前速度投影 `max_allowed_time_to_collision_up_to_carrot` 秒（最远到 carrot），靠近障碍时速度被代价调节压到约 0.04 m/s，所以车头离障碍只剩 1–8 cm。绕行几乎都要先原地转，矩形车体的车角外接半径 0.204 m 会扫到障碍，RPP 一收到绕行路径就报 `detected collision ahead!`。实测停车点车中心到障碍：1.0 s 时 0.234 m（车头间隙 7.9 cm），临时设 3.0 s 时 0.337 m（18.2 cm）。**这一项只讨论，没有改参数。**
2. **只看到障碍正面。** MID-360 的 `/scan` 最小量程 0.5 m。正面接近时只有正面被标进 global costmap，障碍的深度是空地，绕行路径（Smac2D 只按中心点和内切半径约 0.13 m 判断）贴着正面拐弯，穿过看不见的部分；车贴近侧面时，侧面又落在 0.5 m 盲区内。3.0 s 实验中车能转向了，随即碰上了障碍背面（最小净空 0.0）。
3. **幽灵障碍。** `static_map_scan_filter` 把打到墙上的光束改成 inf，costmap 丢弃 inf 光束、不做 raytrace，被移走的障碍永久留在 global costmap 里（一次累积 283 个），污染后续路线。改为原始 `/scan` 负责清除后明显减少，但干净启动后一轮仍有 38–199 个零星残留，未查清。

盲区保持本身是有效的：车停在障碍前 0.28 m 时，global costmap 中仍保留 14–17 个致命栅格（障碍正面）。

## 5. 顺带修正

- `validate_dynamic_obstacle.py`：global costmap 只在启动时发完整栅格，之后走 `/global_costmap/costmap_updates`，原来只读完整栅格，统计一直为 0；rclpy 会把 `data` 赋值转回数组，补丁要存在单独的缓冲里。新增：重规划时障碍框内致命栅格、绕行路径中心线净空、车体最小净空、幽灵栅格、`persistent` 场景、`--trace-output`；四面封闭改为围在车当前位置（原来围在地图原点）。
- 实车 RPP 的停车距离（临时障碍时车体离障碍 2.4–8.1 cm）是 #12“提前减速”要处理的问题。

## 6. 后续（未实施）

安全绕行需要同时解决：停车余量（讨论中）、新障碍的假设厚度和 0.5 m 保持距离、绕行阶段有上限的多次重规划；窄空间需要 #10 的近场 ToF。
