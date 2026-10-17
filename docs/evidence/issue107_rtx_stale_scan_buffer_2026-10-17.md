# Issue #107：Isaac RTX 雷达"假回波"的来源与消除（2026-10-17）

只用工作站，Jetson 未开机。分支 `fix/issue107-rtx-ghost-returns`，在单独的 worktree 里开发。

## 1. 结论

假回波**不是光线追踪打到了什么东西**，而是 Isaac 4.5 整圈扫描缓冲区里的**残留旧点**：

- `/livox/lidar` 由 `ROS2RtxLidarHelper`（`fullScan=true`）发布，内部用 `IsaacCreateRTXLidarScanBuffer` 拼一整圈。缓冲区每条射线一个槽位（本配置 20000 个），每圈只覆盖**本圈有返回**的槽位。
- RTX 默认会丢掉没有返回的射线（`/app/sensors/nv/lidar/skipDroppingInvalidPoints=false`），每圈实际只有约 8700 个返回。其余槽位保留**以前某一圈**写进去的点。
- 缓冲区是传感器坐标系（`transformPoints=false`），所以这些旧点在车体坐标里固定不动，跟着车走、跟着车转。
- 车在出生点没动过时，旧点和新点完全一样，看不出来；车一动，旧点就和实际场景对不上了。车走过的地方越多，"曾经有过返回"的槽位越多：点数从 8766 涨到 15058、16653。

修复：Kit 启动时加 `--/app/sensors/nv/lidar/skipDroppingInvalidPoints=true`。没有返回的射线也会以距离 0 写入缓冲区，把旧点覆盖掉；`keepOnlyPositiveDistance`（默认 true）再把这些距离为 0 的点去掉。点数改为随场景变化（约 8600–11900），不再累积。

这个设置**只在 Kit 启动时读取**（lidar 扩展在脚本运行前就已加载）。在脚本里用 `carb.settings` 设置无效，已实测：假回波照样出现。

## 2. 排查过程

| 步骤 | 结果 |
|---|---|
| 复现：出生点 → (1, 0) → (0, 0) | 回到出生点后，`/scan_global_static_filtered` 多出 (-30°, 1–1.5 m)、(40°, 1–1.5 m)、(110°, 1 m)、(-100°, 3 m) |
| issue 实验 1：车停下时不再 `set_world_pose` | 假回波不变，排除"每步瞬移" |
| 用 `IsaacReadRTXLidarData` 读 RTX 原始返回（每帧方位、俯仰、距离、发射器、传感器位姿） | (-30°, 1.0 m) 处**没有任何 RTX 返回**，但 `/livox/lidar` 里有 |
| 车静止 2 s，比较 20 帧 `/livox/lidar` | 16653 个点里 4720 个**逐位完全相同**（真实返回带测距噪声，不可能逐位相同），假回波位置全在其中；出生点开动前只有 43 个 |
| 运行时把 ScanBuffer 的 `keepOnlyPositiveDistance` 改成 false | 输出变成满 20000 点，其中 4942 个全零，残留点仍有约 6300 个：没有返回的射线根本不进缓冲区 |
| Kit 启动参数加 `skipDroppingInvalidPoints=true` | 残留点 0，假回波消失 |

旁证：之前 issue 里查到的那条"右侧 y≈-0.88 m、与车头平行的线"另有一部分是真实几何（世界 y≈0.1 m 处，出生点开动前就在原始点云里），与本问题无关；它在 `/scan_global_static_filtered` 里被静态地图滤掉。

`objectId` 在 `IsaacReadRTXLidarData` 里始终为 0（即使启动时加 `auxDataOutput=FULL`），这次没有用上。

## 3. 改动

- `isaac_sim/carbot_mid360.py`：`create_mid360_pipeline()` 先检查 `skipDroppingInvalidPoints` 是否为 true，不是就报错退出（错误信息里给出需要的启动参数），避免悄悄带着假回波运行。
- `isaac_sim/auto_play_carbot.py`：`SimulationApp` 通过 `extra_args` 自带这个参数（非 WebRTC 路径）。
- `isaac_sim/auto_play_carbot.py`、`isaac_sim/streaming_carbot.py`：`WORKSPACE` 可以用环境变量 `CARBOT_SIM_WORKSPACE` 覆盖，默认值不变，便于在 worktree 里验证仿真端改动。
- `tests/test_mid360_stale_points_guard.py`：检查函数的单元测试。

**仓库外**的 `../start_nav_all.sh`（不在 git 里）同步修改，已在工作站上生效：

```diff
 WORKSPACE_CONTAINER="/workspace/ros-humble/isaac_3d_lidar_amr_ws"
+# Isaac Sim scripts only; ROS/nvblox keep WORKSPACE_CONTAINER (built install/).
+ISAAC_SIM_WORKSPACE="${ISAAC_SIM_WORKSPACE:-$WORKSPACE_CONTAINER}"
 ...
-exec ./runheadless.sh --exec "/workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/streaming_carbot.py"
+export CARBOT_SIM_WORKSPACE='"${ISAAC_SIM_WORKSPACE}"'
+echo "Isaac Sim workspace: ${CARBOT_SIM_WORKSPACE}"
+# The lidar plugin reads this only at startup; without it the full-scan
+# cloud keeps stale points that move with the robot (issue #107).
+exec ./runheadless.sh --exec "${CARBOT_SIM_WORKSPACE}/isaac_sim/streaming_carbot.py" \
+    --/app/sensors/nv/lidar/skipDroppingInvalidPoints=true
 ...
-exec ./python.sh /workspace/ros-humble/isaac_3d_lidar_amr_ws/isaac_sim/auto_play_carbot.py
+export CARBOT_SIM_WORKSPACE='"${ISAAC_SIM_WORKSPACE}"'
+echo "Isaac Sim workspace: ${CARBOT_SIM_WORKSPACE}"
+exec ./python.sh "${CARBOT_SIM_WORKSPACE}/isaac_sim/auto_play_carbot.py"
```

旧版 `start_nav_all.sh` 的 WebRTC 路径没有这个参数，新代码会在创建雷达时报错退出，所以必须一起更新。

## 4. 验收

判定方法：`/livox/lidar` 连续 20 帧（车静止）中逐位相同的点数（应为 0），加上 `/scan_global_static_filtered` 4 m 内的回波。

### 4.1 各位置无残留点（WebRTC 模式，`wait_retry` parity 配置，ground truth）

| 位置 | 逐位相同的点 | scan 4 m 内回波 |
|---|---|---|
| 出生点（未动） | 0 / 8670 | 只有车后 2 m 的 28 cm 限高梁 |
| (1, 0) 朝东 | 0 / 9665 | 同一根限高梁（2.5 m） |
| (-3, 4) 朝东 | 0 / 11888 | (-100°, 2.5 m) 2 格 |
| (-3, 4) 朝北 | 0 / 11908 | (160°, 2.5 m) 1 格；(-170°, 3.5 m) 6 格 |
| 回到出生点 | 0 / 8763 | 只有限高梁 |

(-3, 4) 朝东和朝北看到的 (-100°, 2.5 m) 与 (160°, 2.5 m) 在地图上是同一点（约 (-3.6, 1.5)），即 `Cylinder_02` 的边缘；(-170°, 3.5 m) 是 (-2, 0) 的限高梁。两者都是真实几何（`/isaac_sim/debug_prim_query` 确认），不跟车走。

非 WebRTC 路径（`auto_play_carbot.py`）：出生点 → (1, 0) → (0, 0) 后残留点 0 / 8671，scan 里只有限高梁。

### 4.2 开阔带往返，不学习伪回波

detour parity 配置，`novel_obstacle_keepoff` 手动启动，`ghost_travel_m:=1000.0`（关闭伪回波学习，也就关闭了静止确认）。(-3, 4) ↔ (3, 4) 共 6 段，全部 SUCCEEDED。

| 到达 | 不在静态地图里的禁区格 |
|---|---|
| (-3, 4)（第 1 次） | 0 |
| (3, 4) | 0 |
| (-3, 4) | 705，x[-3.82, -1.52] y[-0.02, 1.98] |
| (3, 4) | 0 |
| (-3, 4) | 317，全部在真实物体 0.48 m 以内 |
| (3, 4) | 0 |
| (-3, 4) | 388，全部在真实物体 0.46 m 以内 |

(-3, 4) 处的禁区来自 `Cylinder_02`（离地 0.2–0.8 m 的悬空圆柱，地图 x[-3.5, -2.5] y[0.52, 1.52]）和 28 cm 限高梁（地图 x[-2.2, -1.8] y[-0.5, 0.5]）。两者都不在静态地图里，是真实障碍，不是假禁区。#106 证据第 3 节第 3 条"圆柱附近的回波偶尔落到静态遮罩外面"也是这个圆柱。

### 4.3 #5 parity 回归

冷启动，`nav2_params_sim_real_parity.yaml`，`scripts/validate_nav_goal.py`，顺序与 `issue5_isaac_parity_run_2026-10-15.md` 相同：

| 次 | 模式 | Nav2 结果 | 恢复 | 终点误差 | 验证脚本 | #5（2026-10-15） |
|---|---|---|---|---|---|---|
| 1 | 直行 0.70 m | SUCCEEDED | 0 | 0.120 m / 0.29° | 通过 | SUCCEEDED，0.121 m / 0.31°，未通过 |
| 2 | 转弯 90° | SUCCEEDED | 0 | 0.093 m / 2.88° | 通过 | SUCCEEDED，0.090 m / 5.41° |
| 3 | 直行 0.70 m | SUCCEEDED | 0 | 0.079 m / 4.08° | 通过 | SUCCEEDED，0.072 m / 2.07° |
| 4 | 转弯 90° | ABORTED | 0 | 0.244 m / 27.3° | 未通过 | ABORTED，0.209 m / 31.9° |

结果与 #5 一致：第 1 次在 0.12 m 上限附近，第 4 次的目标在膨胀区内。每次结束时车都已停稳。运行参数与 #5 相同（`desired_linear_vel` 0.1、`max_allowed_time_to_collision_up_to_carrot` 1.0、`controller_frequency` 10、`velocity_smoother.max_velocity` [0.1, 0, 0.5]、无恢复行为树）。本修复不改车的驱动、`/odom` 和运动响应，只改雷达输出。

## 5. 后续

- 验收第 4 条（另开 PR）：在无假回波的前提下评估简化 `novel_obstacle_keepoff` 的过滤规则（伪回波学习、静止确认只在学习完成后启用、视角基线），并用 #1 的开阔带场景（绕行、四面封闭、临时障碍）回归。
- 观察：建图用的旧 Carter 脚本（`auto_play_mid360.py`、`streaming_auto_play.py`，被 bootstrap 哈希锁定）同样用整圈缓冲区，没有改。warehouse_v3 的地图是否受过残留点影响没有核查；重建地图时应加上同一个启动参数。
- 数据和调试工具（未提交）：`output/issue107_tools/`（`buf_check.py` 残留点检查、`ghost_probe.py`、`rtx_*.py` 原始返回分析、`streaming_carbot_debug.py` 含调试钩子的版本），RTX 原始帧在 `output/issue107_tools/rtx_frames/`。
