# 绕障模式加后退脱困（2026-10-18 晚）

## 1. 改动

- `navigate_to_pose_detour_real.xml`：每次绕行（Detour）前，先调用 `reverse_escape` 后退 0.10 m；车尾被挡住时改为往前走（`reverse_escape.py`）。之后再等 2 s 让禁区数据稳定，然后重新规划。后退用 `ForceSuccess` 包住，后退不了时照常规划绕行，不会中止目标。
- `carbot_navigation_real.launch.py`：`novel_obstacle_detour:=true` 时也启动 `reverse_escape`。行为树加载时需要这个动作服务。
- 测试：
  - 绕障树的模拟执行（每次绕行前先后退；后退失败时仍然绕行）；
  - 节点集合检查；
  - 启动条件。

## 2. 仿真对比

环境：
- Isaac warehouse_v3，走 `ISAAC_WEBRTC=1` 路线（`streaming_carbot.py`）。这条路线带 #107 的雷达修复和动态方块，默认的 `auto_play_carbot.py` 两者都没有。
- 真值定位。
- 参数：`nav2_params_sim_real_parity_detour.yaml` 加近场通道，RPP 碰撞视野 1.0 s（实车值）。
- 手动启动 `novel_obstacle_keepoff`、`near_field_scan`（`input_type:=pointcloud2`）和 `reverse_escape`。
- 行为树通过 NavigateToPose 的 `behavior_tree` 字段逐个目标指定：原版取自 main，新版取自本分支。验证脚本是加了这个字段的本地副本。

场景：
- **窄缝**：#109 的门洞场景，缝宽 1.0 m。起点 (0, 1.2) 朝南，目标 (0, -3)，方块 x∈[-1.2, 0.0]、y∈[-1.2, -0.77]。
- **开阔通道**：#106 的场景。(-3, 4) → (3, 4)，方块放在路线 3 m 处。

| 场景 | 行为树 | 结果（2 次） | 规划 | 恢复 | 车身离方块最近 |
|---|---|---|---|---|---|
| 窄缝 1.0 m | 原版 | ABORTED, ABORTED | 4 | 7 | 0.056 / 0.052 m |
| 窄缝 1.0 m | 新 | **SUCCEEDED, SUCCEEDED** | 2 | 3 | 0.052 / 0.046 m |
| 开阔通道 | 原版 | ABORTED, ABORTED | 4 | 7 | 0.079 / 0.080 m |
| 开阔通道 | 新 | **SUCCEEDED, SUCCEEDED** | 2 | 3 | 0.081 / 0.076 m |

过程（新树每次相同）：
1. 车在方块前约 5–8 cm 停下（RPP 拒绝转向）；
2. 按原路线重试一次；
3. 后退约 0.10 m（4 次分别为 0.100–0.102 m）；
4. 等 2 s，重新规划，绕过方块，到达目标。

"车身离方块最近"出现在第一次停车时。

原版树每次都是：停车，重试，重新规划 3 次，每次转向都被拒绝，最后中止。

开阔通道场景以前（#106、#111、#112）只在仿真实验把碰撞视野放到 3.0 s 时成功过。按实车的 1.0 s，原版树在开阔通道里也绕不过去。

## 3. 注意

- 验证脚本的 `narrow` 判定是按"窄缝里安全中止"写的，所以新树成功时没有输出 PASS。本文以 Nav2 的结果为准。
- 同日下午的拐角仿真（`docs/evidence/corner_clearance_2026-10-18.md`）走的是默认的 `auto_play_carbot.py` 路线，没有 #107 的雷达修复。那组结果和实车一致，结论不变，但对比数字应以 WebRTC 路线为准。
- 第 5 节之前的版本曾部署到实车（`62fe2d2`），实车上暴露的问题见第 5 节；最终版本的实车结果见 5.4 节。

## 5. 禁区收紧：假设厚度 0.4 → 0.2 m，禁区半径 0.5 → 0.3 m

### 5.1 实车上暴露的问题

部署本分支后，实车上用 `--keepoff-detour` 测试：
- 箱子放在沙发西南角外的通道里，箱子西面到西墙约 0.8 m。
- 车在箱子前停下，后退 0.102 m 后重新规划，结果 `no valid path found`，目标中止。

禁区快照显示：
- 车是从东边看到箱子的，假设厚度 0.4 m 往西延伸，再加 0.5 m 禁区，从看到的箱面往西 0.9 m 都算禁区，正好顶到西墙。
- 西墙边 x 1.9–2.4、y 2.7–3.6 还有一块禁区，那里没有实时雷达点，操作员确认没有矮物。多半是摆箱子时人走过留下的标记：箱子挡住了车看向那里的射线，所以一直没被清除。

这两项把唯一的通道封死了。

### 5.2 改动

`carbot_obstacle_detour_real.yaml`：`assumed_depth_m` 0.2、`keepoff_radius_m` 0.3。当初 0.4 m 和 0.5 m 是在 0.5 m 以内什么也看不到时定的。现在局部代价地图有近场通道（0.1–0.6 m），绕行前又有 `reverse_escape` 后退。

### 5.3 仿真（同第 2 节的环境，新树）

开到起点那一步也使用新树（第一轮用原版树时，0.6 m 中止后车卡在墙端，导致后续起点无效，那几次已作废重跑）。

| 场景 | 结果（2 次） | 规划 | 车身离方块最近（第一次停车时） |
|---|---|---|---|
| 窄缝 1.0 m | SUCCEEDED, SUCCEEDED | 2 | 0.047 / 0.051 m |
| 窄缝 0.8 m | **SUCCEEDED, SUCCEEDED** | 2 | 0.047 / 0.054 m |
| 窄缝 0.6 m | ABORTED, ABORTED | 4 | 0.048 / 0.044 m |
| 开阔通道 | SUCCEEDED, SUCCEEDED | 2 | 0.081 / 0.076 m |

- 0.8 m 的缝在 0.5 m 禁区时被封住（#109），现在可以绕过。按车身四角粗算，经过方块旁边时离方块约 0.38 m（1.0 m 缝约 0.46 m）。
- 0.6 m 的缝：禁区留出约 0.3 m 宽的空间，规划器能规划穿过，但 RPP 每次在离方块 4–5 cm 处停下。三次绕行后中止，没有接触方块。

数据：`output/corner_sim/detour_tight/`、`detour_tight2/`（未提交）。

### 5.4 实车复测（`1430cd8`）

- 部署方式：`deploy_bringup_to_jetson.sh --apply`，`--check` 结果为 IN_SYNC；已核对安装后的 `assumed_depth_m` 为 0.2、`keepoff_radius_m` 为 0.3。
- 启动命令：`./start_auto_nav.sh --keepoff-detour`，近场通道开。
- 场景：箱子放在沙发西南角外的通道里，与 5.1 节同一位置，箱子西面到西墙约 0.8 m。操作员摆好箱子后离开了车的视野。

目标：(4.24, 2.75) → (2.72, 4.94)。

| 时间 | 事件 |
|---|---|
| 22 s | 箱子在前，RPP 报"会撞" |
| 22–25 s | 等 2 s 后按原路线重试，仍然失败 |
| 29 s | `reverse escape done: backed up 0.102 m` |
| 30 s | 等 2 s，重新规划，得到从箱子和西墙之间穿过的路线 |
| 66 s | 绕行中报一次"会撞"，等 2 s 后按同一路线重试，成功继续 |
| 98 s | **SUCCEEDED** |

同一位置，5.1 节在 0.4 / 0.5 m 参数下是 `no valid path found`。

## 4. 数据

`output/corner_sim/`（未提交）：
- `detour_series.sh`、`validate_dynamic_obstacle_bt.py`、`detour_old.xml`、`nav2_detour_near_stocktree.yaml`；
- `detour/*.log`、`detour/*.json`：各次结果和轨迹。
