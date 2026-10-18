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
- 还没有在实车上验证。

## 4. 数据

`output/corner_sim/`（未提交）：
- `detour_series.sh`、`validate_dynamic_obstacle_bt.py`、`detour_old.xml`、`nav2_detour_near_stocktree.yaml`；
- `detour/*.log`、`detour/*.json`：各次结果和轨迹。
