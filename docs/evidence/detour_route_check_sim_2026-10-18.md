# 绕障模式提前发现路线被挡（#123，仿真，2026-10-18）

## 1. 改动

- `navigate_to_pose_detour_real.xml`：
  - 行驶中每次 tick 检查路线前方 4 m（`TruncatePathLocal` + `IsPathValid`，用 `ReactiveSequence` 放在 FollowPath 前）。4 m 等于禁区的标记距离 `mark_max_range_m`。被挡就立即停止 FollowPath，不论离障碍多远。
  - 停车后等 3 s：前方畅通 → 沿**原路线**继续，不重新规划，不计绕行次数；还被挡 → 直接 `ComputePathToPose`，**不后退**。
  - 规划失败 → 继续停车，每 3 s 检查一次：原路线畅通就走原路线，能规划就走新路线，没有时间上限。
  - 新路线又被挡 → 算一次新的绕行。RPP 碰撞兜底（等 2 s 重试 → `reverse_escape` → 等 2 s → 规划）不变，和提前停车共用 3 次绕行上限。
  - 绕行次数用完 → 不中止，每 3 s 检查一次，畅通就沿当前路线走，直到到达或操作员取消。
  - 只用 Humble 自带的 BT 节点（`TruncatePathLocal`、`IsPathValid` 在 bt_navigator 默认插件列表里，`RetryUntilSuccessful num_attempts=-1` 来自 BT.CPP 3.8.7），不需要 C++ 插件。
- `carbot_obstacle_detour_real.yaml`：全局代价地图改用 `robot_radius: 0.130`（`footprint: "[]"`）。
  - 原因：Humble planner 的 `isPathValid` 用多边形时按每个路线点的朝向检查车身是否压到致命格；Smac2D 的路线点朝向都是 0，贴墙路线可能被墙判成"被挡"。用半径时只看路线中心点是否致命或内切（≥253），Smac2D 本来就不会规划到这种格子，所以静态墙不会触发。
  - 0.130 m 是多边形的内切半径（车尾边），膨胀层代价不变；Smac2D 2D 只看中心代价，规划不变。局部代价地图仍用多边形。
- `validate_dynamic_obstacle.py`：新增 `--behavior-tree`（逐目标指定行为树）、`--route-check`、`--expect-pass`、`--cancel-after-stop-s`；记录停车时车身离方块距离、后退次数。
- 单元测试 `test_obstacle_wait_retry.py`：在模拟时钟上执行绕障树，覆盖 issue 列出的各种情况；原来"绕行 3 次后中止"的预期改为继续等待。

## 2. 仿真

环境与 `detour_reverse_escape_2026-10-18.md` 相同：
- Isaac warehouse_v3，`ISAAC_WEBRTC=1`，真值定位；
- 参数：detour parity 加近场 `local_costmap`，默认树为原版，全局代价地图改为半径（`output/route_check_sim/nav2_detour_near_stocktree.yaml`，未提交）；
- 手动启动 `novel_obstacle_keepoff`、`near_field_scan`、`reverse_escape`；
- 新树通过 `--behavior-tree` 逐目标指定，"开到起点"也用新树。

| 场景 | 预期 | 结果（2 次） | 停车时车身离方块 | 规划 | 后退 | 车身离方块最近 |
|---|---|---|---|---|---|---|
| temporary（方块出现后撤走） | 停车、等 3 s、原路线 | SUCCEEDED ×2 | 2.34 / 2.42 m | 1 / 1 | 0 / 0 | — |
| detour（开阔通道，方块在 3 m 处） | 提前停车、绕行、不后退 | SUCCEEDED ×2 | 2.31 / 2.30 m | 2 / 2 | 0 / 0 | 0.504 / 0.504 m |
| 窄缝 1.0 m | 通过 | SUCCEEDED ×2 | 1.53 / 1.59 m | 2 / 2 | 0 / 0 | 0.458 / 0.475 m |
| 窄缝 0.8 m | 通过 | SUCCEEDED ×2 | 1.58 / 1.48 m | 2 / 2 | 0 / 0 | 0.369 / 0.341 m |
| 窄缝 0.6 m | 停车等待，不中止 | 等待 20 s 后由脚本取消 ×2 | 1.50 / 1.50 m | 4 / 4 | 1 / 1 | 0.364 / 0.364 m |
| blocked（四面封闭） | 停车等待，不中止 | 等待 20 s 后由脚本取消 ×2 | 0.20 / 0.35 m | 1 / 1 | 0 / 0 | 0.201 / 0.351 m |

对比 #122（同场景旧树）：开阔通道和 1.0 m 缝原来是开到方块前 5–8 cm 才停，后退 0.10 m 再绕；现在在 1.5–2.4 m 外停下，直接绕行，不后退。

说明：
- 窄缝的 "1.0 m 缝不能过" 判定是 #109 时的（禁区 0.4 / 0.5 m）。#122 把禁区收紧到 0.2 / 0.3 m 后 0.8、1.0 m 缝都能过，所以这两种宽度用 `--expect-pass`。
- 0.6 m 缝：规划器在禁区和门框之间找到一条很窄的通道，车开到缝口，禁区随视角变大，又被挡两次（规划 3、4 次，绕行次数用完）；在缝口 RPP 拒绝前进，走了一次碰撞兜底（后退 1 次）。之后每约 3.5 s 重试一次，RPP 每次拒绝，车不动，最近 0.364 m，没有碰撞。#122 时这个场景是安全中止；按新规则改为一直等待，需要操作员取消。
- blocked：起点被围住，停车后规划失败，每 3 s 检查一次，速度一直为 0，没有中止。
- 等待期间 behavior_server 每次打印 `Running wait`，NavigateToPose 反馈里的 `number_of_recoveries` 随之增加，规划失败时 planner 打印 `failed to create plan`。
- 所有"开到起点"的目标（新树、穿过整个仓库）都正常到达，没有出现静态墙让路线判为被挡的情况。
- 用这个参数文件启动时，`start_nav_all.sh` 的健康检查 "global costmap uses Carbot footprint" 会失败（检查 footprint 字符串里是否有 0.155），是预期的，其他检查都通过。

## 3. 未做

- 实车验证（issue #123 验证计划的"实车"部分）：部署后在 1 m 以上宽的通道放箱子、人在车前 1–2 m 走过、箱子离墙 0.8 m。
- 没有加防抖（连续 N 次被挡才停车）：Humble 自带节点做不到，而禁区本身要连续 3 次扫描确认才标记。一次误报的代价是停车 2–3 s 后沿原路线继续。
