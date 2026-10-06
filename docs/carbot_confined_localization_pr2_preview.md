# Issue #13 PR2：旋转扫掠只读预览

对应实施规格第 11 节 PR2。依赖 PR0 契约。本 PR **没有速度发布者，也不发任何运动命令**；它只计算原地旋转会扫过哪些格子，以及这些格子是已观测自由、已观测障碍还是未知，并在 RViz 里显示出来。

## 使用

正在运行的导航栈上（容器内）：

```bash
ros2 run isaac_3d_lidar_bringup localization_rotation_preview
```

或启动时加 launch 参数 `rotation_preview:=true`（默认 `false`）。RViz 中添加 MarkerArray `/automatic_localization/markers`，Fixed Frame 用 `odom`。逐角度结论发布在 `/automatic_localization/rotation_preview`（JSON，`motion_commanded` 恒为 `false`）。

颜色：绿 = 已观测自由，黄 = 未知，红 = 已观测障碍；蓝线 = 当前车体；文字 = 每个探测角的结论和未知格数。

## 判定（`localization_rotation_policy.py`）

1. **证据栅格**（odom 坐标，以车为中心的局部窗口，默认 ±1.0 m、0.05 m 格）：
   - 自由格只来自 `carbot_nav_recovery.sensor_visibility.scan_visibility`，即真实光束穿过；
   - 有效回波落点为障碍格；3D 点只能增加障碍（`add_obstacle_points`），点之间的空隙不会变成自由；
   - 静态地图不是输入，`range_min`/盲区内的格子保持未知，窗口外按未知处理。
   - 完整一圈的扫描另用半圈错位的副本计算一次，两份结果取并集。原因是 `scan_visibility` 对跨越扫描接缝（±π，车体正后方）的格子一律判未知；两份都只用真实光束，取并集仍然保守。
2. **扫掠区**：旋转角 = 目标角 + 制动余转。余转来自 motion profile（`速度×延迟 + stop_tail + yaw 余量`）；没有实测值时用保守默认 0.40 rad + 5°（参考 2026-09 实车停车尾段 0.57–0.92 s）。车体外扩 `padding 0.05 m + 中心漂移`。采样间隔保证最远顶点每步位移不超过半格，并把半个间隔计入余量。
3. **自身掩膜**：四个角都在当前车体（不加 padding）内部的格子不检查；车体外的 padding 环照常检查。
4. **结论优先级**：扫掠区有已观测障碍 → `OBSTACLE_IN_SWEEP`；有未被操作员承诺覆盖的未知格 → `UNKNOWN_SWEEP`；profile 缺失、未 ACCEPTED 或 hash 不匹配 → `PROFILE_INVALID`；否则放行。`RotationAttestation` 只覆盖未知格，计入 `attested_cells`，已观测障碍永远不能被承诺抵消。

`footprint_geometry_hash(footprint, padding)` 把 profile 绑定到车体外形和 padding，供 PR3 的 guard 校验。

## 验证（2026-10-06）

| 项目 | 结果 |
| --- | --- |
| S01–S06 | `test_localization_rotation_policy.py` 中同名测试全部通过；另覆盖扫描接缝、操作员承诺、profile 校验、窗口外未知、非法角度、JSON 输出 |
| 预览节点 | 源码无 Twist、无 `cmd_vel`，只有 String 和 MarkerArray 两个发布者；launch 默认关闭；用替身 TF 驱动时，TF 缺失报 `TF_AT_SOURCE_MISSING` 且不评估 |
| 容器全套 | `isaac_3d_lidar_bringup` 197 passed, 1 skipped（copyright），含 flake8/pep257；`carbot_nav_recovery` 51 passed |
| 真车 bag 离线回放（`2026-10-06_main_2b13795_recheck_01`，`/scan` range_min 0.5 m） | 6 个探测角全部 `UNKNOWN_SWEEP`，每个约 100 个扫掠格未知、0 个已观测自由；车体内部 20 格按掩膜扣除；最近障碍约 0.64 m，不在扫掠区。工作站每次评估（6 个角）约 26 ms |

真车结果与 PR0 审计一致：只靠现有传感器，车体旁的扫掠环全部是未知。要让旋转放行，只能依靠操作员承诺（`RotationAttestation`）覆盖这些未知格，且 profile 必须是 ACCEPTED。

## 尚未完成

- 在 Jetson 上实际运行预览节点，看 RViz 显示和耗时（只读，不需要人在现场）；
- `choose_probe`（按候选差异选择探测角）和运动预算属于 PR3，与 motion guard 一起实现；
- 启动脚本还没有 `--rotation-preview` 选项，目前用 launch 参数或 `ros2 run`。
