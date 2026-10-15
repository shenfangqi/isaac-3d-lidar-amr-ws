# Issue #5：在 Isaac Sim 里实际运行实车对齐的 Nav2 配置（2026-10-15）

目的：PR #91 把 `configs/nav2_params_sim_real_parity.yaml` 改为从实车加载的
`src/isaac_3d_lidar_bringup/config/nav2/carbot_navigation_real.yaml` 自动生成之后，在 Isaac Sim
warehouse_v3 里实际启动一次，确认配置能加载、参数真的生效、导航行为与实车一致。只用工作站，Jetson 未开机。

## 启动

`start_nav_all.sh` 新增可选环境变量 `NAV2_PARAMS_FILE`（不设置时行为不变）：

```bash
START_RVIZ=0 LOCALIZATION_MODE=ground_truth \
NAV2_PARAMS_FILE=/workspace/ros-humble/isaac_3d_lidar_amr_ws/configs/nav2_params_sim_real_parity.yaml \
./start_nav_all.sh
```

全部启动健康检查通过（`[ OK ] All startup health checks passed.`）。

## 运行中读取的参数（`ros2 param get`）

| 参数 | 实测值 | 说明 |
|---|---|---|
| `FollowPath.max_allowed_time_to_collision_up_to_carrot` | 1.0 | 实车值（旧 parity 为 2.0） |
| `FollowPath.desired_linear_vel` | 0.1 | 实车值（默认仿真为 0.30） |
| `FollowPath.use_regulated_linear_velocity_scaling` | True | 实车新增的弯道减速 |
| `controller_frequency` | 10.0 | |
| local / global `inflation_radius` | 0.25 / 0.25 | 实车值（旧 parity 全局为 0.30） |
| global `footprint_padding` | 0.01 | Nav2 默认值；实车文件也未设置 |
| `velocity_smoother.max_velocity` | [0.1, 0.0, 0.5] | |
| `default_nav_to_pose_bt_xml` | `navigate_w_replanning_only_if_path_becomes_invalid.xml` | 实车的无恢复行为树 |
| `GridBased.plugin` | `nav2_smac_planner/SmacPlanner2D` | |
| `use_sim_time` | True | 唯一与实车不同的参数 |

## 导航目标（`scripts/validate_nav_goal.py`，ground-truth 定位）

| 次 | 模式 | Nav2 结果 | 恢复次数 | 终点误差 | 验证脚本 |
|---|---|---|---|---|---|
| 1 | 直行 0.70 m | SUCCEEDED | 0 | 0.121 m / 0.31° | 未通过（位置上限 0.12 m） |
| 2 | 转弯 90° | SUCCEEDED | 0 | 0.090 m / 5.41° | 通过 |
| 3 | 直行 0.70 m | SUCCEEDED | 0 | 0.072 m / 2.07° | 通过 |
| 4 | 转弯 90°（目标格代价 99） | ABORTED | 0 | 0.209 m / 31.9° | 未通过 |

每次结束时车都已停稳（0 m/s、0 rad/s）。

## 结论

- parity 配置在 Isaac Sim 中正常加载，运行参数与实车 Nav2 文件一致（只差仿真时间）。
- 第 1 次：Nav2 判成功时车距目标约 0.12 m，原因是实车的 `xy_goal_tolerance: 0.10` 加上减速距离。这与实车记录一致（2026-09-19 实车 0.20 m 目标走约 0.106 m 即判成功），验证脚本的 0.12 m 上限是为默认快速仿真配置设定的。阈值没有修改。
- 第 4 次：目标在膨胀区内，RPP 报 `detected collision ahead!` 后 Nav2 中止，没有原地转动或后退等恢复动作，与实车行为树的设计一致（无法通过时停车报失败，不在障碍物旁反复尝试）。问题在于验证脚本选的目标点离障碍物太近，parity 配置本身没有问题。
- 转弯后的航向误差为 2–5°，大于 `yaw_goal_tolerance: 0.03`（1.7°）：Nav2 到点时满足容差，之后 Isaac 车体在零指令后继续转动。实车在 Spin 后的过冲约为 2.8–3.9°。Isaac 接触动力学不是实车数字孪生（#5 明确不在范围内），这里的差异只作为观察记录。
- 日志：`logs/start_nav_all/navigation.log`（本次运行）。
