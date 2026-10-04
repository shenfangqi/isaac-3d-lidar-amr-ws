# Issue #12：复杂弯路近障减速与原目标续航

## 当前实现边界

当前分支提供只读、默认关闭的验证阶段，不直接改变车辆速度：

- 使用 Nav2 原始局部代价地图和实测 footprint 检查路径扫掠区域。
- 计算局部路径最大曲率、距首个不安全位置的路径距离。
- 只有加载经真机验收的严格制动配置后，才给出曲率/制动速度建议。
- 发布 JSON、RViz 标记并可保存路径、代价地图、报告和导航配置哈希。
- 不发布 `Twist`，不重发目标，不改变 Nav2 lifecycle。

这解决的是 Issue #12 的证据与算法前置阶段。自动应用速度和自动续航还未启用。

## 桌面验证

```bash
docker start ros2-dev-humble
docker exec ros2-dev-humble bash -lc '
  source /opt/ros/humble/setup.bash
  cd /workspace/ros-humble/isaac_3d_lidar_amr_ws
  colcon build --packages-select carbot_recovery_interfaces carbot_nav_recovery --symlink-install
  source install/setup.bash
  ROS_DOMAIN_ID=73 pytest -q src/carbot_nav_recovery/test
'
```

维护脚本的静态检查：

```bash
bash -n scripts/start_real_robot_navigation_rviz.sh
bash -n scripts/jetson_nvblox_container.sh
```

本机合成 80×80、0.05 m 栅格、实车 footprint、31 点弯曲路径的 200 次只读几何基准为 median 21.00 ms、p95 21.38 ms、p99 21.88 ms。该结果只说明桌面实现没有明显性能退化，不能替代 Jetson 基准或物理验收。

## 真机阶段（需要人工）

1. 确认急停操作员、空旷制动标定区域、实际载荷和轮胎/地面条件。
2. 测量控制指令到实际减速的完整延迟，以及多个速度点的保守最大制动减速度。
3. 根据定位抖动、路径跟踪误差和 footprint 不确定性确定 `position_margin_m`。
4. 保存原始 rosbag/日志，生成严格制动配置，但在验收签字前保持 `physical_acceptance_complete=false`。
5. 同步本分支到 Jetson 并重新构建；用 `./start_real_nav.sh --complex-route-validation` 启动。
6. 在代表性多弯路线只读采集 advisory 与快照；先不修改控制输出。
7. 对比“首次停止”与“再次点击同一目标后继续”的两段证据，确认控制器失败类型、局部地图、实际速度和停止距离。

进入自动减速/续航实现前，至少需要真机制动配置、代表性路线证据，以及 Issue #10 近场观测对相关扫掠方向的覆盖结论。任何缺项都保持 fail-closed。
