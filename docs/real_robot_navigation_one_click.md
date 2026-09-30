> 自动定位默认保持验证模式：`--automatic` 只旋转并输出候选，`--stationary-validation` 不旋转。三个起点验收通过后新增显式 `--automatic-activate`，只有该入口会在质量验证通过后激活 Nav2；它仍不会发送导航目标。证据见 [验证记录](carbot_localization_validation.md)。

# 真机一键导航

在工作站仓库根目录执行：

```bash
./start_real_nav.sh
```

自动旋转、定位并在验证通过后激活 Nav2：

```bash
./start_real_nav.sh --automatic-activate
```

只验证自动定位候选、保持 Nav2 未激活：

```bash
./start_real_nav.sh --automatic
```

脚本按以下安全顺序执行：

1. 获取单实例启动锁并检查 RViz 是否已经运行；重复执行会明确退出，不会重建正在等待初始位姿或正在导航的容器。
2. 核验 Jetson 身份；SSH 别名的 mDNS 暂时失效时，回退到已记录的地址并再次核验身份。
3. 停止与 Nav2 冲突的网页遥控发布者，确认底盘通信服务处于运行状态。
4. 用默认实车地图重建未激活的 `navigation-safe` 容器。
5. 运行不产生运动的硬件预检，检查 CP2102、轮速、里程计、MID-360、Scan、TF 和速度话题拓扑。若且仅若失败项属于冷启动 DDS/话题/TF 就绪类，等待 5 秒后自动重试一次；USB、服务、非零速度、定位急停或 FAST-LIO 不稳定等安全失败不会重试。
6. 预检确认 FAST-LIO 静止稳定且 `/localization/emergency_stop=false` 后，重启速度补偿服务，清除之前可能残留的进程内急停锁存，并确认速度话题端点恢复。
7. 默认模式激活定位/地图并等待 `2D Pose Estimate`；`--automatic` 原地旋转后停在候选验证；`--automatic-activate` 原地旋转、全图搜索并通过质量保持窗口后继续。
8. 允许激活的模式在定位 TF 建立后激活 Nav2，并使用长生命周期 ROS 节点检查 lifecycle、Action、速度链、Scan、TF、RViz Goal 链路，以及无目标时速度静默或仅有零速停车尾帧。

若预检、定位或最终健康检查失败，脚本会停止导航容器和 RViz、重启速度补偿服务清除本次锁存，并恢复处于未使能状态的网页遥控。

脚本不会自动发送 Goal。最终出现下面一行后，才可在确认 Scan 与地图对齐、车旁安全的前提下使用 RViz `2D Goal Pose`：

```text
READY: RViz navigation is active; no goal was sent and no nonzero /cmd_vel was observed.
```

停止导航时，脚本先停 Nav2/RViz，在速度发布者释放后重启速度补偿服务以清除锁存，最后恢复处于未武装状态的网页遥控服务：

```bash
./stop_real_nav.sh
```

只读复检当前运行栈：

```bash
./start_real_nav.sh --health-check
```

默认地图是：

```text
/home/shenfq/Projects/isaac_ros-dev/maps/real/carbot_map_20260928_215841.yaml
```

临时使用另一张地图时，把 Jetson 上的绝对 YAML 路径作为参数：

```bash
./start_real_nav.sh /home/shenfq/Projects/isaac_ros-dev/maps/real/another_map.yaml
```

等待 `2D Pose Estimate` 的默认超时为 600 秒，可按需覆盖：

```bash
CARBOT_INITIAL_POSE_TIMEOUT=900 ./start_real_nav.sh
```
