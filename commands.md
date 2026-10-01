# Carbot 实车导航命令速查

本文只列出维护的一键入口。实车导航不要手工拼接 SSH、Docker、ROS 2 lifecycle、FAST-LIO 或速度补偿命令。

## 进入仓库

```bash
cd /home/shenfq/projects/ros-humble/isaac_3d_lidar_amr_ws
```

## 查看启动帮助

```bash
./start_real_nav.sh --help
```

`stop_real_nav.sh` 当前没有帮助选项，也不接受其他参数。不要运行 `./stop_real_nav.sh --help`，因为它仍会执行停止流程。

## 手工初始位姿导航

```bash
./start_real_nav.sh
```

流程会完成硬件预检、打开 RViz，并在 `[7/8]` 等待人工标定：

1. 在 RViz 点击 `2D Pose Estimate`；
2. 在地图真实位置按下鼠标；
3. 沿真实车头方向拖动箭头后松开；
4. 等待终端出现 `READY`；
5. 再次确认蓝色小车、车头方向和青色扫描与真实环境一致。

该入口不会自动发送导航目标。只有出现 `READY` 并完成现场核对后，才能使用 RViz 的 `2D Goal Pose`。

## 自动定位验证

```bash
./start_real_nav.sh --automatic
```

流程会安全预检、原地旋转一圈、停车并执行全图候选搜索。成功后停在：

```text
CANDIDATE_READY
```

此模式默认 `validation_only=true`：

- Nav2 不激活；
- 不能发送导航目标；
- 用于检查自动候选是否与真实位置、朝向和墙体扫描一致。

下面的命令与 `--automatic` 等价：

```bash
./start_real_nav.sh --validation-only
```

## 自动定位并激活 Nav2

```bash
./start_real_nav.sh --automatic-activate
```

定位过程与 `--automatic` 相同。候选通过全部质量门后，程序继续激活 Nav2 并执行无目标健康检查。成功标志是：

```text
READY: RViz navigation is active; no goal was sent and no nonzero /cmd_vel was observed.
```

该入口仍不会自动发送目标。出现 `READY` 后必须先核对：

- 蓝色小车位置与真实小车一致；
- 蓝色小车车头方向正确；
- 青色扫描整体贴合地图墙体；
- 小车在无目标状态保持静止。

全部正确后，才可由现场操作者使用 RViz `2D Goal Pose` 选择明确目标。

## 静止人工定位验证

```bash
./start_real_nav.sh --stationary-validation
```

该模式不自动旋转，也不激活 Nav2。它用于：

- 人工 `2D Pose Estimate` 标定；
- 错误位置拒绝测试；
- 正确人工回退测试；
- 静止扫描与地图对齐诊断。

## 只读健康检查

导航已经运行时执行：

```bash
./start_real_nav.sh --health-check
```

它检查 Nav2 lifecycle、Action server、速度话题拓扑、Scan、TF、RViz 和无目标速度状态，不会发送目标。

不要在另一个启动流程仍处于预检、等待定位、旋转或搜索时再次启动第二份导航；单实例保护会拒绝重复启动。

## 停止导航

```bash
./stop_real_nav.sh
```

停止程序会按安全顺序：

1. 关闭 Nav2；
2. 关闭导航容器和 RViz；
3. 重启速度补偿以清除本次安全锁；
4. 恢复未使能的 web teleop。

正常结束或发生故障后不需要再运行单独的“解锁”命令。

## 使用其他保存地图

把 Jetson 上地图 YAML 的绝对路径放在最后：

```bash
./start_real_nav.sh \
  /home/shenfq/Projects/isaac_ros-dev/maps/real/another_map.yaml
```

自动验证和自动激活也可指定地图：

```bash
./start_real_nav.sh --automatic \
  /home/shenfq/Projects/isaac_ros-dev/maps/real/another_map.yaml

./start_real_nav.sh --automatic-activate \
  /home/shenfq/Projects/isaac_ros-dev/maps/real/another_map.yaml
```

默认地图是：

```text
/home/shenfq/Projects/isaac_ros-dev/maps/real/carbot_map_20260928_215841.yaml
```

## 常用环境覆盖

延长手工 `2D Pose Estimate` 等待时间：

```bash
CARBOT_INITIAL_POSE_TIMEOUT=900 ./start_real_nav.sh
```

显式指定 Jetson 地址：

```bash
CARBOT_JETSON_HOST=192.168.1.109 ./start_real_nav.sh --automatic
```

显式指定地图：

```bash
CARBOT_NAV_MAP=/home/shenfq/Projects/isaac_ros-dev/maps/real/carbot_map_20260928_215841.yaml \
  ./start_real_nav.sh --automatic-activate
```

支持的环境变量可通过以下命令查看：

```bash
./start_real_nav.sh --help
```

## 安全退出与重试

以下结果表示程序已拒绝继续，不是永久锁死：

```text
SAFE_STOP
FAULT_STOPPED
WAIT_MANUAL_POSE
```

常见原因包括：

- 旋转净空不足；
- Scan、TF 或里程计过期；
- 里程计跳变；
- 扫描与地图不匹配；
- 候选位置存在歧义；
- AMCL 未采用人工或自动种子。

启动脚本失败时会自动关闭导航并清理速度链。确认现场原因已经消除后，重新运行所需的启动命令即可。

当前旋转安全规则会在完整 `/scan` 中持续检测到小于 `0.55 m` 的回波时拒绝旋转。不要通过手工绕过安全锁处理；将小车移到更有净空的位置后重试。将固定距离改为车体旋转扫掠包络的后续工作记录在 [GitHub Issue #7](https://github.com/shenfangqi/isaac-3d-lidar-amr-ws/issues/7)。

## 命令选择

| 目的 | 命令 | 自动旋转 | 激活 Nav2 | 自动发送 Goal |
|---|---|---:|---:|---:|
| 手工标定后导航 | `./start_real_nav.sh` | 否 | 是 | 否 |
| 只验证自动定位 | `./start_real_nav.sh --automatic` | 是 | 否 | 否 |
| 自动定位后导航待命 | `./start_real_nav.sh --automatic-activate` | 是 | 是 | 否 |
| 静止人工诊断 | `./start_real_nav.sh --stationary-validation` | 否 | 否 | 否 |
| 检查现有导航 | `./start_real_nav.sh --health-check` | 否 | 不改变 | 否 |
| 安全停止 | `./stop_real_nav.sh` | 否 | 关闭 | 否 |

更详细的状态机、实车证据和限制见：

- [`docs/real_robot_navigation_one_click.md`](docs/real_robot_navigation_one_click.md)
- [`docs/carbot_localization_validation.md`](docs/carbot_localization_validation.md)
