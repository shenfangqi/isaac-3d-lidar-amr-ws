# Carbot 真机参数适配 Isaac Sim / Isaac Lab 开发计划

更新时间：2026-09-13
状态：待实施
目标：在保留现有 MID360、nvblox、RViz2、SLAM/建图和 Nav2 接口的前提下，将当前 Nova Carter 仿真底盘替换为尽量接近 Carbot 真机的履带差速模型，并为后续 Isaac Lab 强化学习训练建立可复用环境。

## 1. 开工前状态和保护边界

- 当前本地基线提交：`ab4a855 feat: migrate lidar stack to MID360`。
- 本地 `main` 与远端 `origin/main` 已分叉：本地包含 MID360 迁移，远端新增 README。实施前先安全整合远端提交，不覆盖本地变更。
- 工作树仍有未跟踪的 `.codex_tmp/` 和 `docs/topology_build/node_modules`，不得加入正式提交。
- `warehouse_v1/v2/v3` 地图全部保留；适配期间不覆盖 `warehouse_v3`，新地图必须使用新版本名。
- 本计划中的未知动力学值必须标记为 `TEMP_ESTIMATE_NOT_CALIBRATED`，不得称为真机标定值。
- 仿真和真机不得在同一个可见 ROS Domain 中同时发布同名 `/odom`、TF 或 `/cmd_vel` 链路。

下一会话开始时先执行只读检查：

```bash
git status --short --branch
git log --oneline --decorate -5
git diff --name-status origin/main...HEAD
docker ps -a --format '{{.Names}}\t{{.Status}}'
```

确认工作树后，再整合远端 README 提交并建立 `codex/carbot-isaac-sim-adaptation` 开发分支。

## 2. 已确认的参数基线

### 2.1 运动学和几何

```yaml
drive_type: tracked_differential_skid_steer
physical_track_separation_m: 0.225
effective_track_separation_m: 0.254
effective_sprocket_radius_m: 0.02175
encoder_counts_per_revolution: 1560

overall_length_m: 0.285
overall_width_m: 0.266
overall_height_m: 0.240
base_link_height_m: 0.090
track_width_m: 0.041
track_thickness_m: 0.004
effective_track_contact_length_m: 0.145
total_working_mass_kg: 1.8
```

真实 footprint：

```yaml
footprint:
  - [ 0.155,  0.133]
  - [ 0.155, -0.133]
  - [-0.130, -0.133]
  - [-0.130,  0.133]
```

主体简化碰撞盒：

```yaml
size_m: [0.285, 0.266, 0.230]
position_relative_to_base_link_m: [0.0125, 0.0, 0.035]
```

轮组中心位置均以 `base_link` 为基准：

| 轮组 | X m | 左/右 Y m | Z m | 几何半径 m |
| --- | ---: | ---: | ---: | ---: |
| 前端导向轮 | 0.111250 | ±0.112500 | -0.0419 | 约 0.0210 |
| 前支重轮 1 | 0.067929 | ±0.112500 | -0.0687 | 约 0.0195 |
| 前支重轮 2 | 0.022929 | ±0.112500 | -0.0687 | 约 0.0195 |
| 后支重轮 1 | -0.022071 | ±0.112500 | -0.0687 | 约 0.0195 |
| 后支重轮 2 | -0.067071 | ±0.112500 | -0.0687 | 约 0.0195 |
| 后端主动轮 | -0.111250 | ±0.112500 | -0.0419 | 约 0.0210 |

几何轮半径用于 visual/collision；`0.02175 m` 只用于控制和里程计换算。

### 2.2 控制和安全限制

```yaml
max_wheel_rpm: 250.0
max_wheel_velocity_rad_s: 26.18
max_linear_velocity_mps: 0.50
max_angular_velocity_rad_s: 3.50
max_linear_acceleration_mps2: 0.50
max_angular_acceleration_rad_s2: 2.50
cmd_vel_timeout_s: 0.50
right_straight_trim: 1.0005
wheel_pid: {kp: 0.8, ki: 0.15, kd: 0.0, period_s: 0.01}
differential_period_s: 0.02
```

控制器还必须实施耦合轮速约束：

```text
|v| + |omega| * 0.254 / 2 <= 0.02175 * 26.18 ~= 0.5694 m/s
```

左右履带命令超限时按同一比例缩放，保持曲率。理想仿真默认不应用 `1.0005` 直行修正；该值留作高拟真或随机化参数，避免在 Isaac、Jetson 和 ESP32 重复修正。

### 2.3 MID360

```yaml
translation_m: [-0.003, 0.0, 0.132]
rotation_rpy_rad: [0.0, 0.0, 0.0]
pointcloud_topic: /livox/lidar
real_frame: livox_frame
pointcloud_rate_hz: 10
imu_topic: /livox/imu
imu_rate_hz: 200
real_points_per_frame_observed: 19584..20448
```

`z=0.132 m` 仍需最终装配后从 MID360 厂家坐标原点 O 复核。Isaac RTX 配置只是覆盖范围近似，不复制 Livox 非重复扫描、逐点时间、运动畸变和真实漏点。

### 2.4 真机编码器接口

```text
/wheel_ticks  carbot_msgs/msg/WheelTicks  50 Hz  Best Effort
```

消息包含 `sequence`、`boot_id`、`device_stamp_us`、`left_ticks`、`right_ticks`。车辆前进时左右累计计数都增加。ESP32 不发布 `/odom` 或 TF；后续真机闭环由 Jetson 节点积分 `/wheel_ticks`。

## 3. 必须保持的 ROS 接口

```text
/cmd_vel                 geometry_msgs/msg/Twist
/odom                    nav_msgs/msg/Odometry
/joint_states            sensor_msgs/msg/JointState
/tf, /tf_static
/livox/lidar             sensor_msgs/msg/PointCloud2
/livox/lidar_nvblox      sensor_msgs/msg/PointCloud2, padded 1000 x 40
/scan                    sensor_msgs/msg/LaserScan
/map                     nav_msgs/msg/OccupancyGrid
/nvblox_node/static_occupancy_grid
/navigate_to_pose
```

目标 TF：

```text
map
└── odom
    └── base_footprint
        └── base_link
            ├── lidar_link
            │   ├── livox_frame
            │   └── front_3d_lidar   # 仿真旧接口兼容别名
            └── imu_link
```

发布责任必须唯一：

- `map -> odom`：仿真 ground truth、AMCL 或定位系统三选一。
- `odom -> base_footprint`：Isaac 仿真里程计或 Jetson 真机里程计二选一。
- `base_footprint -> base_link` 及传感器固定外参：机器人描述发布。

## 4. 实施阶段

### 阶段 A：基线整理和参数单一来源

1. 安全整合远端 README 提交并建立开发分支。
2. 记录完整 `git status`，保护未跟踪和用户修改。
3. 新增 Carbot 公共参数文件，集中保存几何、运动学、限制、外参和临时动力学参数。
4. 为已测值、源码值、地面标定值和临时估算值增加明确来源标签。
5. 为关键换算和耦合轮速限制增加单元测试。

建议文件：

```text
src/carbot_description/config/carbot_parameters.yaml
src/carbot_description/test/test_carbot_parameters.py
```

### 阶段 B：Carbot URDF/Xacro 和 USD 模型

1. 新建 `carbot_description` 包，使用 Xacro/URDF 作为 ROS frame、visual、collision 和 joint 的可审查源。
2. 建立 `base_footprint`、`base_link`、`lidar_link`、`livox_frame`、`front_3d_lidar` 和 `imu_link`。
3. 按表格建立左右六组轮、履带 visual 和碰撞体。
4. 第一阶段采用“履带外观 + 多轮接触”的 skid-steer 近似，不实现高成本的完整柔性履带链节。
5. 主动轮/导向轮和支重轮使用各自几何半径；控制层使用有效半径。
6. 建立本地 Carbot USD articulation，替换场景对远程 Nova Carter 机器人资产的引用。
7. 尽量保持场景、灯光、地图原点和 ROS Graph 外围结构不变。

建议文件：

```text
src/carbot_description/urdf/carbot.urdf.xacro
src/carbot_description/meshes/                 # 有 CAD 时再加入
src/carbot_description/launch/description.launch.py
isaac_sim/usd/carbot.usd
isaac_sim/scripts/build_carbot_usd.py
```

### 阶段 C：Isaac Sim 差速控制和 ROS Graph

1. 使用 tracked differential/skid-steer 控制，不创建 steering joint，不使用 Ackermann 控制器。
2. 几何横向位置使用 `0.225 m`；控制器和里程计使用 `0.254 m`。
3. 控制有效半径使用 `0.02175 m`。
4. 接收 `/cmd_vel`，实施车体限速、加速度限制、500 ms watchdog 和耦合轮速饱和。
5. 确认 `linear.x > 0` 向 `+X`，`angular.z > 0` 左转；关节方向通过小速度测试确定。
6. 发布 `/odom`、`odom -> base_footprint`、`/joint_states`、`/clock`。
7. 仿真里程计的 joint/tick 换算与 ESP32 的 1560 counts/rev 语义一致。
8. 不在理想模型中默认复制 ESP32 PID；若复现执行器响应，则作为单独高拟真模式。

### 阶段 D：MID360 重新挂载与 nvblox 回归

1. 将 RTX MID360 proxy 移到实测外参 `[-0.003, 0, 0.132]`、RPY 0。
2. 保留 `/livox/lidar`；仿真发布 PointCloud2。
3. 同时提供 `livox_frame`、`lidar_link` 和旧 `front_3d_lidar` 的兼容 TF。
4. 保留 `/pointcloud_padder` 和 `/livox/lidar_nvblox` 的 `1000 x 40` 约束。
5. 保存地图导航初期继续使用 `lidar_min_valid_range_m=0.5`，避免改变已验证 `warehouse_v3` 行为；Carbot 自遮挡验证后再决定是否降低。
6. 检查新底盘和履带是否进入点云视野；通过过滤或碰撞/可见性配置解决自反射，不用错误外参掩盖问题。

### 阶段 E：sim/real 启动和 Nav2 配置分离

建立清晰的公共层、仿真层和真机层，不再在同一 launch 内写死时间源和 odom relay。

```text
configs/carbot/common.yaml
configs/carbot/sim.yaml
configs/carbot/real.yaml
configs/nav2_params_sim.yaml
configs/nav2_params_real.yaml
launch/carbot_sim.launch.py
launch/carbot_real.launch.py
```

仿真：

- `use_sim_time=true`。
- 使用 `/clock`。
- Isaac 产生 odom/TF。
- 可以保留 `/chassis/odom -> /odom` 兼容层，但新模型优先直接输出 `/odom`。
- Nav2 运行速度初期保留当前保守值约 `0.30 m/s`、`0.35 rad/s`，不因硬件最大能力而强行提速。

真机：

- `use_sim_time=false`。
- 禁止 Isaac `/chassis/odom` relay 和 ground-truth TF。
- `/wheel_ticks -> /odom + TF + joint_states` 由 Jetson 节点负责。
- 首次导航限制为 `0.10 m/s`、`0.30 rad/s`。

Nav2：

1. 用实测 polygon `footprint` 替换 `robot_radius=0.35`。
2. 当前 `inflation_radius=0.45` 先作为保守基线保留；确认静态地图与动态障碍物影响后再调小。
3. 将 `robot_base_frame` 统一为 `base_footprint`，同时保证 `map -> base_link` 仍可解析。
4. 保留 `/scan`、`/odom`、`/cmd_vel` 和现有 Nav2 Action 名称。
5. 同步 controller、behavior server 和 velocity smoother 的速度/加速度约束。

### 阶段 F：Isaac Lab 基础环境

1. 先创建与 Nav2/真机控制边界一致的高层环境，action 使用 `[linear.x, angular.z]`。
2. 观察量第一版使用基础状态、目标相对位姿、速度和降采样 LiDAR/高度信息。
3. 将动作统一送入与 `/cmd_vel` 相同的限速、加速度和 watchdog 层，避免训练策略绕过安全模型。
4. 任务、奖励函数和课程学习作为独立配置，不耦合到底盘 USD。
5. 预留左右履带速度 action 模式；在摩擦和执行器完成标定前，不以 PWM/转矩为主要 Sim-to-Real action。

建议结构：

```text
isaac_lab/carbot_env/
├── carbot_env_cfg.py
├── mdp/
├── tasks/
└── tests/
```

### 阶段 G：验证和验收

#### 静态验证

- Bash、Python、YAML、JSON 语法检查。
- URDF/Xacro 检查和 TF 无环检查。
- 参数单位、轮距/半径用途和速度耦合单元测试。
- 检查工作树，确保无无关用户修改被覆盖。

#### Isaac 单体运动验证

1. 零命令静止且 500 ms watchdog 有效。
2. 正向/反向小速度测试。
3. 正负角速度原地旋转测试。
4. 左右关节方向、轮速和饱和比例正确。
5. 1 m 直行距离与有效半径一致。
6. 90°/360° 转向与有效轮距一致。
7. 检查碰撞穿透、悬空、侧翻和履带异常打滑。

#### ROS 接口验证

必须有实时消息并检查类型、QoS、frame 和频率：

```text
/cmd_vel
/odom
/joint_states
/tf
/tf_static
/clock
/livox/lidar
/livox/lidar_nvblox
/scan
```

必须验证：

```text
map -> odom -> base_footprint -> base_link -> lidar_link/livox_frame
```

#### nvblox、RViz 和 Nav2 回归

- 恰好一个 `/pointcloud_padder`、`/nvblox_node` 和 `/nvblox_container`。
- padded 点云保持 `1000 x 40`。
- `warehouse_v3` 地图尺寸、分辨率和 unknown/free/occupied 统计不被意外改变。
- RViz Fixed Frame 为 `map`，点云、Scan、footprint、costmap 和 TF 对齐。
- Nav2 lifecycle 和 `/navigate_to_pose` 正常。
- 执行一个安全的短距离直线 Goal 和一个带转向的短距离 Goal，记录终态和实际轨迹。
- 若模型替换破坏保存地图定位，先诊断 spawn、odom 和 LiDAR 外参，不覆盖 `warehouse_v3`。

## 5. 临时动力学值和 Isaac Lab 随机化

第一阶段建议基准，仅用于启动仿真：

```yaml
TEMP_ESTIMATE_NOT_CALIBRATED:
  center_of_mass_m: [0.0125, 0.0, 0.035]
  inertia_kg_m2:
    ixx: 0.01855
    iyy: 0.02012
    izz: 0.02280
    ixy: 0.0
    ixz: 0.0
    iyz: 0.0
  static_friction: 0.8
  dynamic_friction: 0.6
  restitution: 0.0
  per_side_effort_limit_nm: 0.5
  joint_damping_nm_s_rad: 0.02
```

若物理后端支持方向摩擦，履带横向摩擦应低于纵向摩擦以允许滑移转向；不支持时先用折中各向同性摩擦并通过域随机化覆盖。

初始训练随机化建议：

```yaml
mass_kg: [1.6, 2.0]
com_offset_m: {x: [-0.02, 0.02], y: [-0.02, 0.02], z: [-0.02, 0.02]}
effective_track_separation_m: [0.24, 0.28]
effective_radius_m: [0.0205, 0.0230]
friction_scale: [0.6, 1.4]
actuator_strength_scale: [0.75, 1.25]
command_latency_s: [0.02, 0.15]
point_dropout_fraction: [0.0, 0.05]
```

以上范围是训练假设，不是真机标定置信区间。

## 6. 后续真机标定优先级

1. 同步记录 `/cmd_vel`、`/wheel_ticks`、`/imu/data_raw`、目标/实际 RPM、PWM、电池电压。
2. 直行和反向 1 m，复核有效半径与左右不对称。
3. 左右 90°/360°，复核不同地面的有效轮距和侧滑范围。
4. 阶跃加速、制动和滑行，拟合延迟、转矩、阻尼和死区。
5. 称重/支点法估算重心；用 CAD 或摆动试验更新惯量。
6. 拉力计或受控斜坡试验估算履带摩擦。
7. 复核 `imu_link` 安装外参和 MID360 原点 O 的 Z。

标定结果必须回填公共参数来源记录；不得用调 Nav2 参数掩盖底盘或 TF 错误。

## 7. 建议提交拆分

1. `chore: reconcile project baseline and add carbot parameters`
2. `feat: add carbot robot description and usd articulation`
3. `feat: add tracked differential simulation control`
4. `feat: align MID360 frames and preserve nvblox interfaces`
5. `feat: split carbot simulation and real robot bringup`
6. `feat: add carbot Isaac Lab environment`
7. `test: validate carbot simulation navigation stack`

每个提交都应可单独审查，避免把生成文件、用户修改和功能代码混在一起。

## 8. 第一阶段完成条件

- 场景不再依赖 Nova Carter 作为机器人底盘。
- Carbot 几何、质量、履带中心距、有效轮距和有效半径用途正确。
- 不存在 Ackermann steering joint 或 Ackermann 控制路径。
- `/cmd_vel`、`/odom`、`/joint_states`、TF、点云和 `/clock` 正常。
- MID360/nvblox padding、保存地图加载和 RViz 显示无回归。
- Nav2 完成至少一次短距离导航并记录实际结果。
- sim/real 时间源、odom 来源和 DDS 环境明确分离。
- Isaac Lab 能批量 reset Carbot 环境并执行受限 `[v, omega]` action。
- 所有临时估算和仍需标定项在配置和报告中显式标注。
- 最终报告列出修改文件、执行过的验证、未执行的验证、临时假设和真机待标定参数。

## 9. 下一会话启动指令

建议在新会话直接发送：

```text
请读取 docs/carbot_isaac_sim_adaptation_plan.md，按阶段 A 开始实施。
先检查并安全整合当前分叉的 main/origin/main，保护所有用户修改和未跟踪文件；
完成参数单一来源与测试后汇报，再进入 Carbot URDF/USD 模型阶段。
```
