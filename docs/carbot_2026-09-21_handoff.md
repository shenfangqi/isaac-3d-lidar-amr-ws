# Carbot 会话交接记录（2026-09-21）

本文是下一会话继续 Carbot 真机对齐、MID-360、Isaac Sim、架空障碍和局部避障
工作的首要入口。开始工作前还应读取：

- `skills/isaac-amr-project-state/SKILL.md`
- `skills/isaac-amr-project-state/references/current-state.md`
- `skills/isaac-amr-project-state/references/jetson-target.md`
- `skills/ros-docker-debug/SKILL.md`

旧的 `docs/carbot_2026-09-19_handoff.md` 保留完整历史，但状态冲突时以本文和
`current-state.md` 的最新 checkpoint 为准。

## 1. 当前结论

当前已达到“导航级基本对齐”，尚未达到“高保真数字孪生”或“强化学习策略可直接
无条件部署”的程度。

2026-09-23 质量/重心补充：用户报告整车装配质量为 `3.4 kg`，重心位于车体
几何正中央。规范模型现在使用 `3.28 kg` 合并车体加十二个 `0.01 kg` 轮体；由于
`base_link` 原点和前后不对称 footprint 的几何中心不重合，中心在该坐标系中仍为
`[0.0125, 0.0, 0.035] m`。惯量只按车体质量保持回转半径进行了临时缩放，尚未
取得 CAD 或摆测证据，因此高保真惯量项目仍未关闭。

2026-09-22 补充结论：在线 MID-360/轮速 EKF 已完成受控真机录包、点云 ICP
外部基准 A/B、部署和静态验收。`/wheel/odom` 是轮编码器原始输入，EKF 是
`/odom` 和 `odom -> base_footprint` 的唯一动态所有者。融合使用轮速 X/Y、轮
积分航向低频锚点和 MID-360 `gyro.z` 高频响应；动态过程噪声已启用。ESP32 仅
负责轮编码器和底盘控制，其旧 `/imu/data_raw` 不进入融合。详见
`calibration_data/2026-09-22_ekf_dynamic/README.md`。

2026-09-23 补充结论：右转补偿 `0.896` 已固化为 Jetson 正式控制链
`/cmd_vel_command -> cmd_vel_compensator -> /cmd_vel -> ESP32`，并由 enabled
的 `carbot-command-compensation.service` 托管；真机 Nav2 默认输出已改到
`/cmd_vel_command`。Isaac Sim 使用同一补偿和实测未补偿右转增益，左右转仿真
残差为 `0°`。最终真机前后/左右序列的位置回位误差为 `0.874 mm`，最终航向
误差 `0.095°`，单独左右转配对残差 `1.414°`。ESP boot ID 未变化，结束后
编码器静止，所有服务 active。未修改 ESP32 固件。证据见
`calibration_data/2026-09-23_cmd_comp_final/README.md`。

2026-09-23 断电离线补充：新增独立的 `evidence_degraded` 仿真模式，证据配置由
已有真机录包自动生成到 `configs/carbot/evidence_degraded.yaml`。该模式加入命令
延迟、有限响应、停止拖尾、编码器量化/稀疏丢包重复、MID-360 gyro 残差以及点云
延迟/噪声/dropout；`evidence_ekf` 路径把真值隔离在 `/ground_truth/odom`，并让
真实同构的轮里程计和 MID-360 EKF 独占 `/odom`。离线响应回归已通过。车辆全程
断电，未连接 Jetson、雷达或 ESP32，也未修改 ESP32 仓库。该结果仍不是质量、
摩擦、履带接触、电机电气特性均已标定的高保真动力学孪生。

2026-09-23 通电静止耐久补充：完成 32.76 分钟联合录包。ESP 全程同一 boot ID，
时间同步有效、重连和 ping 失败均为零，轮计数保持 `0/0`；六项 Jetson 服务在
31 个分钟采样点均为 `active/running` 且 `NRestarts=0`。MID-360 在线频率为约
`200 Hz` IMU 和 `10 Hz` 点云，所有关键 header 严格单调。融合位置无漂移，静止
航向净漂移 `0.905 deg`、最大偏离 `1.180 deg`。最终运动 bag 的 `1.414 deg`
转向残差由轮里程、MID-360 gyro 和 EKF 分别复现为 `1.423/1.438/1.403 deg`，
因此属于单次物理响应差异而不是 EKF 独有误差。完整证据见
`calibration_data/2026-09-23_static_durability/README.md` 和
`calibration_data/2026-09-23_cmd_comp_final/physical_final_04_detailed_analysis.json`。
通过 MID-360 SDK2 内部状态键完成精确 PTP slave offset 查询：5 次 offset 平均
`-26.014 us`、中位数 `-25.888 us`、范围 `-28.612..-23.034 us`，全部
`time_sync_type=1`。master 端 `pmc` 的 `offsetFromMaster=0` 仅表示 Jetson 是
grandmaster，不能代替 slave offset；MID-360 未通过管理 TLV 暴露 mean path delay。

2026-09-23 闭环航向补充：连续开环试验证明 `0.30 rad/s` 位于履带原地转向死区
边缘，并且 `0.896/0.940/0.911` 三组固定比例结果随履带/地面状态非单调变化，
因此拒绝继续用单一比例追求定角精度。使用 MID-360/轮速 EKF `/odom` 反馈的四组
`+/-30 deg` 定角往返，回位误差分别为 `+0.767/-0.994/+0.722/+0.702 deg`，
累计位置误差 `3.285 mm`。真机 Nav2 航向容差已由 `0.25 rad` 收紧至
`0.03 rad`，旋转使用实测可靠的 `0.40..0.50 rad/s` 区间；`0.896` 仅保留为
名义执行器补偿，最终航向由反馈闭环负责。Jetson 已安装 Nav2 依赖和真机
bringup 包，非驱动启动及回归测试通过。证据见
`calibration_data/2026-09-23_closed_loop_yaw/README.md`。
同一四组目标序列通过 Isaac `evidence_degraded` 执行器层回归，最大单组回位误差
`0.671 deg`、累计误差 `-0.090 deg`；原有响应回归保持全通过。该结论闭合的是
真机/Isaac 的反馈控制契约，不替代尚未标定的 PhysX 质量、摩擦和履带接触动力学。

已完成的主线包括：

- 车体主要尺寸、footprint、车高、有效轮径和有效履带间距对齐；
- Jetson、Livox ROS Driver 2、MID-360、完整点云、IMU adapter 和 XYZ 派生流
  在线闭环；
- 真机基础建图、AMCL、短路线 Nav2 和返回路线已有历史验收；
- Isaac Sim 中动态障碍绕行和 `0.35 m` 架空安全阈值已有仿真闭环；
- MID-360 官方参数、RTX 近似字段、驱动配置和一致性测试已纳入仓库。

仍未完成的关键项目包括：

- LiDAR XYZ/RPY 已由地面反转和正交墙面测量闭环，但仍建议用测绘夹具独立
  复核约 `2 mm` X、`0.5 mm` Y 和 `0.20 deg` yaw 系统不确定度；MID-360
  内置 IMU 旋转/杠杆臂和点云相对时间偏差已完成；
- 新局部避障改动的真机静止检查和低速障碍验收；
- `0.35 m` 架空安全阈值的受控真机验收；
- 不同地面、载荷下的履带侧滑、制动、质量分布和惯量建模；
- 独立于 ESP32、网络和上层软件的硬件急停。

## 2. 安全边界

- 2026-09-23 最终补偿验收在用户确认场地净空后执行了受控低速前进、后退和
  左右原地旋转；每段均以显式零速结束。Nav2 未启动。最终
  `/cmd_vel_command` 发布者为 0；`/cmd_vel` 仅保留静默的补偿服务发布端，轮
  计数静止，补偿、EKF、轮里程计和 MID-360 服务保持 active。
- 真车没有合格的独立硬件急停。手机页面的红色停止按钮不能作为独立急停。
- 未重新确认现场清空、线缆固定、电池状态和可立即断电前，不得进行运动测试。
- 工作站 Isaac ROS 图与真机同为 `ROS_DOMAIN_ID=0`。接入真机 DDS 前必须确认
  Isaac Sim、工作站 nvblox 和仿真 Nav2 已停止，防止话题、TF 或 `/cmd_vel`
  污染。
- 不得让真车尝试通过 `0.26 m` 床底或仿真的 `0.28 m` 阻挡横梁。当前自主通行
  阈值为 `0.35 m`；真机首次验收应只使用明显高于该阈值、可随时移除且不会损伤
  顶置雷达的测试结构。

## 3. Git 与工作区状态

- 仓库：`/home/shenfq/projects/ros-humble/isaac_3d_lidar_amr_ws`
- 分支：`codex/carbot-isaac-sim-adaptation`
- 当前 HEAD：`b51ea2041929`
- 2026-09-21 新增修改尚未提交。

本轮主要未提交内容分成两组：

### 3.1 架空障碍和 RViz 可视化

- `configs/carbot/common.yaml`
- `configs/rviz/carbot_navigation.rviz`
- `docs/map_nav/README.md`
- `isaac_sim/auto_play_carbot.py`
- `isaac_sim/streaming_carbot.py`
- `isaac_sim/overhead_clearance_obstacles.py`
- `launch/carbot_navigation.py`
- `scripts/validate_overhead_clearance.py`
- `src/carbot_description/test/test_carbot_phase_e.py`
- `src/isaac_3d_lidar_bringup/isaac_3d_lidar_bringup/overhead_clearance_marker_publisher.py`
- `src/isaac_3d_lidar_bringup/package.xml`
- `src/isaac_3d_lidar_bringup/setup.py`

### 3.2 MID-360 参数、驱动配置和在线验收

- `isaac_sim/MID360_SIM.md`
- `isaac_sim/lidar_configs/Livox_Mid360_Approx.json`
- `scripts/validate_mid360_live.py`
- `src/carbot_description/config/carbot_parameters.yaml`
- `src/carbot_description/test/test_carbot_mid360.py`
- `src/carbot_hardware/config/MID360_config.json`
- `src/carbot_hardware/config/MID360_config.md`
- `src/carbot_hardware/carbot_hardware/pointcloud_xyz_relay.py`
- `src/carbot_hardware/launch/mid360_stack.launch.py`
- `src/carbot_hardware/setup.py`
- `src/carbot_hardware/test/test_pointcloud_xyz_relay.py`
- 本文、`docs/carbot_2026-09-19_handoff.md` 和
  `skills/isaac-amr-project-state/references/current-state.md`

用户目录，禁止删除或提交：

- `.codex_tmp/`
- `docs/topology_build/node_modules`

下一会话在提交前必须重新检查完整 diff，不要把“未提交”误判为“未验证”，也不要
回滚其他组已经完成的修改。

## 4. Jetson 与 MID-360 网络

Jetson 同时拥有两个正确且用途不同的地址：

| 设备/接口 | 地址 | 用途 |
|---|---:|---|
| Jetson Wi-Fi `wlP1p1s0` | `192.168.1.109/24` | SSH、工作站 ROS 2 DDS |
| Jetson Ethernet `enP8p1s0` | `192.168.2.100/24` | MID-360 专网接收端 |
| MID-360 | `192.168.2.202` | 雷达设备地址 |

Livox 配置中的 host IP 必须是 `192.168.2.100`，不能改成 Wi-Fi 地址
`192.168.1.109`。2026-09-21 在线检查中，雷达三次 ping 零丢包；
`enP8p1s0` RX/TX errors、drops、missed、carrier 和 collisions 均为零。

本机 `ubuntu.local` mDNS 当时解析失败，但记录地址 `192.168.1.109` 可正常 SSH，
主机身份为 `ubuntu`、用户为 `shenfq`、架构为 `aarch64`。优先尝试 SSH alias；若
mDNS 仍失败，可在重新确认地址后临时使用：

```bash
ssh -o HostName=192.168.1.109 isaac-jetson
```

## 5. MID-360 仓库配置与部署状态

官方厂家参数的唯一项目源位于：

```text
src/carbot_description/config/carbot_parameters.yaml
sensors.mid360.manufacturer_specs
```

仓库自带驱动配置：

```text
src/carbot_hardware/config/MID360_config.json
```

配置采用：

- `lidar_type=8`
- `pcl_data_type=1`（32-bit Cartesian）
- `pattern_mode=0`（非重复扫描）
- Jetson host `192.168.2.100`
- MID-360 `192.168.2.202`
- JSON extrinsic 全零，实际外参由 ROS TF 唯一管理

Jetson 部署位置：

```text
/home/shenfq/Projects/carbot-ros2
```

2026-09-21 已部署并构建 `carbot_hardware`。工作站、Jetson source、Jetson install
三者的关键文件 SHA-256 完全一致。驱动在线参数确认：

```text
user_config_path=/home/shenfq/Projects/carbot-ros2/install/carbot_hardware/share/carbot_hardware/config/MID360_config.json
xfer_format=0
publish_freq=10.0
```

部署前备份位于 Jetson `/tmp`，重启后可能消失：

```text
/tmp/carbot_hardware_pre_mid360_20260921_1613.tgz
/tmp/carbot_hardware_build_pre_20260921_1616
```

Jetson 的 setuptools 不支持 `colcon --symlink-install` 当前使用的 editable 模式。
真机部署应使用普通构建：

```bash
cd /home/shenfq/Projects/carbot-ros2
source /opt/ros/humble/setup.bash
source /home/shenfq/Projects/lidar-mid360/ws_livox/install/local_setup.bash
colcon build --packages-select carbot_hardware
```

## 6. MID-360 在线验收结果

唯一原始点云 `/livox/lidar` 已确认：

- 类型：`sensor_msgs/msg/PointCloud2`
- frame：`livox_frame`
- 字段：`x/y/z/intensity/tag/line/timestamp`
- `point_step=26`
- 样本点数：`19968` 点/帧
- 发布者数量：1
- 直接 CLI 频率约 `10.00 Hz`

派生流 `/mid360/points_xyz`：

- 仅保留 `x/y/z`
- stride 4，用于 Wi-Fi/nvblox 带宽优化
- 直接 CLI 频率约 `10.00 Hz`
- 不能代替原始流用于 timestamp、tag、line、反射质量或标定分析

适配后的 `/mid360/imu/data_raw` 直接 CLI 频率约 `199.95 Hz`。

`scripts/validate_mid360_live.py --duration 8` 的联合采样结果：

- raw `9.75 Hz`
- compact `9.99 Hz`
- IMU `191.74 Hz`（单线程同时处理三条流时的观察值）
- 逐点 timestamp 为 epoch 纳秒
- 首点与 ROS header 相差约 `0.24 us`
- 单帧首尾点时间跨度约 `100.01 ms`
- header 到验收节点 wall-clock 接收约 `120.75 ms`

运行只读在线验收：

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source /home/shenfq/Projects/lidar-mid360/ws_livox/install/local_setup.bash
source /home/shenfq/Projects/carbot-ros2/install/local_setup.bash
python3 /home/shenfq/Projects/carbot-ros2/scripts/validate_mid360_live.py --duration 8
```

上线时发现 ROS 2 Humble 会把字符串数组中的裸字段名 `y` 按 YAML 布尔值解析，
导致 relay 退出。字段门禁参数已改为 CSV 字符串：

```text
required_input_fields_csv=x,y,z,intensity,tag,line,timestamp
```

修复后驱动、IMU adapter、relay 三个节点同时存在，服务 `NRestarts=0`，最新启动
日志无 warning/error。

## 7. PTP 与登录服务状态

系统级进程正在运行：

```text
/usr/sbin/ptp4l -f /etc/linuxptp/carbot-mid360-ptp.cfg -i enP8p1s0 -m
```

配置中 Jetson 是隔离雷达链路的 PTP master，MID-360 是 slave。管理 socket：

```text
/run/carbot-mid360-ptp/ptp4l
root:root 0660
```

2026-09-23 已用交互式 sudo 只读查询该 socket：Jetson
`CURRENT_DATA_SET` 为 `stepsRemoved=0`、`offsetFromMaster=0`、
`meanPathDelay=0`，端口状态 `MASTER`。这些是 grandmaster 自身值，不是 MID-360
slave offset。随后通过 SDK2 内部状态键 `0x8009..0x800C` 直接查询雷达，5 次
`time_offset` 为 `-24.608/-23.034/-25.888/-28.612/-27.926 us`，平均
`-26.014 us`、标准差 `2.063 us`，全部 `time_sync_type=1`。只读抓包同时确认
雷达 `192.168.2.202` 每秒发送 Delay_Req，Jetson 对同序列返回 Delay_Resp，内核
零丢包。雷达不响应网络 management TLV，因此没有把 master 的零值误写成 slave
offset，也没有声称取得未暴露的 slave mean path delay。

`loginctl` 仍显示 `Linger=no`。这意味着用户级的 MID-360、description、wheel
odometry 和 micro-ROS 服务需要至少一个有效用户登录会话；本轮未擅自启用 linger。

## 8. Isaac Sim 对齐状态

当前 Isaac Sim 属于导航级模型，不是完整物理数字孪生。

已对齐：

- 主要车体尺寸、footprint、车高和雷达安装基线；
- 有效轮径 `0.02175 m`、有效履带间距 `0.254 m`；
- 导航速度/角速度/加速度范围；
- 已观察的前后死区、延迟和停止拖尾范围；
- MID-360 官方量程、FoV、点率、频率和基础机械尺寸；
- ROS 话题、frame 和接口契约。

未对齐：

- 完整履带接触、不同地面/载荷侧滑；
- 质量分布、重心、惯量、电机扭矩和电池电压影响；
- Livox 真实非重复扫描模式；
- 玻璃、黑色材料、金属、反射率、温度噪声和运动畸变；
- LiDAR XYZ/RPY 已由 2026-09-21 A-B-A 静态反转与正交墙面测量闭环，
  MID-360 内置 IMU 旋转/杠杆臂和点云相对时间偏差也已闭环；LiDAR 外参仍应在
  测绘夹具上独立复核。

可选择的仿真运行模式：

```bash
# 导航确定性基线（默认）
CARBOT_SIM_RESPONSE_MODE=ideal_navigation

# 真机证据退化 + 与 Jetson 相同的轮速/MID-360 EKF 接口
CARBOT_SIM_RESPONSE_MODE=evidence_degraded \
CARBOT_SIM_ESTIMATOR_MODE=evidence_ekf
```

第二种模式需同时启动
`carbot_hardware/launch/sim_evidence_state_estimation.launch.py`；仿真原始点云
先发布到 `/livox/lidar_ground_truth`，退化节点再发布正式 `/livox/lidar`。

2026-09-21 A-B-A 静态标定结果：Pose C 对 Pose A 的地面法向复现误差为
`0.127 deg` roll、`0.019 deg` pitch，高度差 `0.49 mm`。分离反转地面/履带
支撑分量后，固定 MID-360 安装角为 roll `-0.3515327 deg`、pitch
`-0.2783163 deg`。随后车体中心线平行墙面的 64 帧静态拟合得到 yaw
`+1.1206755 deg`（`+0.019559477 rad`）；逐帧墙法向标准差 `0.01011 deg`，
另一侧墙给出 `+1.3218736 deg`，两者 `0.2012 deg` 的差异作为墙体/摆放系统
误差保留。该结果已写入 canonical YAML、Jetson 安装空间和重建后的 Isaac
USD；机械 Z 保持 `0.209 m` 基线。

同一位姿下，右墙到近侧履带最外沿实测 `289 mm`；canonical 履带外半宽为
`225/2 + 41/2 = 133 mm`，所以车体中心线距墙 `422 mm`。点云拟合雷达距墙
`421.85533 mm`，解得安装 Y 为 `-0.14467 mm`（中心线右侧），尺量不确定度约
`+/-0.5 mm`。后墙到右/左后驱轮最后端外缘实测 `1153/1155 mm`；结合均值
`1154 mm`、canonical 后轮外缘 `x=-132.25 mm`、点云后墙距离
`1300.78363 mm` 以及墙面 `0.71955 deg` 竖直倾斜，解得安装
`X=+16.56608 mm`。左右测量差 `2 mm` 作为 X 的约 `+/-2 mm` 不确定度保留；
若忽略墙面倾斜只作水平相减会得到 `+14.53363 mm`，因此未采用。

MID-360 IMU 动态标定使用 `243.046 s` 的人工往复偏航数据，包含 `48160` 个
IMU 样本。点云按 20 ms 窗口拟合出 `3595` 个墙面偏航姿态；与去偏置积分
gyro Z 联合拟合后，时间 lag 为 `+9.782937 ms`（IMU 时间戳晚于点云有效
时间），所以适配后的 `/mid360/imu/data_raw` 对原始时间戳加
`-0.009782937 s`。三段独立结果为 `9.1673/10.1891/10.2468 ms`，保留约
`0.6 ms` 不确定度；全量拟合 `R^2=0.9999084`、RMS `0.119 deg`、gyro scale
`0.996542`。官方手册明确 IMU 与点云坐标轴同向，并给出芯片位置
`[11.0, 23.29, -44.12] mm`，因此 canonical `livox_frame -> imu_link` 为该
平移加单位旋转。适配输出 frame 改为 `imu_link`；原始 `/livox/imu` 不改。
本次零偏仅记录为证据，不作为温漂无关常量写死。

RTX 的 `rangeAccuracyM=0.03 m` 与官方保守 1-sigma 上限绑定；
`rangeResolutionM=0.01 m` 和角度标准差 `0.05 deg` 是明确的仿真假设，不是厂家
实测声明。

## 9. 架空障碍和局部避障状态

Isaac Sim 每次启动会生成两根长期测试横梁：

- 绿色 `0.40 m` 净空：应允许通过；
- 红色 `0.28 m` 净空：应阻挡并绕行。

RViz 通过 `/overhead_clearance_markers` 显示对应横梁和文字标签。仿真闭环结果：

- `0.40 m` 横梁 `/scan` 零命中，路线直接通过，横向偏移 `0.000 m`；
- `0.28 m` 横梁最多 32 个 `/scan` 命中，路线绕行，最小横向偏移约 `1.205 m`；
- 两个目标和中间返航均 `SUCCEEDED`，最终速度为零。

动态障碍仿真也已覆盖：临时障碍等待/恢复、持续但可绕障碍重新规划、持续且不可绕
障碍安全终止。相关新局部避障链路尚未部署和运动验收到真机。

## 10. 最近验证记录

- 架空横梁相关 scoped regression：`41 passed, 8 skipped`。
- MID-360 离线定向测试：`25/25`。
- 隔离后的 `carbot_description` 包测试：`52/52`。
- IMU 外参与时间标定后通过 `carbot_description` 5 个 CTest 目标中的
  `49/49` 个 pytest case、`carbot_hardware`
  直接测试 `13/13`；Isaac Lab CPU
  contract `8/8`；重建 USD 的 parameter SHA 为
  `b079ed22e68807d53896cd86e1738ec72a535b4ead613c4f2dc7d416da75567b`。
- Jetson source/install YAML SHA 与上述值一致；live
  `base_link -> lidar_link` translation 为
  `[0.01656608, -0.00014467, 0.072] m`，`base_link -> livox_frame` rotation 为
  `-0.352/-0.278/+1.121 deg`；部署后编码器保持 `left=2, right=0`，
  `/cmd_vel` publisher 为 `0`。
- MID-360 IMU 新配置在线配对核验 `747` 个样本：adapted-minus-raw stamp
  中位数 `-9.783030 ms`，frame 为 `imu_link`，orientation covariance 首项
  `-1`；IMU/原始点云/精简点云频率分别为 `200.11/10.04/10.01 Hz`。实时
  `livox_frame -> imu_link` 为 `[0.011, 0.02329, -0.04412] m` 和单位旋转。
- CSV 兼容修复相关本地测试：`10/10`。
- Jetson relay 测试：`3/3`。
- `carbot_description`、`carbot_hardware` 本地构建成功。
- Jetson `carbot_hardware` 普通构建成功。
- Python flake8 和 `git diff --check` 通过。
- 当前工作站 `ros2-dev-humble`、`isaac-ros-nvblox`、`isaac-sim` 均已停止。

## 11. 下一步工作顺序

### P0：收口当前工作区

1. 重新审查完整 diff，特别检查架空障碍组和 MID-360 组之间没有意外耦合。
2. 重跑与提交范围相符的测试；不要把 `.codex_tmp/` 或
   `docs/topology_build/node_modules` 纳入版本控制。
3. 建立提交并推送；如 Issue #1 仍用于项目闭环，将仿真架空验收、MID-360 在线
   验收、测试结果和提交号写入 Issue。

### P1：只读静态真机补测

1. 在有交互式 sudo 的条件下读取 PTP master/slave 状态和精确 offset，不修改
   配置；保存命令、offset、mean/path delay 和时间。
2. MID-360 点云/IMU 相对时间偏差、轴向和杠杆臂已完成；保留当前动态 bag，
   在固件或驱动升级后用同一分析脚本复验。
3. LiDAR XYZ/RPY 已通过地面 A-B-A 反转和正交墙面尺寸测量完成；后续在
   独立测绘夹具上复核当前结果，不要用仿真假设代替真机测量。
4. 在 IMU 融合前，先完成 ESP32/wheel 时间戳周期重同步和受控定位 A/B 测试。

### P2：新避障链路真机静止验收

1. 确认工作站仿真 ROS 图完全关闭。
2. 仅部署真机需要的局部避障、Scan filter、行为树和依赖，不覆盖无关 Jetson
   文件。
3. 不激活 Navigation，先验证节点数量、话题频率、TF、QoS、CPU/内存、静态墙体
   遮罩和最终 `/cmd_vel` 发布拓扑。
4. 静止检查通过后再申请运动测试条件。

### P3：受控低速运动验收

必须先确认现场清空、车辆电量、线缆、操作员和立即断电手段。按以下顺序每类只做
一次短测试：

1. 无障碍短路线；
2. 临时障碍出现后停车、消失后恢复；
3. 持续可绕障碍触发重新规划；
4. 持续不可绕障碍保持安全停止/终止；
5. 明显高于 `0.35 m` 的软质或可脱离架空结构通过测试。

任何异常立即发零速度并断开驱动条件；不要为了让测试通过而降低 footprint、
inflation 或 `0.35 m` 安全阈值。

### P4：高保真 Sim-to-Real

- 不同地面和载荷下辨识有效履带间距、轮径、侧滑和停止拖尾；
- 测量质量分布、重心、惯量和执行器动态；
- 从真机 bags 建立点云 dropout、噪声、反射率和延迟模型；
- 更新 Isaac Lab domain randomization；
- 在独立硬件急停和 observation parity 完成前，不释放真机强化学习策略。

## 12. 下个会话可直接使用的指令

```text
请先完整读取：
/home/shenfq/projects/ros-humble/isaac_3d_lidar_amr_ws/docs/carbot_2026-09-21_handoff.md
以及其中列出的 project-state、jetson-target 和 ros-docker-debug 技能文件。

先检查 git status、容器状态和真机连接；不要回滚当前未提交修改，不要处理或提交
.codex_tmp/ 与 docs/topology_build/node_modules。当前 MID-360 仓库配置已经部署到
Jetson 并在线验收，完整点云、10 Hz 点云、约 200 Hz IMU 和逐点纳秒时间戳均已
确认。工作站仿真容器已停止。下一步先按 P0 审查 diff、测试、提交和推送；需要真机
操作时严格按 P1/P2/P3 顺序，不得发送未经明确授权的运动命令。
```
