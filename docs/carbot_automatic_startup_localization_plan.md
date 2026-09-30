# Carbot 保存地图开机自动定位开发方案

> 2026-09-30 最新状态：严重假阳性尚未闭环，自动 Nav2 激活已在本地验证版本锁定。以下早期方案和历史操作步骤不是当前实车放行依据。执行步骤与未完成验收见 [验证记录](carbot_localization_validation.md)。

## 1. 目标与边界

在已经存在 Nav2 二维地图的前提下，实现以下无人值守启动闭环：

1. 启动 FAST-LIO2，并等待里程计、TF、点云和 `/scan` 连续稳定。
2. 只激活 `map_server` 与 AMCL，调用 AMCL 全局重定位，使粒子分布覆盖地图自由区。
3. 在导航 lifecycle 仍未激活时，以实车可执行的最低安全角速度原地旋转约一圈，持续进行 `/scan` 与静态地图匹配。
4. 定位通过多指标可信度检查后，停止并确认底盘静止，允许 AMCL 独占发布 `map -> odom`，然后激活 Nav2 导航。
5. 定位超时、质量不足或运行异常时立即停车，保持导航未激活，并明确提示操作者在 RViz 点击一次 `2D Pose Estimate`；收到人工 `/initialpose` 后重新走同一套可信度检查。

首版不处理无地图建图、跨楼层定位、动态地图更新和自动平移脱困。全局定位只在地图的自由栅格中撒粒子，而不是在障碍和未知栅格中撒粒子。

## 2. 关键设计结论

### 2.1 `map -> odom` 只能有一个发布者

定位成功后的“固定 `map -> odom`”定义为：确认 AMCL 已建立可信变换，锁定启动状态并开放导航，而不是额外发布一条静态 TF。导航运行期间继续由 AMCL 动态维护 `map -> odom`，以修正 FAST-LIO2/履带里程计的长期漂移。

禁止在 AMCL `tf_broadcast: true` 时再启动 `static_transform_publisher`。如果将来确实需要冻结模式，必须先停用 AMCL 的 TF 广播并单独验证漂移风险，不能作为默认导航模式。

### 2.2 导航与自动旋转严格分阶段

自动定位阶段只激活 `map_server` 和 `amcl`；`controller_server`、`planner_server`、`behavior_server`、`bt_navigator`、`waypoint_follower`、`velocity_smoother` 保持 inactive。定位控制器是此阶段唯一允许的运动命令源，退出前必须发送零速、确认 `/odom` 静止并释放命令通道，之后才能激活导航 lifecycle。

### 2.3 不用单一协方差判断成功

AMCL 协方差可能在对称走廊中错误收敛。可信度判定必须同时使用粒子/位姿收敛、激光与地图吻合度、`map -> odom` 稳定度和数据新鲜度，并要求连续时间窗口通过。

## 3. 软件结构

新增一个 Python ROS 2 节点 `automatic_localization_manager`，作为一次性启动状态机；复用现有 `carbot_navigation_real.launch.py`、两个 lifecycle manager、FAST-LIO2、点云投影和速度补偿链。

建议新增或修改：

```text
src/isaac_3d_lidar_bringup/
  isaac_3d_lidar_bringup/automatic_localization_manager.py
  config/nav2/carbot_auto_localization_real.yaml
  launch/carbot_navigation_real.launch.py
  setup.py
  package.xml
  test/test_automatic_localization_manager.py
  test/test_real_nav_safety.py

scripts/
  jetson_navigation_start.sh

docs/
  jetson_tracked_base.md
```

节点接口建议如下：

| 类型 | 名称 | 用途 |
|---|---|---|
| 订阅 | `/fast_lio/imu_odom` | 判断 FAST-LIO2 原始输出连续性 |
| 订阅 | `/odom` | 判断底盘速度、位姿连续性和停车状态 |
| 订阅 | `/scan` | 判断扫描频率并计算地图匹配质量 |
| 订阅 | `/map` | 获取地图边界、自由区与占用栅格 |
| 订阅 | `/amcl_pose` | 获取估计位姿与协方差 |
| 订阅 | `/particle_cloud` | 使用 `nav2_msgs/ParticleCloud` 判断粒子簇是否收敛 |
| 订阅 | `/initialpose` | 识别人工 `2D Pose Estimate` 回退输入 |
| TF | `map -> odom -> base_footprint` | 检查完整 TF、跳变和稳定性 |
| 服务 | `/reinitialize_global_localization` | 触发 AMCL 全局定位；启动时运行时探测实际服务名 |
| 服务 | 两个 lifecycle manager | 分别激活定位和导航 |
| 发布 | `/cmd_vel_command` | 自动定位旋转命令，接入现有补偿/底盘链 |
| 发布 | `/automatic_localization/status` | 状态、原因、质量指标和人工操作提示 |

`/automatic_localization/status` 首版可使用结构化 JSON 字符串；稳定后再考虑专用消息。节点同时输出清楚的 ROS 日志，便于 systemd/Docker 日志直接诊断。

## 4. 启动状态机

```text
WAIT_SENSORS
  -> START_LOCALIZATION
  -> GLOBAL_LOCALIZATION
  -> ROTATE_AND_SCORE
  -> STOP_AND_VERIFY
  -> START_NAVIGATION
  -> READY

定位可信度不足 -> SAFE_STOP -> WAIT_MANUAL_POSE
传感器/TF/lifecycle 故障 -> SAFE_STOP -> FAULT_STOPPED
人工 /initialpose -> VERIFY_MANUAL_POSE -> START_NAVIGATION 或 WAIT_MANUAL_POSE
```

### 4.1 `WAIT_SENSORS`：等待 FAST-LIO2 稳定

建议初始门槛（全部参数化，实车录包后标定）：

- `/fast_lio/imu_odom`、`/odom` 和 `/scan` 均在 0.5 s 内有新数据。
- `odom -> base_footprint` 可解析，时间戳单调，无 NaN/Inf。
- 连续 5 s 内 `/odom` 频率不低于 10 Hz，`/scan` 不低于 5 Hz。
- 开机静止窗口内线速度绝对值不大于 0.02 m/s，角速度绝对值不大于 0.03 rad/s。
- 连续位姿不得出现大于 0.20 m 或 0.35 rad 的单帧跳变。
- 最长等待 90 s；超时进入 `SAFE_STOP`，不激活定位运动或导航。

FAST-LIO2 的 pose covariance 可能不具备可比较含义，因此首版不把它作为唯一稳定条件。

### 4.2 `START_LOCALIZATION`：仅激活地图与 AMCL

- 调用 localization lifecycle manager，确认 `map_server`、`amcl` 均为 active。
- 确认 `/map` 已收到、分辨率与尺寸有效、地图 frame 为 `map`。
- 确认 `/scan` frame 能变换到 `base_footprint`。
- 确认当前没有第二个 `map -> odom` 发布者，也没有导航速度源。

### 4.3 `GLOBAL_LOCALIZATION`：全图撒粒子

- 调用 AMCL 全局重定位服务，将粒子均匀分布到地图自由区。
- 配置全局定位阶段更高的 `max_particles`，建议从 8000 起测；收敛后可由 KLD 采样自动下降。
- 服务调用后等待新的粒子云与 `/amcl_pose`，不得把调用成功等同于定位成功。
- 若 ROS 2 Humble 的目标 Nav2 构建未暴露该服务，则实现兼容层：从地图自由栅格采样粒子并使用 AMCL 支持的接口初始化；开发前先在 Jetson 目标容器确认服务名称和类型。

### 4.4 `ROTATE_AND_SCORE`：安全原地旋转并持续评分

- 旋转前检查急停、底盘命令拓扑、传感器新鲜度和机器人 footprint 周围安全裕量。
- 现有实车证据表明履带可靠启动门槛约为 `0.40 rad/s`，因此默认命令建议 `0.40 rad/s`，而不是一个底盘无法执行的更小数值；经速度补偿与限幅后实测标定。
- 通过 `/odom` 累积实际 yaw，目标为 `2*pi`，不能仅按墙钟估算旋转一圈。
- 最大旋转时间为 35 s，最大累计角度建议 `2*pi + 0.35 rad`；达到任一限制立即停车。实测 `0.40 rad/s` 命令产生约 `0.24 rad/s` yaw，一圈约需 26 s。
- 若提前连续达到可信门槛，可允许提前结束，减少不必要运动；首轮验收阶段建议完整旋转一圈以收集基线。
- 任一时刻发生 `/scan` 或 `/odom` 超时、TF 丢失、位姿突跳、非预期线速度、命令链异常或安全输入触发，立即发布零速并进入 `SAFE_STOP`。

### 4.5 `STOP_AND_VERIFY`：停车后判定可信度

建议初始判据如下，要求连续 3 s 同时成立：

- AMCL 平面标准差：`sqrt(cov_x) <= 0.20 m`、`sqrt(cov_y) <= 0.20 m`。
- AMCL 航向标准差：`sqrt(cov_yaw) <= 0.15 rad`。
- 粒子主簇集中，且不存在权重接近的第二个远距离簇。
- 将 `/scan` 端点变换到 `map` 后，落在占用栅格膨胀邻域内的有效束比例不低于 0.65；全部有效采样束进入分母，包括未知区和地图外端点；另检查已知覆盖率及穿墙冲突。
- 3 s 窗口内 `map -> odom` 平移峰峰值不大于 0.08 m，航向峰峰值不大于 0.08 rad。
- `/odom` 已连续 1 s 满足线速度不大于 0.02 m/s、角速度不大于 0.03 rad/s。

阈值是开发起点，不作为未经实车数据验证的最终常数。必须保存成功、失败和对称场景 rosbag，基于 ROC/误判分析调整。

### 4.6 `START_NAVIGATION` 与 `READY`

- 定位管理器停止发布运动命令，并确认其 publisher 已释放或节点进入不会再发布的终态。
- 再次确认 AMCL 是 `map -> odom` 唯一发布者。
- 激活 navigation lifecycle manager，逐个确认全部 Nav2 节点 active。
- 检查 `/cmd_vel_command -> /cmd_vel` 的发布/订阅拓扑符合安全模式，并确认未发送目标时底盘静默。
- 发布 `READY`，包含最终位姿、协方差、扫描匹配分数、TF 稳定指标和总耗时。

### 4.7 `SAFE_STOP` 与人工回退

- 连续发布零速至少 0.5 s，并以 `/odom` 确认静止；若无法确认，保持故障状态且不得激活导航。
- navigation lifecycle 始终保持 inactive。
- 状态和日志显示具体失败原因，并提示：`自动定位未通过，请在 RViz 点击一次 2D Pose Estimate`。
- 保持 AMCL 与地图 active，订阅新的 `/initialpose`。收到人工输入后禁止再次自动旋转，等待 AMCL 更新并执行同一套停车可信度检查。
- 人工位姿仍不可信时继续停车和提示，不自动放宽阈值，也不自动启动导航。

## 5. 参数与启动入口

在 `carbot_navigation_real.launch.py` 增加参数：

```text
automatic_localization:=true|false
auto_localization_config:=<yaml>
localization_only_timeout_sec:=90.0
rotation_speed_rad_s:=0.40
rotation_target_rad:=6.283185
rotation_timeout_sec:=35.0
```

`scripts/jetson_navigation_start.sh` 增加 `auto` 模式，同时保留当前显式 `X Y YAW` 的固定初始位姿模式作为可靠回退。建议命令形态：

```bash
scripts/jetson_navigation_start.sh MAP_YAML auto navigation-safe
scripts/jetson_navigation_start.sh MAP_YAML fixed X_M Y_M YAW_RAD navigation-safe
```

脚本仍执行容器重建和只读 preflight；自动模式不要求起始坐标。所有自动定位阈值放在 YAML 中，避免硬编码在脚本里。

## 6. 开发阶段

### 阶段 A：接口确认与无运动状态机

- 在 Jetson 目标容器确认 AMCL 全局重定位服务名/类型、粒子云消息类型及 QoS。
- 实现数据新鲜度、lifecycle 顺序、状态输出、超时和零速失败路径。
- 使用 rosbag/模拟消息测试，不允许真实底盘转动。

完成标准：可稳定停在 `ROTATE_AND_SCORE` 前；任何输入缺失都进入安全停车，导航保持 inactive。

### 阶段 B：匹配评分与离线标定

- 实现基于 OccupancyGrid 距离场的 scan-map inlier score。
- 加入协方差、粒子聚类和 TF 窗口稳定度联合判定。
- 用已知正确位姿、错误位姿、对称区域和动态障碍 rosbag 建立阈值基线。

完成标准：离线数据上不把已知错误定位判为 READY，并输出可解释的失败指标。

### 阶段 C：受控原地旋转

- 先架空履带验证命令仲裁、方向、超时和零速退出。
- 再在净空场地、人员可触发急停的条件下，以 `0.40 rad/s` 起测。
- 用实际 `/odom` yaw 关闭一圈，验证左右方向各至少一次。

完成标准：所有正常/异常退出均能停车，定位节点与 Nav2 不同时控制底盘。

### 阶段 D：自动启动与人工回退联调

- 验证成功路径可自动激活导航但不自动发送 Goal。
- 人为制造低质量定位，确认不会激活导航，并能在一次 RViz `2D Pose Estimate` 后重新校验和进入 READY。
- 接入 systemd/Docker 开机流程，验证重启、容器重建和重复启动的幂等性。

完成标准：连续冷启动通过，失败路径无非预期运动，日志足以定位原因。

## 7. 测试与验收矩阵

### 自动化测试

- 单元测试：角度累计、环绕处理、超时、协方差阈值、scan-map score、TF 稳定窗口、状态转换。
- launch 测试：定位先于导航、导航失败不放行、TF 唯一发布者、命令源互斥。
- 故障注入：停止 `/scan`、冻结 `/odom`、TF 跳变、AMCL 服务失败、空地图、重复 `/initialpose`。
- 静态检查：新节点 lint、参数 YAML 解析、现有 `test_real_nav_safety.py` 回归。

### 实车验收

每类至少 10 次冷启动并保存指标：

1. 地图中开阔且特征明显的位置。
2. 靠墙、角落和窄通道。
3. 长直对称走廊，重点验证假收敛拒绝。
4. 地图边缘和局部动态遮挡。
5. FAST-LIO2 延迟启动或短时丢帧。
6. 自动定位失败后人工 `2D Pose Estimate`。
7. 自动定位成功后发送短距离 Nav2 Goal，并确认地图与 `/scan` 持续对齐。

通过条件：

- 错误定位不得进入 READY（安全指标优先于自动成功率）。
- 任意失败路径导航保持 inactive，底盘在限定停车时间内归零。
- 成功路径中 `map -> odom` 只有 AMCL 一个发布者。
- 自动定位成功后不自动发送导航目标。
- 冷启动结果、耗时、最终质量指标和失败原因可追溯。

## 8. 风险与缓解

| 风险 | 缓解措施 |
|---|---|
| 对称环境假收敛 | 联合 scan-map score、粒子多峰检查、TF 稳定窗口；不只看协方差 |
| “低速”低于履带死区，实际不转 | 使用已验证的 `0.40 rad/s` 起点，按实测角速度闭环累计 |
| 自动定位节点与 Nav2 同时发速度 | lifecycle 分阶段、命令源拓扑检查、退出后释放 publisher |
| AMCL 与静态 TF 争抢 `map -> odom` | 始终保持 AMCL 为唯一发布者，测试中检查 authority |
| FAST-LIO2 刚启动即运动导致初始化不稳 | 连续静止与数据质量窗口通过后才允许旋转 |
| 动态障碍降低匹配分数 | 采用有效束下限、鲁棒 inlier 比例和连续窗口，不因单帧失败放行或立即失败 |
| 自动失败后误启动导航 | 所有异常统一进入 `SAFE_STOP`，只有可信度门控可调用 navigation lifecycle |

## 9. 交付物

- 自动定位管理节点、参数文件和 launch 集成。
- 支持 `auto` 与 `fixed` 两种模式的 Jetson 启动脚本。
- 单元、launch、安全回归测试。
- 实车标定 rosbag、阈值记录和验收报告。
- 更新后的开机运行、人工回退、故障诊断文档。

## 10. 2026-09-29 实车联调记录

测试环境：保存地图 `carbot_map_20260928_215841.yaml`，小车位于安全空旷区域；测试过程未发送任何 Nav2 Goal。

- 所有轮次的非运动预检均通过：ESP32、`/wheel_ticks`、`/odom`、MID-360、`/scan`、TF 和命令拓扑正常，FAST-LIO 静止稳定。
- 验证了 `/scan` 和 `/particle_cloud` 必须使用 Best Effort QoS；否则管理节点无法收到真实传感器数据。
- AMCL 全局定位的 8000 粒子云可能增加单线程回调负载，尚需实测确认其与扫描中断的因果关系。管理节点现只保留最新一帧、最多抽样 1000 粒子评分；该小地图的 AMCL 上限调整为 4000。
- 修正了旋转数据中断宽限计时：传感器数据变旧时立即发零速，连续中断超过 2 秒才报错，数据恢复后才继续旋转。
- 成功完成一次自动原地旋转：里程计累计 `6.315 rad`，最终 scan-map score `0.902`，粒子集中度 `1.0`；底盘随后停车并进入质量复核。
- 自动复核在 15 秒内未满足连续质量门槛，正确转入 `WAIT_MANUAL_POSE`，导航保持 inactive。一次人工 `2D Pose Estimate` 也未在 15 秒内通过复核。
- 已增加 AMCL 标准差、证据年龄、TF 平移/偏航窗口跨度和质量保持时间等状态诊断字段。由于电池报警，部署诊断版本后的下一轮在刚完成 arm 时被人工中止，没有继续旋转。
- 每次失败和最终中止后均运行维护的停机程序；最终状态为 Nav2/RViz 已关闭、安全锁已清除、web teleop 已恢复且未使能。

下次续测：

1. 电池充足后运行 `./start_real_nav.sh --automatic`，保存 `STOP_AND_VERIFY` 阶段完整状态 JSON。
2. 根据诊断字段只调整实际失败的门槛（优先检查 AMCL 协方差与 `map→odom` 3 秒窗口），不得直接取消联合质量门控。
3. 自动路径达到 `READY` 后只运行 `./start_real_nav.sh --health-check`；本阶段仍不发送导航 Goal。
4. 单次成功后继续做多位置冷启动、人工回退和故障注入验收，不能以本次测试代替完整验收矩阵。

### 2026-09-30 续测结果与假阳性纠正

- 修复静止速度单帧尖峰反复清空停车确认的问题：保持 `0.02 m/s` 门槛，只有持续超过 0.3 秒才判定仍在运动。
- 将 AMCL 负载限制为最多 3000 粒子、每次 120 条激光束；全局定位仍能收敛，但尚无同步 CPU/扫描测量证明此前中断由计算竞争导致。
- 近障保护改为首次近点立即发零速，连续 0.3 秒仍小于 0.55 m 才锁定故障，过滤单帧反射噪声但不延迟停车。
- 修正 AMCL 证据新鲜度与最终验证时长的参数矛盾，并修复 TF 滑动窗口删除边界样本导致“3 秒窗口永远不成立”的逻辑错误；增加对应单元测试。
- 状态机曾以旋转 `6.308 rad`、AMCL XY 标准差 `0.011 m`、偏航标准差 `0.006 rad`、旧版 scan-map score `0.720`、粒子集中度 `1.0` 和稳定 TF 进入 `READY`，Nav2 健康检查也通过且未发送 Goal。
- **该结果随后被操作员通过 RViz 明确判定为错误全局位置，因此是严重的假阳性，不是成功验收。** 截图显示激光点大量穿墙/落在错误结构，机器人图标也不在真实位置。发现后立即用维护程序关闭 Nav2/RViz 并清除安全锁。
- 已确认的评分漏洞（尚无本次原始扫描重放，不能断言为本事件完整根因）：旧版 scan-map score 只用“落在地图已知区域的端点”作分母，地图外和未知区光束被忽略；少量偶然命墙的光束可产生虚高分。同时只检查端点，没有拒绝射线提前穿墙。
- 修复后所有有限量程采样束都进入评分分母，未知区/地图外不再免费忽略；增加激光射线穿越已占用墙体的拒绝规则、已知地图覆盖率门槛及专项回归测试。
- 在新的错误定位拒绝测试和人工目视对齐确认完成前，不得把自动 `READY` 视为实车验收成功，也不得用其发送导航 Goal。
