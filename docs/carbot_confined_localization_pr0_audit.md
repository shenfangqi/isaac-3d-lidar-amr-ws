# Issue #13 PR0：只读审计与契约

对应 `docs/carbot_confined_localization_implementation.md` 第 11 节 PR0。代码基线 `2a51cd0`。本 PR 不改 manager 行为、不改 launch/YAML、不新增速度发布者，所有脚本只读。

## 交付物

| 文件 | 内容 |
| --- | --- |
| `src/isaac_3d_lidar_bringup/isaac_3d_lidar_bringup/localization_contracts.py` | v1 数据结构（SE2、Keyframe、Hypothesis、SearchResult、RotationDecision、MotionProfile）、拒绝原因枚举与中文文案、motion_request/motion_status 严格 JSON 编解码、profile 校验、策略组合与参数校验。纯 Python，无 ROS 依赖 |
| `scripts/audit_mid360_localization_visibility.py` | 近场覆盖审计：无 bag 时做解析几何覆盖；给 bag 时逐帧统计实际类型、过滤比例、源时间年龄、源时间 TF 失败率和扫掠环 UNKNOWN/OBSERVED_OCCUPIED/OBSERVED_FREE |
| `scripts/build_localization_baseline_manifest.py` | 只读 `metadata.yaml` 生成基线 bag manifest（话题/类型/计数/哈希/可用于哪类审计） |
| `docs/evidence/issue13_pr0_baseline_bag_manifest.json` | 77 个本地 bag 的 manifest（含 payload sha256） |
| `docs/evidence/issue13_pr0_geometry_audit.{json,md}` | 当前参数下的解析覆盖结论 |
| `src/isaac_3d_lidar_bringup/test/test_localization_contracts.py`、`tests/test_localization_visibility_audit.py` | 契约与审计纯逻辑测试 |

## 1. 输入与控制源清单（按仓库代码核对）

| 话题 / 接口 | 类型 | 生产者（仓库可见） | 关键配置 | 状态 |
| --- | --- | --- | --- | --- |
| `/livox/lidar` | **不一致**，见第 3 节 | `livox_ros_driver2`（`carbot-mid360.service` → `mid360_stack.launch.py`） | launch `xfer_format: 1`（CustomMsg）；`MID360_config.md` 写 `xfer_format=0`；canonical 参数写 PointCloud2 | 待实机核对 |
| `/mid360/imu/data_raw` | `sensor_msgs/Imu` | `mid360_imu_adapter` | 时间修正 −9.782937 ms | 已记录 |
| `/fast_lio/cloud_registered_body` | `PointCloud2`，frame `imu_link` | FAST-LIO（`lidar_type: 1`=AVIA，订阅 CustomMsg） | `blind: 0.5`（3D 距传感器）、`point_filter_num: 3`、`det_range: 20` | 已记录 |
| `/fast_lio/imu_odom` | `nav_msgs/Odometry` | FAST-LIO（`tf_en: false`） | 估计器健康，不作独立真值 | 已记录 |
| `/odom` + TF `odom→base_footprint` | `nav_msgs/Odometry` | `fast_lio_base_adapter`（唯一动态 TF 权威） | 轮速计 systemd 用 `wheel_odometry_fused.yaml`：发 `/wheel/odom`、`publish_tf: false`，不竞争 | 已记录；preflight 要求 `/odom` 单发布者 |
| `/scan` | `LaserScan`，frame `base_footprint` | `pointcloud_to_laserscan`（输入 cloud_registered_body） | 高度 0.10–0.35 m，`range_min 0.5`，`use_inf: true` | 已记录 |
| `/scan_localization` | `LaserScan`，frame `base_footprint` | 同上 | 高度 0.22–0.35 m，`range_min 0.5` | 已记录 |
| `/map` | `OccupancyGrid`，transient-local | `map_server` | 地图 yaml/pgm 在证据目录有 sha256 | 已记录 |
| TF `map→odom` | — | 自动模式：AMCL；手动模式：`manual_map_localizer` | 本 Issue 不新增 map→odom 发布器 | 已记录 |
| 静态 TF | — | robot_state_publisher（`carbot-description.service`），URDF 来自 canonical 参数 | 雷达原点离地 0.209 m，相对 base_link xy (0.0166, −0.0001) | 已记录 |
| `/cmd_vel_command` | `Twist` | 定位阶段：manager（`start` 时才创建，START_NAVIGATION 销毁）；Nav2 激活后：velocity_smoother（`cmd_vel_smoothed` remap 到同一话题） | preflight 要求 Nav2 激活前 0 发布者、1 订阅者 | 已记录 |
| `/cmd_vel` | `Twist` | `cmd_vel_compensator`（右转 ×0.896），systemd `carbot-command-compensation.service` | 订阅者为 micro-ROS ESP32 | 已记录 |
| 底盘 watchdog | — | ESP32 固件（不在仓库） | canonical `cmd_vel_timeout_s: 0.50` | 待实机核对：实际超时与清零行为 |
| `/localization/emergency_stop` | `std_msgs/Bool`，RELIABLE + TRANSIENT_LOCAL | **仓库内找不到发布者** | 订阅方：`jetson_navigation_control.py`、`jetson_navigation_wait.py`、`jetson_ros_freshness_check.py`、`carbot_bounded_ground_pulse.py` | 待实机核对：发布节点与锁存语义 |
| `/localization/fault_reason` | `std_msgs/String`，同上 QoS | 同上 | — | 待实机核对 |
| `/carbot/status` | `carbot_msgs/CarbotStatus` | ESP32（micro-ROS） | 字段含 `motion_blocked`、`active_command_source`、`agent_connected`、`time_synchronized`、`invalid_cmd_count`、`last_disconnect_reason` | 已记录；manager 目前未订阅 |
| `/automatic_localization/status` | `std_msgs/String` JSON，transient-local | manager | 新字段见 `STATUS_EXTENSION_FIELDS` | 已记录 |

### 控制源发现

1. **compensator 锁存与仓库代码不符。** `start_real_robot_navigation_rviz.sh::clear_command_compensation_latch` 说明 compensator 会锁存定位急停，需要重启服务才能清除。但仓库里的 `carbot_hardware/cmd_vel_compensator.py` 只做右转比例补偿，没有订阅急停，也没有锁存。Jetson 实际运行的是 `/home/shenfq/Projects/carbot-ros2` 里的构建，和本仓库可能不同步。PR3 的 guard 依赖这条链路，所以合入 PR3 之前必须在实机上确认实际运行的 compensator 版本和锁存行为。
2. **定位阶段与 Nav2 共用 `/cmd_vel_command`。** 当前靠两点保证互斥：manager 先销毁自己的 publisher，Nav2 才激活；preflight 也检查发布者数量。新设计里 guard 必须沿用同样的时序（spec C07）。
3. manager 目前不订阅 `/localization/emergency_stop` 和 `/carbot/status`。spec 4.1 要求新状态在急停或锁止时禁止运动，PR3 需要接入这两个话题，而且要使用实机核实过的类型和 QoS。

## 2. 近场覆盖解析结论（无运动、无 bag）

`python3 scripts/audit_mid360_localization_visibility.py --output docs/evidence/issue13_pr0_geometry_audit.json`

输入均来自仓库现有参数：footprint、碰撞高度带 0.01–0.24 m、雷达原点离地 0.209 m、厂商垂直 FOV −7°～52°、最小测距 0.1 m、FAST-LIO `blind 0.5`、两路 scan 的 `range_min 0.5`。扫掠环取车体边缘（0.13 m）到原地一圈外包络加 0.05 m padding（0.254 m）。

| 通道 | 扫掠环内可观测高度 | 能否证明扫掠自由 |
| --- | --- | --- |
| `/scan`、`/scan_localization` | 无（range_min 0.5 m 和 blind 0.5 m 覆盖整个扫掠环） | 否：全部 UNKNOWN |
| `/fast_lio/cloud_registered_body` | 无（blind 0.5 m） | 否 |
| 原始 MID-360 | 约 0.18–0.20 m 以上；下视 −7° 看不到更低的高度 | 否：碰撞带 74–80% 不可观测 |

这里还没有计入车体对近场光束的自遮挡，实际覆盖只会更少。

**结论：仅靠传感器，第 6 节的旋转安全门对任何旋转都只能给出 `UNKNOWN_SWEEP`。** 不能为了放行去调低全局 `range_min`，或扩大 self mask。

**运行约定（2026-10-05 与用户确认）：操作者只会把小车放在确定能原地旋转的位置。** 这个承诺作为传感器盲区的证据，契约里的落实方式如下：

- `RotationAttestation(session, odom_pose, issued_mono, max_translation_m, max_age_s)` 只在本次会话内有效。会话不同、相对确认时的 odom 位置平移超过上限（被搬动或打滑）、超时或时钟倒退，都会使 `attestation_covers` 返回 False。原地转向不会让它失效。
- 它只覆盖**传感器看不到的**扫掠格子，单独计入 `RotationDecision.attested_cells`。传感器实际看到的障碍仍然判 `OBSTACLE_IN_SWEEP`；profile、预算、停稳确认、急停这些检查照常执行。
- 必须在启动时显式给出（`operator_rotation_clear`，只在 `segmented_rotation + guarded` 下合法，默认关闭），并写入状态和报告，方便追溯。不跨会话保存，也不是长期豁免开关，与 spec 第 1 节一致。
- 剩余风险由操作者看护承担：确认之后才进入扫掠区的动态障碍（人、宠物），在 0.18 m 以下 MID-360 看不到。PR3/PR4 的实车验收必须在现场看护、可急停的条件下进行。

## 3. 原始点云类型不一致（需实机核对）

| 来源 | `/livox/lidar` 类型 |
| --- | --- |
| `fast_lio_mid360.yaml`（`lidar_type: 1`，注释称依赖 CustomMsg 的逐点时间偏移） | `livox_ros_driver2/msg/CustomMsg` |
| `mid360_stack.launch.py`（`xfer_format: 1`） | CustomMsg |
| `MID360_config.md` 文字 | `xfer_format=0`（PointCloud2） |
| `carbot_parameters.yaml` `pointcloud_type` | `sensor_msgs/msg/PointCloud2` |
| 2026-09-21 静态 bag 实录 | `sensor_msgs/msg/PointCloud2` |

如果实机发布的是 PointCloud2，FAST-LIO 的 AVIA 订阅者就收不到数据。现在 FAST-LIO 实际在运行，说明当前实机很可能已经切到 CustomMsg，那么 09-21 的 bag 就不代表当前驱动输出。审计脚本两种类型都能处理，并在报告里写出 `raw_lidar_type_matches_fast_lio`。下次采集时，需要在实机上记录 `ros2 topic info /livox/lidar -v`，并更新文档和 canonical 参数中错误的那一方。

## 4. 基线 bag 覆盖与缺口

`python3 scripts/build_localization_baseline_manifest.py --hash-storage --output docs/evidence/issue13_pr0_baseline_bag_manifest.json`

| 审计用途 | 可用 bag 数 | 说明 |
| --- | --- | --- |
| 静止搜索回放（scan_localization+odom+map+TF） | 9 | 2026-09-30 自动定位证据 |
| 派生 scan 近场审计 | 10 | 同上 + postsearch_static_01 |
| 原始近场审计（/livox/lidar + tf_static） | 5 | 2026-09-21 静态 bag，类型为 PointCloud2（见第 3 节） |
| FAST-LIO 体坐标点云近场审计 | **0** | 没有 bag 录过 `/fast_lio/cloud_registered_body` |
| 旋转响应分析（命令链+odom） | 18 | 09-22/09-23 标定 |
| 控制源审计（命令链+急停+底盘状态） | **0** | 没有 bag 同时录这三类话题 |

缺少的两类数据，下次授权的**静止**采集可以一次补齐，不需要让车运动。建议录这些话题：`/livox/lidar`、`/fast_lio/cloud_registered_body`、`/scan`、`/scan_localization`、`/tf`、`/tf_static`、`/odom`、`/localization/emergency_stop`、`/localization/fault_reason`、`/carbot/status`、`/cmd_vel_command`、`/cmd_vel`，同时保存 `ros2 topic info -v` 的输出。采集时在扫掠环内外放软障碍（例如纸箱），用来验证解析结论。录完在容器内运行：

```bash
python3 scripts/audit_mid360_localization_visibility.py <bag_dir> --output <证据目录>/visibility_audit.json
```

## 5. 契约决定（需确认）

- **`motion_policy` 只管新策略。** spec 同时规定"legacy_full_rotation 是开发期默认"、"forbid 是默认"，以及"forbid 始终无非零输出"。三条同时成立的唯一读法是：legacy 沿用原有旋转和互锁，`motion_policy` 对它不生效。`legacy + guarded` 和 `stationary_only + guarded` 都视为参数冲突，启动时报错；`segmented_rotation + guarded` 必须提供 profile 路径。
- **motion_status 增加了 `schema_version` 字段。** spec 列出的 status 字段里没有它。加上之后，消费方能识别并拒绝不兼容的 guard 版本。
- **摆放确认**：见第 2 节，`RotationAttestation` / `attestation_covers` / `RotationDecision.attested_cells` / `validate_strategy_configuration(..., operator_rotation_clear)`。
- **Profile 状态分四级：INSUFFICIENT / ESTIMATED / REVIEWED / ACCEPTED。** 自动分析最高只能产出 ESTIMATED。REVIEWED 和 ACCEPTED 要求有外部复核记录、证据 id，且测量值不能为 null。只有 ACCEPTED 且三个 hash 都匹配时，才允许非零输出。

## 6. 验证

- `ros2-dev-humble` 容器：`colcon build --packages-select isaac_3d_lidar_bringup` 成功，两个新测试文件 67 项全部通过（用户执行，加入摆放确认之前的版本）。
- 加入摆放确认后：宿主机契约 63 项、审计 12 项通过，容器内尚需重跑；`test_flake8`/`test_pep257` 也在容器内执行。
- bag 统计模式需要 `rosbag2_py`，还没有在容器里对真实 bag 跑过。
- 全程没有运动命令，没有改动 manager、launch 或 YAML。
