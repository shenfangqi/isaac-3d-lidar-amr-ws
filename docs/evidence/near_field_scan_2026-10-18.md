# 近场雷达通道 `/scan_near`（2026-10-18）

只用工作站离线验证，没有部署，实车上默认关闭。

## 1. 问题

2026-10-17 夜，实车绕障测试（`detour_repro_02`）中，车贴着一个手提包蹭了约 15 秒，最后压上去，车身被顶起 17°，定位检查锁车。

原因：局部代价地图用的 `/scan` 是由 FAST-LIO 的点云投影的，而 FAST-LIO 丢弃 0.5 m 以内的点（`blind 0.5`），`/scan` 本身又设了 `range_min 0.5`。障碍一进入 0.5 m 就从局部代价地图里消失了。原始 MID-360 点云在这 15 秒里一直在车头前 0.24–0.27 m 处看得到手提包。

项目文档规定不全局调低 `/scan` 的 `range_min`；近场要走专用通道，并先验证原始数据和物理覆盖。

## 2. 原始数据检查

车静止在空旷处（`static_check_01`、`detour_repro_01`）时，在 `/scan` 的高度带（离地 0.10–0.35 m）内：

- 0.1–0.4 m 没有任何点，车身不会被扫到；
- 0.4–0.5 m 只有两侧的墙；
- 0–0.1 m 的点都在原点上，是 Livox 用来填"没有回波"的零点。

MID-360 安装高度约 0.21 m，下视只有 7°：0.3 m 处离地约 16 cm 以下的物体，这个通道看不到，需要 ToF。

## 3. 实现

- 节点 `near_field_scan`：
  - 输入：原始 `/livox/lidar`。直接从 CDR 字节解析点，与 rclpy 反序列化逐点一致；每帧 0.11 ms，rclpy 方式要 40 ms。
  - 输出：`/scan_near`，360 个方位格。
  - 标记：0.1–0.6 m、离地 0.10–0.35 m、车身轮廓外扩 5 cm 以外的最近点。
  - 清除：只有当仰角在 -8°～13° 的光束打到 0.6 m 以外时，才报告这个方位"近场无障碍"（0.65 m，只清除不标记）。
  - 其余方位输出 NaN，不标记也不清除。
- `carbot_near_field_real.yaml`：局部代价地图的 `obstacle_layer` 增加观测源 `scan_near`；`/scan` 的设置不变。
- 开关：launch 参数 `near_field_scan`（默认 `false`）；启动脚本加 `--near-field`（容器脚本第 19 个参数）。普通和有界恢复两种控制器都会叠加这个配置。
- 订阅默认 best-effort：`/livox/lidar` 也是 FAST-LIO 的输入，可靠订阅以前曾把 FAST-LIO 拖停。

## 4. 验证

单元测试：包内全套 698 passed, 1 skipped。

离线（实车 bag）：
- 撞包前 16 秒，`/scan_near` 每帧都在车头前 0.24–0.27 m 处标出障碍。按 RPP 1 s 碰撞推算，车会在接触前约 15 秒就判定前方会碰撞。
- 正常行驶约 4500 帧里，只触发过 1 次，经查是真实物体。

端到端（bag → 节点 → Nav2 ObstacleLayer）：车头前出现致命格。本地回放时 best-effort 只收到约 59% 的帧（约 6 Hz），可靠订阅为 94%；实车上的实际帧率还没测。

`controller_server` 用实车参数文件加载（普通 / 叠加有界恢复）：`observation_sources` 为 `scan scan_near`；`/scan` 的 topic 和 `observation_persistence 2.0` 不变；`scan_near` 的范围为 0.6 / 0.65 m。

两个观测源同时工作（独立代价地图，配置与实车相同，合成数据：`/scan` 10 Hz 打到 2 m 外，`/scan_near` 6 Hz）：

| 阶段 | 纸箱位置的致命格 |
|---|---|
| 纸箱在，`/scan` 的光束从它上面穿过 | 2（持续，不闪烁） |
| 纸箱拿走，`/scan_near` 看到 0.6 m 以外 | 0 |
| 纸箱放回 | 2 |
| `/scan_near` 没有信息（NaN） | 0（`/scan` 照旧清掉，与现状相同） |

## 5. 限制和待做

- 实车物理覆盖验证（文档要求，启用前必须做）：在车周围不同距离、高度放置物体，确认能标记、拿走后能清除。
- 实车上 `/scan_near` 的实际帧率，以及对 FAST-LIO 的影响（CPU、点云订阅）。
- 0.3 m 处离地约 16 cm 以下、或落到最低光束以下的物体，这个通道看不到，`/scan` 仍会清掉它们，要靠 ToF。ToF 以后作为第三个观测源加入，清除规则届时统一设计。

数据：`output/issue1_robot/bags/`（未提交）；离线脚本在 `lio_replay_ws/near_field_offline.py`、`near_field_rpp_check.py` 和 `near_e2e/`（仓库外）。
