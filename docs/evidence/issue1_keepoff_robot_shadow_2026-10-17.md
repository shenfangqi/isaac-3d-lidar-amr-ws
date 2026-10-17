# Issue #1：禁区节点实车旁路观察（2026-10-17 晚）

第一次在实车上运行新障碍禁区链路。**车全程没有动**，Nav2 没有激活；禁区只输出到话题，没有接入 Nav2。操作员在现场。

## 1. 方法

- 启动：`./start_real_nav.sh --automatic --localization-strategy stationary_only`。结果 CANDIDATE_READY，位姿 (1.19, 3.47, -0.1°)，扫描得分 0.925，AMCL 2.1 cm / 1.0°，`validation_only`。
- 旁路节点：在 Jetson 的 `carbot-nvblox` 容器里作为独立进程运行，代码放在临时目录，用 `PYTHONPATH` 优先加载，已部署的包没有改动。结束后临时目录已删除。
  - `static_map_scan_filter`（仿真 launch 里的参数：`/scan` → `/scan_global_static_filtered`，余量 0.12 m）；
  - 逐光束 5 帧中值：代替 `laser_filters`，因为镜像里没装这个包，行为与仿真的 `MultiChannelMedianFilterFloat` 相同；
  - `novel_obstacle_keepoff`（参数文件 `carbot_obstacle_detour_real.yaml`）。先用 main 6b0c007 的代码，发现第 2 节的问题后，两个节点都换成本 PR 的修复版；第 3、4 节都是修复后的结果。
- 实车 `/scan` 的角度从 -π 到 +π、每 1° 一束，首尾方向重合，没有仿真那种接缝；频率约 9.8 Hz。

## 2. 发现并修复：查 TF 时阻塞执行器，导致扫描大量丢弃

第一次启动时，两个节点都在持续报 TF 外推错误。禁区节点手里的 `map → base_footprint` 落后了 3–11 s，CPU 占用却只有约 7%，100 帧都没处理完；静态过滤节点只输出约 6 Hz。

原因：实车上 `/scan` 和 `odom → base_footprint` 的时间戳完全相同，两者同时到达（相对当前时间的延迟都是 0.04–0.23 s），扫描回调常常先执行。节点用带超时的 `lookup_transform` 阻塞等待，但 TF 监听和扫描回调共用同一个执行器，等待期间根本收不到 TF，只能超时丢帧，积压越来越多。`TransformListener(spin_thread=True)` 在 Humble 的 rclpy 里也没有用（实测）。仿真里 TF 一般比扫描先到，所以一直没暴露。

修复（本 PR）：新增 `tf_wait_queue.py`。回调里查不到 TF 时不阻塞，先把扫描放进队列；每来一帧、以及一个 20 Hz 定时器都会去取队列，TF 到了就按到达顺序处理，等超过 `transform_wait_sec`（0.5 s）就丢掉并计数。静态过滤节点和禁区节点都改用它；禁区节点的标记和清除扫描共用一个队列，保持先后顺序。参数 `transform_timeout_sec` 改为 `transform_wait_sec`。

修复后（实车，约 12 分钟）：

- `/scan_global_static_filtered` 约 8.8 Hz；禁区栅格 2 Hz。
- 禁区节点处理了约 7200 帧标记扫描和约 7200 帧清除扫描（10 Hz）。启动后因 TF 超过 0.5 s 未到而丢弃约 96 帧（不到 1%），静态过滤约 31 帧（约 0.5%）。修复前绝大部分帧都被丢弃。
- Jetson 上的耗时（每 100 次调用统计一次，共 72 个窗口）：
  - 清除（含 #112 的插值）：中位数 43–48 ms；最差一个窗口的 p95 为 84 ms；单次最大 591 ms，只有一次，当时我在同一台机器上跑分析脚本。
  - 标记：中位数 0.7–0.8 ms，单次最大 58 ms。
  - 离线基准：不插值的清除每帧 10 ms，插值后 55 ms。清除在 10 Hz 下大约占掉一半的单帧时间，正式接入时要留意。

## 3. 静止观察：没有误报

车不动，每 30 s 统计一次不在静态地图里的禁区格，共 6 次（406–988 格）。逐一标在 RViz 上（借用 `/overhead_clearance_markers` 显示），由操作员到现场确认：

| 标记 | 位置（地图） | 相对车 | 现场是什么 |
|---|---|---|---|
| C | x[-2.0, -0.5]，y[2.6, 3.9] | 后方 1.5–3 m，靠墙 | 最近放的、地图上没有的东西 |
| A | (3.7, 3.1) | 右前方约 2.6 m | 操作员的小手提包 |
| B | (2.9, 3.5) | 正前方约 1.7 m | 操作员放的纸箱（清除验证用） |
| D | (0.2, 4.6) | 左后方约 1.5 m | 操作员本人，坐在电脑旁 |

每一处都是地图上没有的真实物体，没有误报。实车上也没有看到"跟着车走"的假回波，支持 #111 去掉假回波过滤的前提（只验证了静止的情况）。

这些点到地图上最近的墙都有 0.18–0.47 m，超出了静态过滤 12 cm 的余量，所以都被当作新障碍。贴墙放的东西也会被标出来。

## 4. 清除验证（实车）

纸箱 B 放在车前约 1.7 m，确认被标出（20 s 内有 26 次回波）。拿走后，B 处在 10 s 内没有任何回波，禁区里也没有任何一块连通区域留在纸箱位置。剩下的块分别属于 C、A（包还在）和 D。

## 5. 没有验证的

- 车在移动时的情况（假回波、转向门控、视角变化）；
- 禁区接入 Nav2、真正的绕行。停车余量（第①条）仍在讨论中，绕行仍只用于仿真；
- 实车 launch 里还没有这两个过滤节点（launch 只在 `runtime == "sim"` 时启动它们），镜像里也没有 `laser_filters`。

## 6. 数据

`output/issue1_robot/`（未提交）：节点日志、`still_*.json` 禁区快照和渲染图、`after_box_removed.json`。

## 7. 后续：接入实车 launch（观察用，默认关闭）

- `carbot_navigation_real.launch.py` 新增 `novel_obstacle_keepoff_preview`（默认 `false`）。打开后启动 `static_map_scan_filter`、`scan_temporal_median`（节点名 `global_scan_filter`）和 `novel_obstacle_keepoff`。实车 Nav2 配置不读 `/novel_obstacle_keepoff`，规划、控制和速度都不变。
- 用法：`./start_real_nav.sh ... --keepoff-preview`（或 `./start_auto_nav.sh --keepoff-preview`）。启动脚本把它作为第 18 个参数传给 `jetson_nvblox_container.sh`。
- `scan_temporal_median` 代替镜像里没有的 `laser_filters`。在仿真里与 `laser_filters` 吃同一路输入，逐帧比对 293 帧（包括车在行驶的时候），输出完全相同。
- 还没有部署到 Jetson：下次实车会话先用 `scripts/deploy_bringup_to_jetson.sh` 部署，再用 `--keepoff-preview` 验证。
