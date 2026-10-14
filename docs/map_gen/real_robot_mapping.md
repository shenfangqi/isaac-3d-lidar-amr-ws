# 实车建新地图（Carbot + MID-360 + Jetson）

本文是在**新的环境**给实车建一张导航地图的完整步骤：建图、保存、用新地图自动定位和导航。仿真里的建图看 `docs/map_gen/README.md`；那篇的原理（nvblox 2.5D 地图、点云补齐、`free_thresh 0.196`）同样适用于实车。

2026-10-14 在实车上逐步验证过本文的建图、保存、加载流程（车没开动，只做了静止建图和测试保存；正式建图还没按本文完整跑过）。

## 0. 先知道的事

- **建图链路**：MID-360 → FAST-LIO2（位姿，`/odom`）→ nvblox（2.5D 占用地图 + 三维网格）。**没有 SLAM Toolbox，也没有闭环**：FAST-LIO2 的漂移会直接进地图，所以建图路线不要太长、要回到起点附近闭合，开得慢一些。
- **坐标**：地图建在这次建图会话的 `odom` 坐标系里，原点就是建图开始时车的位置。导航时这个坐标系就是 `map`。
- **一次建图产生的文件**（Jetson `~/Projects/isaac_ros-dev/maps/real/` 下）：
  - `NAME.pgm` / `NAME.yaml`：二维导航地图（Nav2 和二维定位用）；
  - `NAME.ply`：三维表面网格（三维定位用，**必须和 pgm 来自同一次建图**）；
  - `NAME.nvblx`：nvblox 原生地图（备份）。
- **不要覆盖现有地图**。现在在用的是 `carbot_map_20260928_215841`。保存脚本拒绝覆盖同名文件；新地图请用新名字。
- **Jetson 开关机由操作员自己做**，agent 不要关机、重启 Jetson。连接只用 SSH 别名 `isaac-jetson`（192.168.1.109），不要用 `ubuntu.local`。

## 1. 建图前准备（现场）

来自 2026-10-12 的实车经验（`docs/map_gen/README.md` 第 11 节）：

- **路线经过矮家具附近 0.6–1.5 m**：床、沙发、矮柜、桌子。MID-360 装得低，向上视角有限，离远了扫不到悬空的水平结构；0.5 m 以内的点被丢掉（`lidar_min_valid_range_m=0.5`），太近也建不进去。上一张图就因此漏了卧室的床，后来只能用地图补丁补。
- **把经常挪动的东西挪开或固定**：椅子、小桌、垃圾桶。建进地图的东西一旦挪动，三维定位会看到"光束穿过了地图里的东西"，可能把正确位置排除。
- **建图时人不要在场**；门、窗帘保持平时的状态。
- 电量充足（建图中掉电会丢掉整张图）。
- 操作员全程在场，随时能拿起车或断电（这台车没有标准物理急停）。

## 2. 启动建图

工作站先停掉导航（如果在跑）：

```bash
cd ~/projects/ros-humble/isaac_3d_lidar_amr_ws
./stop_real_nav.sh
```

在 Jetson 上启动建图容器（会先等系统时间同步，然后启动 FAST-LIO2 + nvblox）：

```bash
ssh isaac-jetson '~/Projects/isaac_ros-dev/scripts/jetson_nvblox_container.sh recreate mapping'
```

工作站打开建图用的 RViz（低带宽模式，Fixed Frame `odom`）：

```bash
scripts/real_robot_mapping_rviz.sh
```

### 启动后检查（都满足再开车）

```bash
ssh isaac-jetson 'docker exec carbot-nvblox bash -lc "source /opt/ros/humble/setup.bash; source /workspaces/isaac_ros-dev/install/setup.bash; \
  for t in /livox/lidar /fast_lio/imu_odom /odom /nvblox_node/static_occupancy_grid; do printf \"%s \" \$t; timeout 6 ros2 topic hz \$t | grep average | tail -1; done; \
  ros2 param get --no-daemon /nvblox_node lidar_min_valid_range_m"'
```

2026-10-14 的正常值：`/livox/lidar` 约 7–10 Hz、`/fast_lio/imu_odom` 和 `/odom` 约 10 Hz、`/nvblox_node/static_occupancy_grid` 约 6 Hz，`lidar_min_valid_range_m = 0.5`。车静止时地图约 144 × 136 格、0.05 m，同时有空地、障碍和未知格。RViz 里能看到蓝色体素和二维地图。

- `carbot-mid360-ptp.service` 是**系统**服务，用 `systemctl is-active carbot-mid360-ptp.service` 查（不要加 `--user`）。
- 容器内 `ros2 node list` / `ros2 service list` 每次只能发现一部分节点，**不能用它判断节点不存在**；看话题频率和参数即可。

## 3. 开车覆盖（遥控）

用手机或工作站浏览器打开网页遥控：`http://192.168.1.109:8080/`（2026-10-14 实测可访问）。它启动时是**未解锁**状态；确认车周围安全后在页面上解锁，再低速开车。它只往 `/cmd_vel_command` 发指令，经过速度补偿节点到底盘；浏览器指令有 300 ms 租约，松手即停。**不要同时启动 Nav2 或其他遥控。**

路线要求：

- 低速；直线和转弯都慢。原地转向在 0.40–0.50 rad/s 才能可靠启动（更低会卡在死区）。
- 往返路线有重叠；在角落、被遮挡处停几秒，让 nvblox 融合多帧。
- 经过所有矮家具附近（见第 1 节），也经过门口两侧。
- 最后**回到起点附近**，让开头和结尾看到同一处墙，方便检查漂移。
- 不要贴墙高速掠过；不要钻到床下或很窄的缝里。

开车过程中盯着 RViz：墙应当是直的、不重影。如果出现明显重影或墙错开，说明 FAST-LIO2 漂移了，停下来，重新建图（`recreate mapping` 会从零开始）。

## 4. 保存前验收

- 外墙闭合，房间、门洞、家具轮廓和现场一致；
- 自由区域里没有大块未知空洞，地面没有密集的噪点；
- 开头和结尾经过的同一面墙重合（没有重影）；
- 车停稳（网页遥控松手，至少几秒）。

## 5. 保存

在 Jetson 上运行（名字只能用字母、数字、`_ . -`，不能和已有地图重名）：

```bash
ssh isaac-jetson '~/Projects/isaac_ros-dev/scripts/jetson_save_maps.sh NAME'
```

脚本依次保存 `.nvblx`、`.ply`、`.pgm/.yaml`（二维图由 `map_saver_cli` 从 `/nvblox_node/static_occupancy_grid` 导出，`trinary`、`free_thresh 0.196`、`occupied_thresh 0.65`），检查文件都不为空、阈值正确，最后打印用新地图导航的命令。2026-10-14 实测：一次成功，`.nvblx` 是 SQLite，`.ply` 是 ASCII 网格，`.pgm` 尺寸和实时地图一致。

保存后打开 `.pgm` 看一眼（可以 `scp` 到工作站），确认和 RViz 里看到的一致。

停止建图：

```bash
scripts/stop_real_robot_mapping_rviz.sh
ssh isaac-jetson '~/Projects/isaac_ros-dev/scripts/jetson_nvblox_container.sh stop'
```

## 6. 用新地图自动定位和导航

```bash
./start_auto_nav.sh /home/shenfq/Projects/isaac_ros-dev/maps/real/NAME.yaml
```

- 三维定位自动使用同目录同名的 `NAME.ply`；第一次启动会生成三维缓存 `NAME.surface-*.npz`（约 1 分钟）。
- 终端显示 `READY` 后，在 RViz 里用 "2D Goal Pose" 导航。先发 1–2 m 的短目标。
- 不加地图参数时，`start_auto_nav.sh` 仍然用旧地图 `carbot_map_20260928_215841`。要把新地图设为默认，设置环境变量 `CARBOT_NAV_MAP`，或改 `scripts/start_real_robot_navigation_rviz.sh` 里的 `default_map`（改之前和用户确认）。

## 7. 新地图的定位验收（建议）

旧地图上有的这些配套，新地图都要重新做；不做也能用，但定位会差一些：

- **易变区域**（会开关的门、窗帘）：`scripts/open_region_editor.sh` 画框，`scripts/check_variable_regions.py quick|full` 检查，保存为 `src/isaac_3d_lidar_bringup/config/localization/NAME_variable_regions.yaml`（绑定新网格的 SHA-256，旧地图的不能用）。
- **地图补丁**（建图漏掉的矮家具等）：`scripts/build_surface_additions.py`，保存为 `NAME_surface_additions.npz`。
- **随意放置测试**：车放在几个不同位置（每个房间、不同朝向），用 `./start_auto_nav.sh NAME.yaml` 看能否自动定位、RViz 箭头和实际是否一致。失败时看终端的原因，不要用 RViz 的 "2D Pose Estimate" 手动给位姿。

## 常见问题

| 现象 | 处理 |
|---|---|
| `jetson_save_maps.sh` 说容器不在 mapping 模式 | 先 `jetson_nvblox_container.sh recreate mapping` |
| 保存时提示拒绝覆盖 | 换一个新名字；不要删除或覆盖已有地图 |
| 墙出现重影 | FAST-LIO2 漂移；放慢、缩短路线，重新建图 |
| 新地图启动后二维候选都不通过 | 地图覆盖不够（只在原地扫过）或环境和建图时差别大；补开一圈重新建图 |
| `ros2 node list` 看不到 nvblox 节点 | 容器内 CLI 发现不完整，看话题频率；不要据此重启 |
| 网页遥控打不开 | `systemctl --user status carbot-web-teleop` |
