# Issue #13 交接文档（2026-10-12 晚）

给下一个会话用：现在的状态、下一步做什么，以及继续工作必须知道的信息。细节见文中链接。

## 0. 先读这些

- 项目规定（`../AGENTS.md`）：开工前先完整读两个技能入口：`skills/isaac-amr-project-state/SKILL.md` 和 `skills/ros-docker-debug/SKILL.md`。动 Jetson 之前读 `skills/isaac-amr-project-state/references/jetson-target.md`。
- 当前状态以 `skills/isaac-amr-project-state/references/current-state.md` 的最新检查点为准，它指向本文。
- 今天的证据：`docs/evidence/issue13_robot_decide_and_bed_2026-10-12.md`。

## 1. 用户的要求（必须遵守）

- **所有回复都用中文**，包括短的进度说明。今天多次在长时间处理英文工具输出后写成英文，用户非常不满。
- **全自动定位**：不要提议用 RViz 的 2D Pose Estimate 人工定位，也不要把它当成修复或验证手段。让操作员看箭头确认真值可以，这只是标注。
- 每完成一步（提交、合并、部署、测试等），立刻用中文简短汇报：做了什么、确认了什么、还剩什么。
- **连 Jetson 一律用 IP 192.168.1.109**：SSH 别名 `isaac-jetson` 已指向这个 IP，并且核对原主机指纹。不要用 `ubuntu.local`，它时好时坏。不要保存 Jetson 密码。
- 需要实车时提前告诉用户。用户要外出或睡觉时，会说明是否需要 poweroff 工作站，按当次的说明做。
- 合并：用户今天授权"验证全部通过就合并"。PR 用 merge commit。部署用 `scripts/deploy_bringup_to_jetson.sh`：本机 `.claude/settings.local.json` 已允许直接运行（预览、`--check`、`--apply`）。
- 报时间要用实测值，不要凭印象估。不要因为一点小改动就把整批验证全部重跑。

## 2. 当前版本和状态

- **main = `819b553`**。今天合并了 PR #44–#60。
- **Jetson 运行 `8228856`**（PR #57）。之后的 #58–#60 只改了文档，不需要重新部署。
- 当前地图：`carbot_map_20260928_215841`。网格 SHA256 是 `9eb7d035…`，Jetson 上的 `maps/real/*.ply` 与它一致。
- 实车今晚 5 次 decide 全部一次成功：床边 107 s、床尾 121 s、右上 83 s、中上（x 2.2 门关着）56 s、床东侧（两扇门都关着）114 s，AMCL 误差约 2 cm。每次的位置都由操作员确认。

### 定位流程（stationary_only，车不动）

1. 采集静止扫描。
2. 二维全图搜索。
3. 二维验证。
4. 二维拒绝时交给三维判断，拒绝原因包括 AMBIGUOUS、NO_VALID_CANDIDATE、UNOBSERVABLE_AXIS（PR #52）、预算受限的 SEARCH_INCOMPLETE。三维判断依次做：全图三维搜索 → 用三维搜索剔除不可能的二维种子（PR #56）→ 剔除后再检查候选数上限（PR #57）→ 公平精化 → 穿透排除 → 近邻竞争 → 支撑范围 → 两个时间窗一致 → 二维合理性检查。
5. 交给 AMCL，交接前最多等 5 s，等 `/odom` 新鲜、车确认静止（PR #55）。
6. AMCL 复核后进入 CANDIDATE_READY；activate 模式再进入 READY。

### 关键配置（`src/isaac_3d_lidar_bringup/config/localization/`）

`carbot_map_20260928_215841_variable_regions.yaml`：易变区域，绑定网格 SHA256。

| 区域 | 用途 | 2D |
|---|---|---|
| bottom-wall curtains | 底墙窗帘（只框窗帘面） | — |
| top door (x 2.2) | 门 | — |
| top door (x 0.94) | 门 | — |
| bottom door (bedroom-living, x 1.9) | 门 | — |
| bed south edge under the rail | 床沿横梁下，扫描能穿过 | `mask_2d: true` |

- 实车证据：x 2.2 门关着时，有区域的分差是 0.431，没有是 0.337；下部门关着时，0.228 对 0.170。

`carbot_map_20260928_215841_surface_additions.npz`：床的地图补丁，5386 个点。它**只参与拟合打分，不当实体**（不进穿透检查）。

- `mask_2d: true`：在二维定位打分（`scan_map_metrics`）里，这块区域"可穿过"。光束落在这里仍算命中，穿过这里不算穿墙冲突。导航地图、运动安全、路线规划和地图哈希都不受影响。扫描高度段 `LOCALIZATION_SCAN_BAND = (0.22, 0.35)` 定义在 launch 里，管理器参数和它一致。
- 不采用的区域：中部门 x 2.0（帮错误位置通过了反例；平时开着），右下窗帘（窗帘开着时少接受 19 个视角，合成场景看不到收益）。

## 3. 下一步任务（按顺序）

### 3.1 共用的局部子图模块（离线，下一项）

#53 和 #54 都要"多帧拼成局部子图，再和全局地图比对"，所以先做共用的部分。两个 issue 的评论里写了这个安排。

- **子图拼接**：按里程计（FAST-LIO）把多帧点云放进同一个坐标系，去掉运动畸变，滤掉临时出现的东西（人），压成体素。
- **和地图比对**：每个点离地图表面多远；哪里多了（added）、哪里少了（removed）、哪里光束穿过了地图（see-through）。`scripts/check_scene_consistency.py` 是它的单帧简化版。
- **先在位置已知的情况下验证**：用今天这批带真值的录包，要能自动复现今天手动找出的问题——床（多出来的）、床沿横梁下的假表面（光束穿过）、窗帘、门。

### 3.2 #54 地图持续更新（离线，然后上实车）

issue #54 里写了路线和顺序：在固定地图上加自动覆盖层，暂不换 SLAM。

1. 只用静止定位成功的会话做变化检测：READY、三维分差足够、AMCL 复核通过。
2. 覆盖层自动管理：多次会话都出现才写入；带版本，可以回退；每次只改一小块；写入后做秒级在线自检；出问题自动退回。回退时要分清失败原因：位置本来分不开不算地图问题。
3. 实车受控实验：挪椅子、挪家具、开关窗帘、人员走动。
4. 再处理"东西被拿走"。床沿下的假表面就是这一类，现在靠手动 `mask_2d` 区域处理。

### 3.3 #53 移动中定位

- 已有（PR #41，默认禁用）：平移守卫、平移协议、路线规划、管理器里的流程框架、仿真闭环。
- 没做：子图比对定位（离线）；**直线运动标定和验收（需要实车，操作员在场）**；实车验证。

### 3.4 次要（有机会时）

- `carbot-description` 掉线：今天上午出现两次，下午 7 次循环没复现。启动脚本现在发现静态坐标变换缺失时，会自动重启这个服务并重试一次（PR #55）。
- Jetson 开机时时钟会跳：系统服务在校时前就启动了。可以考虑让它们排在 time-sync 之后。
- 全图合成扫描里，加床补丁后右上 (4.3–4.6, 6.2–6.5) 少接受了 7 个视角。实车两次都正常（分差 0.243、0.261），继续留意。
- 床东侧 (0.94, 0.77) 的二维吻合度是 0.683，离 0.65 门槛很近。

## 4. 操作方法

### 实车采样（车不动）

```bash
# decide 模式：定位 → RViz 箭头（绿色三维、蓝色二维）→ 录 90 s 静止数据
SURFACE_RECHECK=decide bash scripts/record_localization_sample.sh NAME /path/to/outdir
# 停止（导航、RViz、清除安全锁、网页遥控恢复为未解锁）
bash scripts/stop_real_robot_navigation_rviz.sh
```

- 录包保存在 Jetson 的 `/home/shenfq/Projects/isaac_ros-dev/calibration_data/<日期>_issue13_sample_NAME`，用 `scp -r isaac-jetson:...` 拷回 `calibration_data/2026-10-12_issue13/`。要等 Jetson 上 `/tmp/sample_NAME_bag.log` 出现 "Recording stopped" 才能挪车。
- 引导操作员摆车：在 Jetson 上用 `systemd-run --user --unit=issue13-pose-markers --setenv=ROS_DOMAIN_ID=0 bash -lc "source /opt/ros/humble/setup.bash; exec python3 /tmp/publish_pose_markers.py X,Y,YAW_DEG,1,0.5,0,TARGET"` 发一个箭头（先 `scp scripts/publish_pose_markers.py isaac-jetson:/tmp/`）。Jetson 关机时，用 `bash scripts/open_region_editor.sh` 在工作站本地打开 RViz，再在容器 `carbot-region-editor` 里以 ROS_DOMAIN_ID=78 发箭头。
- 真值文件格式：`calibration_data/2026-10-09_localization_diagnosis/r12_*_ground_truth.json`。

### 离线评估（在 `ros2-dev-humble` 容器里，目录 `/workspace/ros-humble/isaac_3d_lidar_amr_ws`）

- 诊断：`scripts/diagnose_localization_bag.py BAG --map MAP.yaml [--stationary-capture] --variable-regions REGIONS --mesh PLY --output OUT`。
  - **它不会覆盖已有的输出文件**：会报错 "output already exists" 然后跳过。今天因此有两轮验证用了旧结果，因为我把报错过滤掉了。重跑前先删掉旧文件，并且保留它的报错输出。
  - 录包里有完整定位会话（时间线里有 WAIT_FOR_START）就不加 `--stationary-capture`；出结果后补录的录包要加。
- 样本评估：`scripts/evaluate_surface_decisions.py --samples LIST.json ... --search3d union --variable-regions R --surface-additions A`。比较剔除效果时加 `--no-prune`。
- 多时间窗重复：`scripts/evaluate_window_repeats.py --max-pairs 20 ...`。
- 合成扫描：`scripts/analyze_localizability_3d.py ... --variable-regions R --surface-additions A [--no-ceiling --clutter 0.2]`。
- 易变区域：画框用 `scripts/open_region_editor.sh`；检查用 `scripts/check_variable_regions.py quick|full`。
- 地图补丁：`scripts/build_surface_additions.py`。
- 完整决策链回放：`scripts/replay_localization_node_chain.sh BAG decide OUTDIR START`。用环境变量 `EXTRA_PARAMS` 传 `surface_variable_regions:` 和 `surface_additions:`。回放前先 `colcon build --packages-select isaac_3d_lidar_bringup`，否则用的是旧代码。
- 样本清单：`calibration_data/2026-10-09_localization_diagnosis/surface_samples_2026-10-12_masked.json`（14 份，已用屏蔽后的二维地图重新诊断）。
- 实测耗时（工作站）：样本 13–14 份约 5 分钟；多时间窗重复约 15 分钟；全图退化合成约 55–60 分钟；决策链回放 6 份约 10 分钟。

### 测试

- 在 `src/isaac_3d_lidar_bringup` 目录：`python3 -m pytest -q -p no:cacheprovider test/`。当前 538 个通过、1 个跳过。
- pep257 要求 docstring 用祈使句开头，多行 docstring 的摘要从第二行开始。

## 5. 今天的教训

- **实验和实现必须一致**：二维屏蔽实验时只屏蔽了床沿一条，实现时却扩大到所有区域，结果把窗帘和门后面的墙也屏蔽了。
- **"未知"和"可穿过"不一样**：把床沿当未知，会丢掉床腿、横梁上的真实命中。
- 加新功能前，要找出前面已有的限制：这次的候选数上限挡在了剔除之前。
- 床这类低矮家具，雷达装得低、建图时离得远就扫不到。建图注意事项已写进 `docs/map_gen/README.md`。
- 不要把工具的报错过滤掉。
- 用户的问题要先确认理解对了再答：床的障碍不是用户手工标的，是 nvblox 按车高自动生成的。
