# Issue #13 / #53 / #54 交接文档（2026-10-13）

接着 `docs/issue13_handoff_2026-10-12.md`。那份里的用户要求、定位流程、关键配置和操作方法仍然有效，这里只写新增的内容。

## 0. 先做：按顺序合并 3 个 PR

这次会话里，自动权限拦下了合并（理由是"未经审核合并"），所以 3 个 PR 都还开着。它们是叠在一起的，请按顺序合并：

1. **#63** 共用的局部子图模块（base main）
2. **#64** #54 自动地图覆盖层（base #63 的分支）
3. **#65** #53 多停车点子图的三维判断（base #64 的分支）

合并 #63 后，GitHub 可能需要把 #64 的 base 改成 main（`gh pr edit 64 --base main`），#65 同理。全部只是离线代码，不改任何节点、launch 和配置，**不需要重新部署 Jetson**。Jetson 仍然运行 `8228856`。

## 1. 这次做了什么（全程离线，Jetson 关机）

| | 内容 | 证据 |
|---|---|---|
| #63 | `localization_submap.py`：多帧拼接、去掉临时物体、和地图比对（多出来的 / 光束穿过 / 少掉的），按物体分组；`scripts/check_submap_changes.py` | `docs/evidence/issue13_local_submap_2026-10-13.md` |
| #64 | `localization_map_overlay.py`：带版本、可回退的覆盖层和一组更新约束；`scripts/replay_map_overlay.py`、`scripts/check_overlay_wrong_pose.py` | `docs/evidence/issue54_map_overlay_offline_2026-10-13.md` |
| #65 | `localization_submap_decision.py`：多停车点子图交给 `decide_surface`，穿透检查按每个点的停车点画光线；`scripts/check_submap_decision.py` | `docs/evidence/issue53_submap_decision_offline_2026-10-13.md` |

主要结果：

- 子图比对在 10-12 的 14 份录包上自动找出了床、床沿横梁下的假表面、拉上的窗帘、关着的门，每份约 3 s。
- 覆盖层：从零开始自动学到的结果覆盖了人工床补丁 97% 的位置；按时间顺序回放 14 个会话，0 次接受错误位置；28 次"错误位姿被接受"的对抗更新全部被拒，写入 0 个格子。
- 子图判断：合成的对称场景里，单个视角分不开，加一个停车点就分开了；改动前后静止定位的生产决策输出逐项相同。
- 包测试：561 passed, 1 skipped（#65 分支，含 flake8/pep257）。另外修了一个过时的测试（`test_issue13_startup.py` 还在找 #61 之前的写法）。

## 2. 下一步（都需要实车）

### 2.1 #53：直线运动标定和运动数据（操作员在场）

1. 照 2026-10-06 旋转 profile 的做法，标定平移速度、停止尾程、侧滑和 watchdog 截停距离，得到 ACCEPTED 的 linear profile。没有它，平移守卫一直禁止运动。
2. 录分段移动的数据：每段停稳至少 6 s，录包要有 `/fast_lio/cloud_registered_body /odom /tf /tf_static`。**以前的旋转探测录包都没有录点云**，所以至今没有一份车动过的点云。顺便原地转一圈也录点云。
3. 离线分析：`scripts/check_submap_changes.py BAG --pose X Y YAW --reference last ...`（看拼接是否对齐，多出来的比例应和静止时相近），`scripts/check_submap_decision.py BAG --truth ...`（多停车点的三维证据）。
4. 然后再接入管理器的流程。

### 2.2 #54：家里的受控实验

1. 先把覆盖层接入运行流程：READY 后用录下的点云建子图 → `observe` → `update_overlay` → 自检 → 保存新版本，再把 `surface_additions` 指向新版本。这一步可以离线写，但在线自检的耗时要在 Jetson 上测。
2. 受控实验：每次只改一处（挪椅子约 1 m、挪家具、开关窗帘、东西挪回原位）、人员走动；操作员确认真值。
3. 注意：r12_15door 用了学到的覆盖层后分差从 0.228 降到 0.203（仍接受）。实车上要看每次写入后，后续会话的分差有没有变化。

### 2.3 次要

- Jetson 开机时钟跳变：先在 Jetson 上确认用的是哪种校时（timesyncd 还是 chrony）、哪些服务是用户级哪些是系统级，再决定怎么让它们排在校时之后。仓库里的服务文件在 `src/carbot_hardware/systemd/`、`src/carbot_description/systemd/`。
- 10-12 交接文档 3.4 的其他几项不变。

## 3. 操作和工具补充

- 新工具都在 `ros2-dev-humble` 容器里、在 `/workspace/ros-humble/isaac_3d_lidar_amr_ws` 下运行；输出文件已存在时都会报错退出，不会覆盖。
- 工作站上的实测耗时：子图比对每份录包约 3 s；覆盖层回放 14 个会话约 7 分钟（每个会话 18–54 s，含正例和反例两次决策）；14 份样本的生产决策评估约 5 分钟（10-12 实测）。对抗检查没有单独计时。
- 结果目录（不进 git，容器写的文件归 root 所有）：`calibration_data/submap_checks/`、`calibration_data/overlay_checks/`。
- 工作站重启后容器会停，先 `docker start ros2-dev-humble`。

## 4. 这次的教训

- **自检挡不住错误的第一名**：覆盖层是用这个第一名自己的观测生成的，只会让它更吻合。第一版约束在对抗检查里每份样本都写进了几百个格子。真正起作用的是会话层面的检查（整次会话多出来的比例）和多视点要求。约束要用对抗检查验证，不能只看正常流程。
- **反例检查要剔除所有来源的真值候选**：`decide_surface` 自带三维全图搜索，会把真值重新加回候选；只去掉二维候选的话，反例实际上又成了正例。要先单独搜索，再剔除，和 `evaluate_surface_decisions.py --search3d union` 一样。
- 等待某个进程结束时，用 `pgrep -f '[n]ame'` 这种写法，否则 `pgrep` 会匹配到自己的命令行，循环永远不会结束（上一个会话留下的等待循环空转了 8 个多小时）。
- 合成测试里的可见性要和几何尺度匹配：离墙 1 m 时 0.01 rad 的分箱太细，墙后的点会从缝隙"漏"过来。
