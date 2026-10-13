# Issue #53：实车第一次带平移的定位（2026-10-13 晚）

用 2026-10-13 验收的直线标定（`docs/evidence/issue53_linear_profile_accepted_2026-10-13.json`），在实车上跑通了"原地定位 → 安全前进一小段 → 在新位置重新判断"的完整流程。操作员在场。

## 为什么要用强制测试模式

离线用实测的运动参数跑管理器的路线规划（`scripts/analyze_localization_routes.py`，停车外延 0.184 m、横向 1.2 cm、航向 4°），所有有记录的歧义位置都不会选平移：
- 加上三维网格各层后，原地就能分开（DISTINGUISHABLE_NOW）；
- 只用二维扫描时，窄的地方又没有对所有候选都安全的路线（NO_COMMON_SAFE_ACTION）。

所以在这个家里，平移不会被自然触发。为此加了只用于验证的 `--force-translation-once`（PR #69）：静止定位通过后，仍然走一次对所有合理候选都安全的前进段，然后用新视角重新判断，Nav2 不激活。

## 启动参数

```bash
CARBOT_INITIAL_POSE_TIMEOUT=900 scripts/start_real_robot_navigation_rviz.sh --automatic \
  --localization-strategy segmented_rotation --localization-motion guarded \
  --motion-profile docs/evidence/issue13_motion_profile_accepted_2026-10-06.json --operator-present \
  --localization-translation guarded \
  --linear-profile docs/evidence/issue53_linear_profile_accepted_2026-10-13.json \
  --force-translation-once --surface-recheck record
```

没有 `--operator-rotation-clear`，所以管理器不能原地旋转，车唯一可能的动作就是这一段前进。三维判断只记录、不参与决定。

## 三次运行

| 次 | Jetson 版本 | 结果 | 原因和修复 |
|---|---|---|---|
| 1 | `60cd5e9`（PR #69） | 起步前被守卫拒绝（`SWEEP_TOO_SHORT`），车没动 | 管理器按自己的里程计算出剩余 0.599 m，守卫按自己的起点要 0.6 m。修复（PR #70）：证据覆盖两边估计里较大的剩余距离，再加 0.02 m |
| 2 | `05222bf`（PR #70） | 起步前被守卫拒绝（`SWEEP_STALE`），车没动 | 证据指向守卫还没收到的那一帧里程计，或者 MOVE 之后第一份证据还没到，守卫就锁定了故障。修复（PR #71）：守卫保留最近几份证据，最新一份说了算（拒绝或过期立即停车）；只有它指向还没收到的帧时，才用之前的证据顶替，一份可用的都没有就发零速度等待（最多 0.5 s） |
| 3 | `c639988`（PR #71、#72） | **成功** | — |

PR #70 还修了代码审查时发现的另一个问题：守卫原来拿证据和**最新一帧**里程计比，容差 5 mm，可是 0.06 m/s、10 Hz 下一帧就差 6 mm，车一动就会误判。现在按证据里的时间戳找对应的那一帧来比，并要求那一帧最多比最新帧旧 0.3 s。

强制测试请求的是 0.4 m（PR #70）：守卫会提前一个停车外延（0.184 m）开始停，实际走约 0.22 m。如果请求 0.6 m，车大约在 0.42 m 处才停，前进右偏造成的横向偏离会碰到标定的上限（0.41 m 内 1.2 cm），守卫会误判偏离路线。

离线复现（PR #72）：把真实的管理器和守卫放在一起跑，里程计带 1 mm 抖动，每条消息有 0–60 ms 的随机延迟，两边节拍错开。修复前的守卫 16/16 以 `SWEEP_STALE` 失败，和第 2 次实车一样；修复后 16/16 都走完。

## 第 3 次的经过（管理器日志）

| 时间（s） | 状态 |
|---|---|
| 0 | START_LOCALIZATION → COLLECT_STATIC → SEARCH_MULTI_VIEW |
| 60.9 | VERIFY_HYPOTHESES：静止结果通过；`force_translation_once: FORCED_FORWARD, distance 0.4, candidates 1` |
| 63.9 | EXECUTE_TRANSLATION |
| 68.3 | SETTLE_TRANSLATION（行驶约 4.4 s） |
| 69.8 | COLLECT_STATIC（新视角）→ SEARCH_MULTI_VIEW |
| 83.8 | VERIFY_HYPOTHESES → STOP_AND_VERIFY |
| 93.6 | **CANDIDATE_READY** |

- 最终位姿 (2.765, 2.792, 87.8°)，AMCL 1.9 cm / 0.9°，二维打分 0.82、覆盖率 0.97。守卫 RELEASED，Nav2 没有激活。
- 操作员确认：车前进了**约 20 cm**，RViz 箭头的位置和朝向都和车一致。

## 局限

- 这次只有 1 个合理候选，路线证据只需要对一个位姿检查；多个候选同时检查的情况只在单元测试和离线模拟里验证过。
- 平移后的判断用的是两个视角的二维和 AMCL，不是 #65 的多停车点三维子图判断，那部分还没接进管理器。
- 请求 0.6 m 时，横向偏离会碰到标定上限，这个问题还没解决。正常规划路线也可能选 0.6 m。可以考虑按实际行驶距离设横向偏离的上限，或者给直行加修正。
- 这次没有录包（为了不拖慢 FAST-LIO）。证据来自启动日志和管理器日志，以及操作员的目视确认。
