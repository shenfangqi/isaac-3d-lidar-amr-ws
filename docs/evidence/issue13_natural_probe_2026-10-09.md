# Issue #13 natural probe rotation on the real robot (2026-10-08/09)

All runs used the same launch:

```
./start_real_nav.sh --automatic-activate --localization-strategy segmented_rotation \
  --localization-motion guarded \
  --motion-profile docs/evidence/issue13_motion_profile_accepted_2026-10-06.json \
  --operator-rotation-clear --operator-present
```

Conditions for every run:

- An operator was at the robot.
- The area within about 0.3 m of the robot, including the body's sweep, was clear; the user confirmed this before each run.
- No goal was sent.
- A 420-480 s `jetson_nav_command_monitor.py --skip-upstream` monitor and a bag recorder ran from before the launch.

After each run, `stop_real_robot_navigation_rviz.sh` restored web teleop disarmed. Bags are under `calibration_data/`.

| run | main | placement | result |
| --- | --- | --- | --- |
| 2026-10-08_issue13_activate_probe_02 | 6ae6821 | B | READY without a probe |
| 2026-10-08_issue13_activate_probe_03 | 6ae6821 | C | 2 natural probes, then FAULT_STOPPED "probe rotation did not settle" → fix #34 |
| 2026-10-08_issue13_activate_probe_04 | 82688e3 | C | 2 probes settled; session timeout during the third map-wide search → fix #35 |
| 2026-10-09_issue13_activate_probe_05 | f600613 | C | aborted before localization (SSH connection reset), no motion |
| 2026-10-09_issue13_activate_probe_06 | f600613 | C | **4 natural probes, then READY; the operator confirmed the pose** |

## probe_02: no ambiguity at placement B

- Manager states went straight from VERIFY_HYPOTHESES to STOP_AND_VERIFY and READY.
- Pose `(2.881, 6.296, 19.6 deg)`. Search: score 0.875, coverage 0.99, wall conflict 0.075, 8 hypotheses, no runner-up after validation. AMCL 1.8 cm/0.9 deg; drift after the search 2 mm.
- Monitor: 0 of 939 `/cmd_vel` samples nonzero; wheel ticks 0/0; `/odom` drifted 10.8 mm.

## probe_03: the settle check trusted a biased twist

- First search SEARCH_INCOMPLETE → probe of -90 deg, swept 83.3 deg, settled in 0.8 s.
- Second search SEARCH_INCOMPLETE again → second probe, 82.3 deg (165.6 deg in total).
- The guard then stayed STOPPING. After the 5 s settle timeout the manager failed with `probe rotation did not settle` (FAULT_STOPPED).
- Cause: after the second turn, the FAST-LIO twist read 0.02-0.04 m/s (median 0.027) for 10 s, while the position moved 2.5 mm net (0.0002 m/s). The 0.02 m/s stationarity gate used that twist. After the first turn it read 0.008 m/s.
- Fix #34: translation speed now comes from the change in odometry position over 0.5 s, in both the guard and the manager.
- Bag replay of the failed settle: 7.5 s with the twist, 0.6 s with positions. The other settles were unchanged.
- Monitor: 248 of 2991 `/cmd_vel` samples nonzero (the two probes), max 0.358 rad/s; wheel ticks +4297/-4275.

## probe_04: map-wide re-search per view exceeded the session

- The two probes (83.1 and 83.4 deg) settled in 0.7 s each, confirming #34.
- After every probe the manager re-ran the map-wide search over all views: 42 s, then 84 s, then still running at 96 s.
- At 246 s it hit the 240 s session limit: `SEARCH_INCOMPLETE (session timeout)` → SAFE_STOP → WAIT_MANUAL_POSE.
- Fix #35 (spec 5.3): after a probe, a complete earlier search is re-checked with every TRAIN view instead.
  - An incomplete search still needs a new map-wide search.
  - If the re-check refutes every candidate, a map-wide search must complete before anything is accepted.
- Offline replay of this bag on the workstation, using `/scan` in place of `/scan_localization`:

| step | map-wide search | re-check |
| --- | --- | --- |
| 2 views | 14.9 s | 1.9 s |
| 3 views | 22.5 s | 2.5 s |

  Both kept the same leading candidates.
- Monitor: 243 of 7162 `/cmd_vel` samples nonzero, max 0.358 rad/s; wheel ticks +4340/-4236.

## probe_05: transport failure, no motion

At launcher step 7, SSH to the Jetson was reset (`kex_exchange_identification: Connection reset by peer`) and the launcher cleaned up. The bag shows the manager only in WAIT_FOR_START, no `/cmd_vel` message, and constant wheel ticks.

## probe_06: natural probes resolve the placement

| time (s) | step |
| --- | --- |
| 6 → 47 | map-wide search, SEARCH_INCOMPLETE → probe 1 |
| 55 → 137 | map-wide search (previous search incomplete), 82 s, AMBIGUOUS_LOCATION → probe 2 |
| 145 → 163 | re-check of 8 hypotheses, 17 s, ambiguous → probe 3 |
| 171 → 191 | re-check of 7 hypotheses, 19 s, ambiguous → probe 4 |
| 197 → 221 | re-check, 24 s, accepted |
| 222 → 233 | STOP_AND_VERIFY → START_NAVIGATION → READY |

| probe | request | swept | zero to stopped |
| --- | --- | --- | --- |
| 1 | -90 deg | 83.9 deg | 0.8 s |
| 2 | -90 deg | 86.1 deg | 0.7 s |
| 3 | -90 deg | 83.9 deg | 0.7 s |
| 4 | -60 deg | 54.8 deg | 0.7 s |

- Total travel 5.388 rad (309 deg). Every segment commanded zero before its target; no guard halt and no e-stop.
- READY at `(-0.768, 0.095, -93.4 deg)`. All eight Nav2 lifecycle nodes active.
- The guard was RELEASED and `velocity_smoother` was the only `/cmd_vel_command` publisher.
- The operator confirmed pose, heading and scan alignment in RViz.
- Monitor: 438 of 4321 `/cmd_vel` samples nonzero (the four probes), max 0.358 rad/s, no linear command; wheel ticks +7736/-7932; `/odom` 25.1 mm (base origin circling the rotation centre).
- This bag also records `/scan_localization`, so it can be replayed exactly.

## Open

- **Session margin.** probe_06 finished at 233 s of the 240 s session. The first search was incomplete, so the second view needed a full map-wide search (82 s).
- **Number of probes.** It is bounded by the guard budget (6 segments), not by the hypothesis-count rule; the count fell from 8 to 7, so that rule never fired. Operators should be told the budget, not "two segments".
