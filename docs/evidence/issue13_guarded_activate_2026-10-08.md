# Issue #13 guarded activation on the real robot (2026-10-08)

Supervised (operator at the robot, about 0.3 m clear around it, user-confirmed placement attestation). Jetson on main `6ae6821`. Launch:

```
CARBOT_INITIAL_POSE_TIMEOUT=320 ./start_real_nav.sh --automatic-activate \
  --localization-strategy segmented_rotation --localization-motion guarded \
  --motion-profile docs/evidence/issue13_motion_profile_accepted_2026-10-06.json \
  --operator-rotation-clear --operator-present
```

No goal was sent. Placement: the same spot as `forced_probe_04` (2026-10-07).

## Result

- Manager states: WAIT_SENSORS → START_LOCALIZATION → COLLECT_STATIC → SEARCH_MULTI_VIEW → VERIFY_HYPOTHESES → STOP_AND_VERIFY → START_NAVIGATION → READY. The pose was not ambiguous, so PLAN_PROBE was never entered and no rotation happened. This is the fifth run at these placements without natural ambiguity.
- Pose `(2.670, 4.163, 52.3 deg)`. Search: score 0.925, coverage 0.99, wall-conflict ratio 0.058, no runner-up. AMCL: std 2.0 cm/0.9 deg. Drift after the search: 8 mm/0.1 deg.
- All eight Nav2 lifecycle nodes active. `navigate_to_pose`, `spin` and `backup` ready. Launcher: `NAVIGATION_HEALTHY goal_sent=false`, `READY`.
- After activation, `/cmd_vel_command` had exactly one publisher (`velocity_smoother`). The guard reported `RELEASED`, `stopped=true`, `abs_travel_rad 0.0`.

## No-motion evidence

A 420 s monitor (`jetson_nav_command_monitor.py --skip-upstream`) covered localization and activation:

| signal | samples | result |
| --- | --- | --- |
| `/cmd_vel` | 992 | 0 nonzero, max linear/angular 0 |
| wheel ticks | 20 989 | left/right delta 0/0 |
| `/odom` | 2 424 | displacement 12.9 mm (FAST-LIO stationary drift; wheels did not move) |

Bag: `calibration_data/2026-10-08_issue13_activate_probe_01` (23 MB). The recorder was stopped by `timeout -s INT`, so systemd recorded exit 124; the bag closed normally.

Then `stop_real_robot_navigation_rviz.sh` brought Nav2/RViz down, cleared the latch and restored web teleop disarmed.

## Still open

The probe rotation was shown only in `--automatic` with `--force-probe-once` (2026-10-07). An activation that rotates naturally needs a placement with real ambiguity.
