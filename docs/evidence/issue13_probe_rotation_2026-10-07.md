# Issue #13 probe rotation on the real robot (2026-10-07)

Supervised (operator at the robot, area clear, user-confirmed placement attestation). Launch: `--automatic --localization-strategy segmented_rotation --localization-motion guarded --motion-profile docs/evidence/issue13_motion_profile_accepted_2026-10-06.json --operator-rotation-clear --operator-present`; Nav2 never activated.

## Natural ambiguity: not found

Two placements, four runs (`calibration_data/2026-10-07_issue13_probe_rotation_01..04`): every run localized stationary (scores up to 0.90) and never entered PLAN_PROBE; the operator confirmed the poses. Run 3 was torn down by the launcher after a lost `/automatic_localization/start` response (fixed: the launcher now checks the manager state). Zero motion in all four runs.

## Forced probe (`--force-probe-once`, validation-only test mode)

| run | result | cause / fix |
| --- | --- | --- |
| forced_probe_01 | guard started, halted after 0.1 s with SENSOR_STALE; one nonzero command, wheels moved 1-2 ticks | backward odometry wobble made target - progress exceed the covered angle (and pi/2) → sweep remaining capped at the target, `sweep_tolerance_rad` 0.02 |
| forced_probe_02 | rotated to -56.4 deg, halted with ODOM_JUMP | base origin circling the rotation centre moved 17 mm vs the 15.4 mm 60-deg profile value → `CENTER_DRIFT_MARGIN` 2.0 on both guard limit and sweep padding |
| forced_probe_03 | **full chain**: PLAN → EXECUTE (1.484 rad) → SETTLE → second view → CANDIDATE_READY (2.652, 4.134, -33.8 deg), guard RELEASED; operator confirmed the pose | four scans failed the sweep TF lookup ("extrapolation into the future") → guard waits up to 0.15 s for the scan-time TF |
| forced_probe_04 | **full chain, no warnings**: CANDIDATE_READY (2.668, 4.153, 52.6 deg), guard RELEASED, `/cmd_vel_command` without publisher | — |

forced_probe_04 motion (FAST-LIO odometry): target 90 deg; nonzero commands for 6.50 s, at most 0.40 rad/s; 81.6 deg at the last nonzero command; final 85.3 deg (3.7 deg after zero, inside the predicted stop angle of about 7.5 deg); largest base translation 2.3 cm (limit 3.1 cm). Wheel ticks -2103/+2229.

Every halt above was a guard decision and zeroed the output in the same cycle; none needed the e-stop.
