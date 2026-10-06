# ESP32 command watchdog: supervised ground test (2026-10-06)

Chain: `/cmd_vel_command` -> `cmd_vel_compensator` -> `/cmd_vel` -> ESP32 (the Issue #13 motion-guard chain; Nav2 inactive). Robot on the floor in a cleared area with an operator watching. Script: `scripts/jetson_rotation_profile_capture.py --mode watchdog --operator-present --speed 0.40 --trials 4 --hold 1.5 --safety-net 1.5`.

Each trial commanded an in-place turn for 1.5 s, then stopped publishing entirely (no zero command). "Stop" = odometry stationary and wheel ticks unchanged for 0.2 s; the time is that of the last tick change. A zero command would have been sent after 1.5 s; it was never needed.

| trial | direction | measured rate at cut-off (rad/s) | stop after last command (s) | extra rotation (deg) | result |
| --- | --- | --- | --- | --- | --- |
| 0 | left | 0.200 | 0.492 | 8.42 | pass |
| 1 | right | 0.260 | 0.546 | 8.74 | pass |
| 2 | left | 0.244 | 0.566 | 8.59 | pass |
| 3 | right | 0.241 | 0.566 | 8.92 | pass |

The chassis stops 0.49-0.57 s after the last command, consistent with the canonical `cmd_vel_timeout_s: 0.50` plus braking. If the guard process dies mid-probe, the robot therefore continues for about 0.15 rad (8.4-8.9 deg) before stopping; the guard's sweep check covers the probe target plus its braking extension, not this crash case, so placement attestation and on-site supervision remain required.

Evidence: `calibration_data/2026-10-06_issue13_esp32_watchdog_02` (bag; gitignored). An earlier attempt (`..._watchdog_01`) aborted in preflight because the bag recorder also subscribed to `/cmd_vel_command`; the robot did not move.
