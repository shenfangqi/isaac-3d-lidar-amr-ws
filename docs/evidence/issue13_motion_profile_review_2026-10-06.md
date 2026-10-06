# Rotation motion profile: review and acceptance record (2026-10-06)

Profile: `docs/evidence/issue13_motion_profile_accepted_2026-10-06.json` (status **ACCEPTED**, `externally_reviewed: true`).
Request binding (`profile_hash`, sha256 of the canonical encoding): `0c6bd95daf34efc612c034bc55bee47255e8e2323540c80c6345f1cbeae5c5a4`.

## Decisions

- **REVIEWED** (external cross-check): the operator watched the supervised turns on 2026-10-06 (eight alternating 60 deg turns at a 0.40 rad/s command and four watchdog cut-offs) and confirmed on site that the motion, the stops and the post-cut rotation showed no obvious anomaly. Recorded as evidence id `review:2026-10-06-operator-visual`. This is a visual check only: it confirms the absence of obvious abnormality, not centimetre or degree accuracy.
- **ACCEPTED**: explicit decision by the user in the session on 2026-10-06 ("reviewed, accepted"). Analysis code never sets this status.

## Values (worst valid sample, FAST-LIO odometry)

| field | value |
| --- | --- |
| latency_s (stop latency) | 0.171 |
| stop_tail_rad | 0.0012 |
| center_drift_m | 0.0154 |

Source: `calibration_data/2026-10-06_issue13_rotation_stops_01` (8/8 valid stops) and `calibration_data/2026-10-06_issue13_esp32_watchdog_02` (4/4 watchdog cut-offs, 0.49-0.57 s, +8.4-8.9 deg). Analysis: `docs/evidence/issue13_pr4_rotation_profile_2026-10-06.md`, `docs/evidence/issue13_pr4_esp32_watchdog_2026-10-06.md`.

## Scope of validity

- Probe speed 0.40 rad/s only; the wheels deliver about 0.25-0.26 rad/s at this command.
- The floor and payload present on 2026-10-06 at the operator's test area. Other surfaces or payloads are not covered.
- Bound to geometry_hash `c92b05c4...` (footprint + 0.05 m padding), extrinsics_hash `7691c3c2...` (`sensors.mid360`) and control_chain_hash `f7942d7c...` (`control` section of `carbot_parameters.yaml`). Any change to those inputs invalidates the profile (`PROFILE_INVALID`).

## What it does not enable

Acceptance alone does not move the robot. Guarded rotation additionally needs `localization_strategy=segmented_rotation`, `motion_policy=guarded`, this profile path and both hashes at launch, `operator_rotation_clear` for the sensor blind zone, `carbot_msgs` in the container (done) and a fresh, unblocked chassis. The start script still refuses `segmented_rotation`; enabling it is PR5 work and every motion test still needs separate authorization with an operator on site.
