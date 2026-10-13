# Issue #53 linear motion profile estimate

Status: **ESTIMATED** (never REVIEWED/ACCEPTED automatically). Odometry positions only; no external reference.

- probe speed 0.08 m/s; geometry_hash `c92b05c46bf28ccb46de7269aa753f2c56d8577709017bc6c60a4f3f0584b9fe`, extrinsics_hash `7691c3c24bd782869c291d7121f4679fb5da1e453340635343061c98580b4bf3`, control_chain_hash `f7942d7c4e72fe5946076e2f2123e316ca60d30f7e3139e4922596d3fc413284`
- valid: 12/12 (forward stops 4, backward stops 4, watchdog trials 4; need 3 stops each way and 3 trials)
- invalid reasons: none
- start_latency_s: max 0.587 s, median 0.448 s (n=12)
- reaction_s: max 0.158 s, median 0.087 s (n=8)
- stop_tail_m: max 0.023 m, median 0.002 m (n=12)
- watchdog_s: max 0.813 s, median 0.771 s (n=4)
- lateral_error_m: max 0.012 m, median 0.005 m (n=12)
- yaw_change_rad: max 0.042 rad, median 0.013 rad (n=12)
- tracking (speed before the stop / command): min 0.63, median 0.79

| bag | command m/s | result | reaction s | stop tail m | watchdog s | travel m | lateral m | yaw deg |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `2026-10-13_issue53_linear_stops_02` | +0.080 | ok | 0.078 | 0.002 |  | 0.414 | 0.011 | 2.4 |
| `2026-10-13_issue53_linear_stops_02` | -0.080 | ok | 0.060 | 0.004 |  | 0.409 | 0.004 | 0.2 |
| `2026-10-13_issue53_linear_stops_02` | +0.080 | ok | 0.087 | 0.005 |  | 0.417 | 0.012 | 2.1 |
| `2026-10-13_issue53_linear_stops_02` | -0.080 | ok | 0.158 | 0.000 |  | 0.416 | 0.004 | 0.4 |
| `2026-10-13_issue53_linear_stops_02` | +0.080 | ok | 0.087 | 0.001 |  | 0.408 | 0.011 | 2.4 |
| `2026-10-13_issue53_linear_stops_02` | -0.080 | ok | 0.092 | 0.003 |  | 0.411 | 0.004 | 0.5 |
| `2026-10-13_issue53_linear_stops_02` | +0.080 | ok | 0.114 | 0.003 |  | 0.410 | 0.011 | 2.4 |
| `2026-10-13_issue53_linear_stops_02` | -0.080 | ok | 0.081 | 0.002 |  | 0.410 | 0.005 | 0.8 |
| `2026-10-13_issue53_linear_watchdog_01` | +0.080 cut | ok | 0.601 | 0.001 | 0.813 | 0.131 | 0.003 | 1.0 |
| `2026-10-13_issue53_linear_watchdog_01` | -0.080 cut | ok | 0.278 | 0.019 | 0.680 | 0.128 | 0.005 | 0.5 |
| `2026-10-13_issue53_linear_watchdog_01` | +0.080 cut | ok | 0.569 | 0.002 | 0.771 | 0.133 | 0.004 | 0.7 |
| `2026-10-13_issue53_linear_watchdog_01` | -0.080 cut | ok | 0.163 | 0.023 | 0.770 | 0.129 | 0.005 | 0.4 |
