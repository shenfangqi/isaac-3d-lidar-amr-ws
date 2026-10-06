# Issue #13 rotation motion profile estimate

Status: **ESTIMATED** (never REVIEWED/ACCEPTED automatically). Odometry-only; no external reference.

- geometry_hash `c92b05c46bf28ccb46de7269aa753f2c56d8577709017bc6c60a4f3f0584b9fe` (footprint + padding 0.05 m)
- extrinsics_hash `7691c3c24bd782869c291d7121f4679fb5da1e453340635343061c98580b4bf3` (sensors.mid360)
- control_chain_hash `f7942d7c4e72fe5946076e2f2123e316ca60d30f7e3139e4922596d3fc413284` (control)
- valid stops: 8/8 (left 4, right 4; need 3 each)
- invalid reasons: none
- start_latency_s: max 0.332 s, median 0.234 s (n=8)
- stop_latency_s: max 0.171 s, median 0.153 s (n=8)
- stop_tail_rad: max 0.001 rad, median 0.000 rad (n=8)
- center_drift_m: max 0.015 m, median 0.014 m (n=8)
- rate_at_stop: max 0.298 rad/s, median 0.262 rad/s (n=8)

| bag | command rad/s | result | start latency s | stop latency s | stop tail rad | centre drift m |
| --- | --- | --- | --- | --- | --- | --- |
| `2026-10-06_issue13_rotation_stops_01` | +0.40 | ok | 0.103 | 0.069 | 0.001 | 0.015 |
| `2026-10-06_issue13_rotation_stops_01` | -0.40 | ok | 0.235 | 0.163 | 0.000 | 0.015 |
| `2026-10-06_issue13_rotation_stops_01` | +0.40 | ok | 0.232 | 0.171 | 0.000 | 0.015 |
| `2026-10-06_issue13_rotation_stops_01` | -0.40 | ok | 0.228 | 0.161 | 0.000 | 0.013 |
| `2026-10-06_issue13_rotation_stops_01` | +0.40 | ok | 0.231 | 0.099 | 0.000 | 0.014 |
| `2026-10-06_issue13_rotation_stops_01` | -0.40 | ok | 0.259 | 0.155 | 0.000 | 0.013 |
| `2026-10-06_issue13_rotation_stops_01` | +0.40 | ok | 0.332 | 0.151 | 0.001 | 0.010 |
| `2026-10-06_issue13_rotation_stops_01` | -0.40 | ok | 0.313 | 0.091 | 0.001 | 0.014 |
