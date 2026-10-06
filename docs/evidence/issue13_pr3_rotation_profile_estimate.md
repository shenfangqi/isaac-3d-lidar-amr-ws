# Issue #13 rotation motion profile estimate

Status: **ESTIMATED** (never REVIEWED/ACCEPTED automatically). Odometry-only; no external reference.

- geometry_hash `c92b05c46bf28ccb46de7269aa753f2c56d8577709017bc6c60a4f3f0584b9fe` (footprint + padding 0.05 m)
- extrinsics_hash `7691c3c24bd782869c291d7121f4679fb5da1e453340635343061c98580b4bf3` (sensors.mid360)
- control_chain_hash `f7942d7c4e72fe5946076e2f2123e316ca60d30f7e3139e4922596d3fc413284` (control)
- valid stops: 8/10 (left 4, right 4; need 3 each)
- invalid reasons: other speed
- start_latency_s: max 0.591 s, median 0.408 s (n=8)
- stop_latency_s: max 0.138 s, median 0.058 s (n=8)
- stop_tail_rad: max 0.069 rad, median 0.048 rad (n=8)
- center_drift_m: max 0.008 m, median 0.005 m (n=8)
- rate_at_stop: max 0.254 rad/s, median 0.238 rad/s (n=8)

| bag | command rad/s | result | start latency s | stop latency s | stop tail rad | centre drift m |
| --- | --- | --- | --- | --- | --- | --- |
| `ground_turn_0p40_4pairs_10` | +0.40 | ok | 0.417 | 0.076 | 0.056 | 0.004 |
| `ground_turn_0p40_4pairs_10` | -0.40 | ok | 0.436 | 0.135 | 0.018 | 0.006 |
| `ground_turn_0p40_4pairs_10` | -0.40 | ok | 0.591 | 0.010 | 0.069 | 0.008 |
| `ground_turn_0p40_4pairs_10` | +0.40 | ok | 0.368 | 0.127 | 0.027 | 0.005 |
| `ground_turn_0p40_4pairs_10` | +0.40 | ok | 0.386 | 0.027 | 0.053 | 0.003 |
| `ground_turn_0p40_4pairs_10` | -0.40 | ok | 0.564 | 0.002 | 0.063 | 0.004 |
| `ground_turn_0p40_4pairs_10` | -0.40 | ok | 0.381 | 0.040 | 0.042 | 0.005 |
| `ground_turn_0p40_4pairs_10` | +0.40 | ok | 0.399 | 0.138 | 0.024 | 0.005 |
| `turn_left_0p20_8s_bag` | +0.20 | other speed |  |  |  |  |
| `turn_right_0p20_12s_bag` | -0.20 | other speed |  |  |  |  |
