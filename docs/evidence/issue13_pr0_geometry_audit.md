# MID-360 定位近场可见性审计（只读）

- 扫掠环：内半径 0.13 m，外半径 0.2542 m（padding 0.05 m）
- 碰撞高度带：[0.01, 0.24] m；传感器原点 [0.0166, -0.0001, 0.209] m

| 通道 | 可证明扫掠自由 | 各半径最差不可观测比例 |
| --- | --- | --- |
| raw_mid360 | False | 0.130m:0.80, 0.155m:0.79, 0.180m:0.78, 0.204m:0.76, 0.229m:0.75, 0.254m:0.74 |
| fast_lio_cloud_registered_body | False | 0.130m:1.00, 0.155m:1.00, 0.180m:1.00, 0.204m:1.00, 0.229m:1.00, 0.254m:1.00 |
| /scan | False | 0.130m:1.00, 0.155m:1.00, 0.180m:1.00, 0.204m:1.00, 0.229m:1.00, 0.254m:1.00 |
| /scan_localization | False | 0.130m:1.00, 0.155m:1.00, 0.180m:1.00, 0.204m:1.00, 0.229m:1.00, 0.254m:1.00 |

## 结论（fail-closed）

- raw_mid360: can_prove_sweep_free=False (worst unobservable fraction of the collision band 0.80)
- fast_lio_cloud_registered_body: can_prove_sweep_free=False (worst unobservable fraction of the collision band 1.00)
- /scan: can_prove_sweep_free=False (worst unobservable fraction of the collision band 1.00)
- /scan_localization: can_prove_sweep_free=False (worst unobservable fraction of the collision band 1.00)
