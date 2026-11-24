## Baseline: the full stack against the oracles

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| stack | 33/33 | 36.75 | 23.83 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| oracle | 33/33 | 36.91 | 23.78 | 14.55 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| stack | 0.019 | 1.000 | 0.038 | 0.024 | 0.02 | 1.000 | 0.993 | 0.028 | 0.78 |
| oracle | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.029 | 0.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | stack | oracle |
|---|---|---|
| controller | 368 / 748 | 408 / 748 |
| estimation_predict | 0 / 0 | – |
| estimation_update | 157 / 1224 | – |
| exploration | 89 / 230 | 104 / 236 |
| perception | 282 / 557 | – |
| stack_control | 368 / 748 | 408 / 748 |
| stack_scan | 463 / 1882 | 10 / 208 |
