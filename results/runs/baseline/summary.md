## Baseline: the full stack against the oracles

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| stack | 33/33 | 40.06 | 23.83 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| oracle | 33/33 | 40.26 | 23.79 | 14.55 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| stack | 0.020 | 1.000 | 0.040 | 0.024 | 0.02 | 1.000 | 0.993 | 0.026 | 0.78 |
| oracle | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.027 | 0.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | stack | oracle |
|---|---|---|
| controller | 380 / 751 | 412 / 746 |
| estimation_predict | 0 / 1 | – |
| estimation_update | 162 / 1242 | – |
| exploration | 80 / 217 | 90 / 204 |
| perception | 290 / 595 | – |
| stack_control | 380 / 751 | 412 / 747 |
| stack_scan | 479 / 1823 | 9 / 180 |
