## Ablation: the path-tracking controller

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 12/12 | 37.52 | 23.06 | 14.60 | – | 1.17 | 0.00 | 2.3 |
| stanley | 12/12 | 35.73 | 23.00 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| mpc | 12/12 | 35.93 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 0.017 | 1.000 | 0.044 | 0.022 | 0.02 | 1.000 | 0.994 | 0.116 | 1.08 |
| stanley | 0.015 | 1.000 | 0.036 | 0.021 | 0.02 | 1.000 | 0.994 | 0.105 | 0.88 |
| mpc | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |

Compute time per call, p50 / worst p99 (µs):

| Module | pure_pursuit | stanley | mpc |
|---|---|---|---|
| controller | 21 / 26 | 12 / 15 | 464 / 835 |
| estimation_predict | 0 / 0 | 0 / 0 | 0 / 1 |
| estimation_update | 165 / 1339 | 176 / 1358 | 168 / 1418 |
| exploration | 101 / 232 | 114 / 234 | 115 / 248 |
| perception | 325 / 580 | 366 / 584 | 350 / 624 |
| stack_control | 21 / 26 | 12 / 15 | 464 / 836 |
| stack_scan | 536 / 1992 | 587 / 1996 | 554 / 2053 |
