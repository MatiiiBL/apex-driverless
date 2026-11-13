## Ablation: the path-tracking controller

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 12/12 | 40.51 | 23.06 | 14.60 | – | 1.25 | 0.00 | 2.5 |
| stanley | 12/12 | 37.88 | 23.00 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| mpc | 12/12 | 39.06 | 23.01 | 14.56 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 0.017 | 1.000 | 0.040 | 0.023 | 0.02 | 1.000 | 0.994 | 0.117 | 1.09 |
| stanley | 0.018 | 1.000 | 0.035 | 0.023 | 0.02 | 1.000 | 0.994 | 0.106 | 0.94 |
| mpc | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |

Compute time per call, p50 / worst p99 (µs):

| Module | pure_pursuit | stanley | mpc |
|---|---|---|---|
| controller | 20 / 26 | 11 / 14 | 413 / 745 |
| estimation_predict | 0 / 0 | 0 / 0 | 0 / 0 |
| estimation_update | 164 / 1297 | 175 / 1324 | 159 / 1288 |
| exploration | 82 / 199 | 95 / 206 | 92 / 208 |
| perception | 307 / 571 | 343 / 569 | 311 / 565 |
| stack_control | 20 / 27 | 12 / 14 | 413 / 745 |
| stack_scan | 503 / 1921 | 564 / 1941 | 508 / 1859 |
