## Ablation: the path-tracking controller

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 12/12 | 40.67 | 23.07 | 14.60 | – | 1.17 | 0.00 | 2.3 |
| stanley | 12/12 | 38.05 | 23.00 | 14.56 | – | 0.17 | 0.00 | 0.3 |
| mpc | 12/12 | 38.87 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 0.029 | 1.000 | 0.090 | 0.034 | 0.03 | 1.000 | 0.994 | 0.116 | 1.12 |
| stanley | 0.037 | 1.000 | 0.082 | 0.041 | 0.04 | 1.000 | 0.993 | 0.106 | 1.01 |
| mpc | 0.031 | 1.000 | 0.083 | 0.036 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |

Compute time per call, p50 / worst p99 (µs):

| Module | pure_pursuit | stanley | mpc |
|---|---|---|---|
| controller | 21 / 37 | 12 / 35 | 455 / 1306 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 168 / 990 | 187 / 1364 | 162 / 1161 |
| exploration | 67 / 183 | 78 / 1194 | 76 / 326 |
| perception | 338 / 686 | 385 / 2037 | 345 / 1097 |
| stack_control | 21 / 38 | 12 / 38 | 455 / 1309 |
| stack_scan | 536 / 1573 | 615 / 3285 | 542 / 1830 |
