## Ablation: perfect perception and perfect state

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| none | 12/12 | 39.06 | 23.01 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| perception | 12/12 | 39.09 | 23.00 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| state | 12/12 | 39.22 | 22.97 | 14.55 | 0.00 | 0.00 | 0.00 | 0.0 |
| both | 12/12 | 39.25 | 22.96 | 14.55 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| none | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |
| perception | 0.020 | 1.000 | 0.040 | 0.024 | 0.02 | 1.000 | 0.994 | 0.027 | 0.78 |
| state | 0.004 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.029 | 0.76 |
| both | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.029 | 0.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | none | perception | state | both |
|---|---|---|---|---|
| controller | 448 / 750 | 488 / 823 | 499 / 826 | 463 / 827 |
| estimation_predict | 0 / 0 | 0 / 0 | – | – |
| estimation_update | 170 / 1310 | 185 / 1480 | – | – |
| exploration | 88 / 207 | 104 / 208 | 105 / 214 | 104 / 225 |
| perception | 341 / 566 | – | 369 / 599 | – |
| stack_control | 448 / 751 | 488 / 825 | 499 / 827 | 463 / 828 |
| stack_scan | 554 / 1931 | 194 / 1609 | 413 / 728 | 9 / 181 |
