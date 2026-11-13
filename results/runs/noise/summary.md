## Ablation: sensor noise

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| noise-0 | 12/12 | 39.04 | 23.01 | 14.58 | – | 0.00 | 0.00 | 0.0 |
| noise-0.5 | 12/12 | 39.09 | 23.01 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| noise-1 | 12/12 | 39.06 | 23.01 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| noise-2 | 12/12 | 39.05 | 23.03 | 14.55 | – | 0.00 | 0.00 | 0.0 |
| noise-4 | 12/12 | 39.02 | 23.02 | 14.58 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| noise-0 | 0.004 | 1.000 | 0.018 | 0.011 | 0.01 | 1.000 | 0.994 | 0.028 | 0.77 |
| noise-0.5 | 0.009 | 1.000 | 0.022 | 0.015 | 0.02 | 1.000 | 0.994 | 0.027 | 0.77 |
| noise-1 | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |
| noise-2 | 0.037 | 1.000 | 0.096 | 0.046 | 0.05 | 1.000 | 0.991 | 0.027 | 0.78 |
| noise-4 | 0.086 | 0.999 | 0.152 | 0.108 | 0.14 | 1.000 | 0.811 | 0.034 | 0.82 |

Compute time per call, p50 / worst p99 (µs):

| Module | noise-0 | noise-0.5 | noise-1 | noise-2 | noise-4 |
|---|---|---|---|---|---|
| controller | 496 / 845 | 496 / 841 | 497 / 837 | 497 / 836 | 470 / 833 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 0 | 0 / 0 |
| estimation_update | 191 / 1481 | 193 / 1485 | 191 / 1455 | 180 / 1320 | 117 / 1031 |
| exploration | 97 / 230 | 105 / 215 | 106 / 222 | 105 / 216 | 105 / 228 |
| perception | 327 / 563 | 357 / 599 | 368 / 618 | 337 / 548 | 314 / 493 |
| stack_control | 496 / 845 | 496 / 842 | 496 / 837 | 496 / 836 | 470 / 834 |
| stack_scan | 559 / 2111 | 602 / 2142 | 606 / 2107 | 571 / 1902 | 470 / 1532 |
