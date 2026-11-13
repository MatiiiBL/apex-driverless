## Ablation: sensor noise

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| noise-0 | 12/12 | 38.90 | 23.02 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| noise-0.5 | 12/12 | 38.87 | 22.98 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| noise-1 | 12/12 | 38.87 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| noise-2 | 12/12 | 38.96 | 23.01 | 14.54 | – | 0.00 | 0.00 | 0.0 |
| noise-4 | 0/12 | – | – | – | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| noise-0 | 0.010 | 1.000 | 0.037 | 0.019 | 0.02 | 1.000 | 0.994 | 0.024 | 0.78 |
| noise-0.5 | 0.017 | 1.000 | 0.051 | 0.023 | 0.02 | 1.000 | 0.994 | 0.024 | 0.78 |
| noise-1 | 0.031 | 1.000 | 0.083 | 0.036 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |
| noise-2 | 0.060 | 1.000 | 0.169 | 0.065 | 0.06 | 1.000 | 0.989 | 0.025 | 0.78 |
| noise-4 | 0.020 | 0.029 | 0.059 | – | – | 1.000 | 0.863 | – | 0.00 |

Compute time per call, p50 / worst p99 (µs):

| Module | noise-0 | noise-0.5 | noise-1 | noise-2 | noise-4 |
|---|---|---|---|---|---|
| controller | 535 / 879 | 534 / 875 | 523 / 1352 | 521 / 1563 | – |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 195 / 1182 | 193 / 1187 | 193 / 1153 | 177 / 1212 | 13 / 126 |
| exploration | 79 / 225 | 85 / 220 | 85 / 219 | 87 / 432 | 2 / 155 |
| perception | 364 / 608 | 396 / 612 | 402 / 1054 | 366 / 1112 | 380 / 1251 |
| stack_control | 535 / 880 | 533 / 876 | 523 / 1360 | 520 / 1569 | 0 / 2 |
| stack_scan | 598 / 1803 | 630 / 1746 | 638 / 1781 | 583 / 2112 | 396 / 1284 |

Runs that did not finish:

- noise-4: mission failed: no exploration path for 3.02 s (12 runs)
