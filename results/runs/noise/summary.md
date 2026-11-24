## Ablation: sensor noise

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| noise-0 | 12/12 | 35.96 | 23.01 | 14.58 | – | 0.00 | 0.00 | 0.0 |
| noise-0.5 | 12/12 | 35.94 | 23.01 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| noise-1 | 12/12 | 35.93 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| noise-2 | 12/12 | 35.93 | 23.03 | 14.55 | – | 0.00 | 0.00 | 0.0 |
| noise-4 | 12/12 | 35.89 | 23.01 | 14.60 | – | 0.42 | 0.00 | 0.8 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| noise-0 | 0.007 | 1.000 | 0.023 | 0.014 | 0.01 | 1.000 | 0.994 | 0.031 | 0.77 |
| noise-0.5 | 0.011 | 1.000 | 0.025 | 0.016 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| noise-1 | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| noise-2 | 0.038 | 1.000 | 0.082 | 0.048 | 0.05 | 1.000 | 0.991 | 0.030 | 0.78 |
| noise-4 | 0.137 | 0.998 | 0.186 | 0.159 | 0.18 | 1.000 | 0.799 | 0.039 | 1.06 |

Compute time per call, p50 / worst p99 (µs):

| Module | noise-0 | noise-0.5 | noise-1 | noise-2 | noise-4 |
|---|---|---|---|---|---|
| controller | 573 / 891 | 562 / 841 | 562 / 841 | 565 / 847 | 527 / 842 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 212 / 1501 | 210 / 1556 | 210 / 1519 | 199 / 1434 | 131 / 1149 |
| exploration | 126 / 246 | 131 / 254 | 130 / 250 | 131 / 260 | 129 / 255 |
| perception | 383 / 576 | 407 / 599 | 420 / 627 | 383 / 569 | 352 / 516 |
| stack_control | 573 / 891 | 562 / 842 | 562 / 842 | 564 / 848 | 526 / 842 |
| stack_scan | 646 / 2148 | 670 / 2254 | 679 / 2210 | 640 / 2094 | 525 / 1763 |
