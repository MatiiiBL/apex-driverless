## Ablation: perfect perception and perfect state

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| none | 12/12 | 35.93 | 23.00 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| perception | 12/12 | 35.92 | 23.02 | 14.56 | 0.05 | 0.00 | 0.00 | 0.0 |
| state | 12/12 | 36.10 | 22.97 | 14.55 | 0.00 | 0.00 | 0.00 | 0.0 |
| both | 12/12 | 36.08 | 22.96 | 14.55 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| none | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| perception | 0.018 | 1.000 | 0.029 | 0.023 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| state | 0.004 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.032 | 0.76 |
| both | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.032 | 0.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | none | perception | state | both |
|---|---|---|---|---|
| controller | 510 / 839 | 541 / 845 | 560 / 841 | 521 / 845 |
| estimation_predict | 0 / 1 | 0 / 1 | – | – |
| estimation_update | 199 / 1373 | 213 / 1606 | – | – |
| exploration | 110 / 245 | 126 / 251 | 128 / 255 | 129 / 247 |
| perception | 390 / 627 | – | 415 / 642 | – |
| stack_control | 510 / 840 | 540 / 848 | 560 / 842 | 521 / 845 |
| stack_scan | 630 / 1986 | 224 / 1768 | 462 / 779 | 11 / 228 |
