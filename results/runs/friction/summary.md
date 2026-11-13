## Ablation: friction mismatch (the planner keeps the nominal tires' mu)

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| grip-110 | 12/12 | 38.89 | 22.96 | 14.52 | – | 0.00 | 0.00 | 0.0 |
| grip-100 | 12/12 | 38.87 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| grip-90 | 12/12 | 38.93 | 23.07 | 14.61 | – | 0.00 | 0.00 | 0.0 |
| grip-80 | 12/12 | 38.90 | 23.18 | 14.69 | – | 0.08 | 0.00 | 0.2 |
| grip-70 | 5/12 | 39.02 | 25.34 | 15.26 | – | 24.25 | 1.92 | 67.7 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| grip-110 | 0.035 | 1.000 | 0.159 | 0.038 | 0.04 | 1.000 | 0.993 | 0.033 | 0.83 |
| grip-100 | 0.031 | 1.000 | 0.083 | 0.036 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |
| grip-90 | 0.034 | 1.000 | 0.123 | 0.039 | 0.03 | 1.000 | 0.993 | 0.041 | 0.75 |
| grip-80 | 0.132 | 1.000 | 0.313 | 0.152 | 0.11 | 1.000 | 0.988 | 0.100 | 0.98 |
| grip-70 | 0.444 | 1.000 | 0.638 | 2.019 | 0.64 | 0.998 | 0.776 | 0.888 | 6.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | grip-110 | grip-100 | grip-90 | grip-80 | grip-70 |
|---|---|---|---|---|---|
| controller | 606 / 1111 | 615 / 1630 | 585 / 1610 | 572 / 1071 | 581 / 1982 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 223 / 1296 | 224 / 1275 | 216 / 1833 | 213 / 1285 | 172 / 1462 |
| exploration | 93 / 236 | 96 / 239 | 95 / 1149 | 92 / 230 | 90 / 354 |
| perception | 459 / 813 | 462 / 1255 | 447 / 1091 | 433 / 821 | 398 / 914 |
| stack_control | 606 / 1116 | 615 / 1661 | 585 / 1623 | 572 / 1075 | 580 / 1984 |
| stack_scan | 714 / 2055 | 717 / 2200 | 701 / 2703 | 686 / 2018 | 620 / 2397 |

Runs that did not finish:

- grip-70: off the track by 5.0 m (4 runs)
- grip-70: stalled for 10 s (3 runs)
