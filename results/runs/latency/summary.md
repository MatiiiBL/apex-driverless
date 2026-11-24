## Ablation: processing and actuator latency

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| nominal | 12/12 | 35.93 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| processing-2x | 12/12 | 36.30 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| processing-4x | 12/12 | 36.32 | 23.00 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| actuators-2x | 12/12 | 34.76 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| actuators-2x-unmodeled | 12/12 | 35.87 | 23.01 | 14.57 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| nominal | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| processing-2x | 0.023 | 1.000 | 0.038 | 0.028 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| processing-4x | 0.017 | 1.000 | 0.038 | 0.027 | 0.03 | 1.000 | 0.994 | 0.030 | 0.78 |
| actuators-2x | 0.021 | 1.000 | 0.043 | 0.026 | 0.02 | 1.000 | 0.994 | 0.031 | 0.78 |
| actuators-2x-unmodeled | 0.020 | 1.000 | 0.054 | 0.025 | 0.02 | 1.000 | 0.994 | 0.054 | 0.78 |

Compute time per call, p50 / worst p99 (µs):

| Module | nominal | processing-2x | processing-4x | actuators-2x | actuators-2x-unmodeled |
|---|---|---|---|---|---|
| controller | 553 / 842 | 558 / 841 | 560 / 853 | 561 / 846 | 532 / 840 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 200 / 1579 | 206 / 1488 | 207 / 1543 | 204 / 1533 | 193 / 1535 |
| exploration | 124 / 255 | 132 / 262 | 130 / 273 | 131 / 250 | 131 / 263 |
| perception | 416 / 630 | 419 / 636 | 420 / 642 | 419 / 631 | 401 / 632 |
| stack_control | 552 / 842 | 558 / 842 | 560 / 853 | 561 / 847 | 532 / 841 |
| stack_scan | 660 / 2285 | 673 / 2224 | 673 / 2231 | 671 / 2196 | 633 / 2234 |
