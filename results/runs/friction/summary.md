## Ablation: friction mismatch (the planner keeps the nominal tires' mu)

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| grip-110 | 12/12 | 35.86 | 22.95 | 14.53 | – | 0.00 | 0.00 | 0.0 |
| grip-100 | 12/12 | 35.93 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| grip-90 | 12/12 | 35.98 | 23.08 | 14.62 | – | 0.00 | 0.00 | 0.0 |
| grip-80 | 12/12 | 36.00 | 23.20 | 14.70 | – | 0.00 | 0.00 | 0.0 |
| grip-70 | 6/12 | 35.79 | 24.71 | 15.93 | – | 25.50 | 1.83 | 69.3 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| grip-110 | 0.021 | 1.000 | 0.067 | 0.028 | 0.03 | 1.000 | 0.994 | 0.033 | 0.82 |
| grip-100 | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| grip-90 | 0.017 | 1.000 | 0.050 | 0.024 | 0.02 | 1.000 | 0.994 | 0.052 | 0.75 |
| grip-80 | 0.024 | 1.000 | 0.158 | 0.038 | 0.04 | 1.000 | 0.994 | 0.109 | 0.80 |
| grip-70 | 0.142 | 1.000 | 0.285 | 1.561 | 0.63 | 0.989 | 0.759 | 0.750 | 6.75 |

Compute time per call, p50 / worst p99 (µs):

| Module | grip-110 | grip-100 | grip-90 | grip-80 | grip-70 |
|---|---|---|---|---|---|
| controller | 558 / 838 | 559 / 841 | 560 / 844 | 556 / 841 | 525 / 1344 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 198 / 1504 | 199 / 1502 | 200 / 1505 | 199 / 1485 | 174 / 1941 |
| exploration | 123 / 244 | 130 / 257 | 130 / 254 | 131 / 249 | 130 / 251 |
| perception | 418 / 627 | 419 / 648 | 419 / 636 | 420 / 638 | 398 / 669 |
| stack_control | 558 / 838 | 559 / 841 | 560 / 845 | 556 / 841 | 524 / 1346 |
| stack_scan | 660 / 2175 | 665 / 2191 | 667 / 2175 | 665 / 2184 | 630 / 2602 |

Runs that did not finish:

- grip-70: off the track by 5.0 m (2 runs)
- grip-70: stalled for 10 s (4 runs)
