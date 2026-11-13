## Ablation: friction mismatch (the planner keeps the nominal tires' mu)

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| grip-110 | 12/12 | 39.05 | 22.95 | 14.53 | – | 0.00 | 0.00 | 0.0 |
| grip-100 | 12/12 | 39.06 | 23.01 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| grip-90 | 12/12 | 39.06 | 23.08 | 14.62 | – | 0.00 | 0.00 | 0.0 |
| grip-80 | 12/12 | 39.06 | 23.20 | 14.70 | – | 0.00 | 0.00 | 0.0 |
| grip-70 | 6/12 | 39.20 | 25.03 | 15.39 | – | 21.17 | 1.08 | 53.2 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| grip-110 | 0.019 | 1.000 | 0.069 | 0.024 | 0.03 | 1.000 | 0.994 | 0.031 | 0.82 |
| grip-100 | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |
| grip-90 | 0.017 | 1.000 | 0.060 | 0.025 | 0.02 | 1.000 | 0.994 | 0.049 | 0.75 |
| grip-80 | 0.022 | 1.000 | 0.155 | 0.035 | 0.04 | 1.000 | 0.994 | 0.107 | 0.80 |
| grip-70 | 0.139 | 1.000 | 0.252 | 1.233 | 0.53 | 0.995 | 0.809 | 0.514 | 6.75 |

Compute time per call, p50 / worst p99 (µs):

| Module | grip-110 | grip-100 | grip-90 | grip-80 | grip-70 |
|---|---|---|---|---|---|
| controller | 494 / 835 | 496 / 838 | 496 / 837 | 490 / 843 | 470 / 1312 |
| estimation_predict | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| estimation_update | 190 / 1432 | 190 / 1399 | 190 / 1406 | 192 / 1406 | 173 / 1630 |
| exploration | 100 / 223 | 105 / 212 | 105 / 208 | 106 / 217 | 105 / 215 |
| perception | 367 / 619 | 369 / 614 | 370 / 619 | 370 / 631 | 347 / 764 |
| stack_control | 493 / 836 | 496 / 838 | 496 / 838 | 490 / 843 | 470 / 1313 |
| stack_scan | 599 / 2058 | 606 / 2057 | 605 / 2042 | 607 / 2032 | 573 / 2273 |

Runs that did not finish:

- grip-70: off the track by 5.0 m (3 runs)
- grip-70: stalled for 10 s (3 runs)
