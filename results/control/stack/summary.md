## Lap 1 and racing, full stack, three controllers

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| mpc | 9/9 | 37.15 | 22.60 | 10.25 | – | 0.44 | 0.22 | 3.1 |
| stanley | 9/9 | 35.84 | 22.53 | 10.29 | – | 0.00 | 0.00 | 0.0 |
| pure_pursuit | 9/9 | 41.80 | 22.59 | 10.31 | – | 6.11 | 0.00 | 12.2 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| mpc | 0.034 | 1.000 | 0.080 | 0.040 | 0.03 | 1.000 | 0.991 | 0.070 | 3.00 |
| stanley | 0.037 | 1.000 | 0.085 | 0.043 | 0.03 | 1.000 | 0.991 | 0.109 | 0.90 |
| pure_pursuit | 0.034 | 1.000 | 0.080 | 0.040 | 0.03 | 1.000 | 0.990 | 0.085 | 1.97 |

Compute time per call, p50 / worst p99 (µs):

| Module | mpc | stanley | pure_pursuit |
|---|---|---|---|
| controller | 312 / 812 | 9 / 12 | 16 / 21 |
| estimation_predict | 0 / 0 | 0 / 0 | 0 / 0 |
| estimation_update | 105 / 722 | 110 / 786 | 106 / 731 |
| exploration | 43 / 115 | 46 / 119 | 47 / 130 |
| perception | 239 / 334 | 251 / 381 | 243 / 361 |
| stack_control | 313 / 815 | 9 / 12 | 16 / 21 |
| stack_scan | 359 / 1042 | 380 / 1144 | 368 / 1064 |
