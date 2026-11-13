## Lap 1 and racing, full stack, three controllers

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| mpc | 9/9 | 37.15 | 22.60 | 10.25 | – | 0.44 | 0.22 | 3.1 |
| stanley | 9/9 | 35.84 | 22.53 | 10.29 | – | 0.00 | 0.00 | 0.0 |
| pure_pursuit | 9/9 | 38.55 | 22.59 | 10.31 | – | 0.44 | 0.00 | 0.9 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| mpc | 0.034 | 1.000 | 0.080 | 0.040 | 0.03 | 1.000 | 0.991 | 0.070 | 3.00 |
| stanley | 0.037 | 1.000 | 0.085 | 0.043 | 0.03 | 1.000 | 0.991 | 0.109 | 0.90 |
| pure_pursuit | 0.035 | 1.000 | 0.081 | 0.042 | 0.03 | 1.000 | 0.990 | 0.111 | 1.70 |

Compute time per call, p50 / worst p99 (µs):

| Module | mpc | stanley | pure_pursuit |
|---|---|---|---|
| controller | 274 / 720 | 8 / 9 | 15 / 20 |
| estimation_predict | 0 / 0 | 0 / 0 | 0 / 0 |
| estimation_update | 92 / 634 | 97 / 680 | 98 / 677 |
| exploration | 39 / 102 | 40 / 109 | 41 / 133 |
| perception | 212 / 370 | 224 / 302 | 226 / 323 |
| stack_control | 274 / 720 | 8 / 10 | 15 / 20 |
| stack_scan | 319 / 900 | 336 / 983 | 340 / 961 |
