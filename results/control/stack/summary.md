## Lap 1 and racing, full stack, three controllers

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| mpc | 9/9 | 37.31 | 22.54 | 10.26 | – | 0.00 | 0.00 | 0.0 |
| stanley | 9/9 | 35.79 | 22.53 | 10.29 | – | 0.00 | 0.00 | 0.0 |
| pure_pursuit | 9/9 | 38.52 | 22.59 | 10.30 | – | 0.44 | 0.00 | 0.9 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| mpc | 0.034 | 1.000 | 0.066 | 0.040 | 0.03 | 1.000 | 0.991 | 0.040 | 0.77 |
| stanley | 0.034 | 1.000 | 0.071 | 0.041 | 0.03 | 1.000 | 0.991 | 0.109 | 0.88 |
| pure_pursuit | 0.030 | 1.000 | 0.058 | 0.037 | 0.03 | 1.000 | 0.990 | 0.112 | 1.54 |

Compute time per call, p50 / worst p99 (µs):

| Module | mpc | stanley | pure_pursuit |
|---|---|---|---|
| controller | 329 / 708 | 10 / 16 | 18 / 25 |
| estimation_predict | 0 / 1 | 0 / 0 | 0 / 0 |
| estimation_update | 111 / 1017 | 134 / 926 | 115 / 838 |
| exploration | 48 / 174 | 56 / 193 | 53 / 140 |
| perception | 253 / 501 | 308 / 540 | 275 / 457 |
| stack_control | 329 / 710 | 10 / 17 | 18 / 26 |
| stack_scan | 383 / 1582 | 463 / 1354 | 412 / 1210 |
