## Lap 1 and racing, both oracles, three controllers

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| mpc | 9/9 | 37.43 | 22.48 | 10.24 | – | 0.00 | 0.00 | 0.0 |
| stanley | 9/9 | 35.73 | 22.48 | 10.27 | – | 0.00 | 0.00 | 0.0 |
| pure_pursuit | 9/9 | 38.48 | 22.53 | 10.29 | – | 0.67 | 0.11 | 2.4 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| mpc | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.040 | 0.76 |
| stanley | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.108 | 0.89 |
| pure_pursuit | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.115 | 5.55 |

Compute time per call, p50 / worst p99 (µs):

| Module | mpc | stanley | pure_pursuit |
|---|---|---|---|
| controller | 351 / 464 | 10 / 11 | 19 / 29 |
| exploration | 69 / 139 | 71 / 152 | 78 / 165 |
| stack_control | 351 / 464 | 10 / 11 | 19 / 30 |
| stack_scan | 7 / 136 | 7 / 145 | 8 / 165 |
