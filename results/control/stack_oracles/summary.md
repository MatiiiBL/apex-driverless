## Lap 1 and racing, both oracles, three controllers

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| mpc | 9/9 | 37.13 | 22.47 | 10.24 | – | 0.00 | 0.00 | 0.0 |
| stanley | 9/9 | 35.73 | 22.48 | 10.27 | – | 0.00 | 0.00 | 0.0 |
| pure_pursuit | 9/9 | 38.48 | 22.53 | 10.29 | – | 0.67 | 0.11 | 2.4 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| mpc | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.038 | 0.76 |
| stanley | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.108 | 0.89 |
| pure_pursuit | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.115 | 5.55 |

Compute time per call, p50 / worst p99 (µs):

| Module | mpc | stanley | pure_pursuit |
|---|---|---|---|
| controller | 302 / 446 | 8 / 15 | 16 / 21 |
| exploration | 61 / 127 | 63 / 144 | 64 / 132 |
| stack_control | 302 / 448 | 8 / 15 | 16 / 21 |
| stack_scan | 6 / 121 | 6 / 121 | 6 / 124 |
