## Lap 1 and racing, both oracles, three controllers

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| mpc | 9/9 | 37.13 | 22.47 | 10.24 | – | 0.00 | 0.00 | 0.0 |
| stanley | 9/9 | 35.73 | 22.48 | 10.27 | – | 0.00 | 0.00 | 0.0 |
| pure_pursuit | 9/9 | 41.27 | 22.54 | 10.29 | – | 5.22 | 0.11 | 11.6 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| mpc | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.038 | 0.76 |
| stanley | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.108 | 0.89 |
| pure_pursuit | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.086 | 5.91 |

Compute time per call, p50 / worst p99 (µs):

| Module | mpc | stanley | pure_pursuit |
|---|---|---|---|
| controller | 312 / 440 | 9 / 14 | 18 / 26 |
| exploration | 63 / 129 | 74 / 206 | 77 / 174 |
| stack_control | 312 / 443 | 9 / 15 | 18 / 26 |
| stack_scan | 6 / 129 | 11 / 205 | 9 / 176 |
