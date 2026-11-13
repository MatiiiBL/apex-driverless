## New-landmark range: 20 m against 30 m

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| range-20 | 32/33 | 39.87 | 23.68 | 14.56 | – | 0.09 | 0.06 | 0.8 |
| range-30 | 31/33 | 39.66 | 23.53 | 14.56 | – | 0.27 | 0.15 | 2.1 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| range-20 | 0.047 | 1.000 | 0.091 | 0.053 | 0.08 | 1.000 | 0.990 | 0.150 | 6.16 |
| range-30 | 0.019 | 1.000 | 0.040 | 0.097 | 0.77 | 1.000 | 0.991 | 0.188 | 6.75 |

Compute time per call, p50 / worst p99 (µs):

| Module | range-20 | range-30 |
|---|---|---|
| controller | 334 / 927 | 394 / 2303 |
| estimation_predict | 0 / 1 | 0 / 1 |
| estimation_update | 120 / 844 | 153 / 1676 |
| exploration | 48 / 193 | 81 / 229 |
| perception | 253 / 572 | 303 / 612 |
| stack_control | 334 / 928 | 394 / 2330 |
| stack_scan | 385 / 1362 | 481 / 2297 |

Runs that did not finish:

- range-20: stalled for 10 s (1 run)
- range-30: off the track by 5.0 m (1 run)
- range-30: stalled for 10 s (1 run)
