## Sensor noise at the 30 m new-landmark range

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| noise-2 | 12/12 | 38.72 | 23.01 | 14.54 | – | 0.00 | 0.00 | 0.0 |
| noise-3 | 0/12 | – | – | – | – | 0.00 | 0.00 | 0.0 |
| noise-4 | 0/12 | – | – | – | – | 0.00 | 0.00 | 0.0 |
| noise-3-r20 | 0/12 | – | – | – | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| noise-2 | 0.038 | 1.000 | 0.081 | 0.048 | 0.05 | 1.000 | 0.991 | 0.024 | 0.79 |
| noise-3 | 0.021 | 0.083 | 0.023 | – | – | 1.000 | 0.958 | – | 0.00 |
| noise-4 | 0.024 | 0.079 | 0.040 | – | – | 1.000 | 0.846 | – | 0.00 |
| noise-3-r20 | 0.017 | 0.032 | 0.030 | – | – | 1.000 | 0.967 | – | 0.00 |

Compute time per call, p50 / worst p99 (µs):

| Module | noise-2 | noise-3 | noise-4 | noise-3-r20 |
|---|---|---|---|---|
| controller | 469 / 998 | – | – | – |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 166 / 1362 | 56 / 478 | 36 / 498 | 13 / 83 |
| exploration | 100 / 282 | 3 / 29 | 3 / 12 | 2 / 7 |
| perception | 335 / 678 | 317 / 834 | 312 / 700 | 310 / 602 |
| stack_control | 469 / 1001 | 0 / 7 | 0 / 7 | 0 / 2 |
| stack_scan | 525 / 2014 | 393 / 1093 | 358 / 830 | 329 / 641 |

Runs that did not finish:

- noise-3: mission failed: no exploration path for 3.02 s (12 runs)
- noise-4: mission failed: no exploration path for 3.02 s (12 runs)
- noise-3-r20: mission failed: no exploration path for 3.02 s (12 runs)
