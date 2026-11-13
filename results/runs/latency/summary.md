## Ablation: processing and actuator latency

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| nominal | 12/12 | 38.87 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| processing-2x | 12/12 | 39.33 | 23.00 | 14.55 | – | 0.00 | 0.00 | 0.0 |
| processing-4x | 12/12 | 39.39 | 23.00 | 14.55 | – | 0.00 | 0.00 | 0.0 |
| actuators-2x | 12/12 | 37.91 | 23.01 | 14.55 | – | 0.00 | 0.00 | 0.0 |
| actuators-2x-unmodeled | 12/12 | 38.76 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| nominal | 0.031 | 1.000 | 0.083 | 0.036 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |
| processing-2x | 0.033 | 1.000 | 0.076 | 0.038 | 0.03 | 1.000 | 0.993 | 0.024 | 0.78 |
| processing-4x | 0.030 | 1.000 | 0.084 | 0.038 | 0.03 | 1.000 | 0.993 | 0.023 | 0.78 |
| actuators-2x | 0.035 | 1.000 | 0.085 | 0.040 | 0.04 | 1.000 | 0.993 | 0.026 | 0.77 |
| actuators-2x-unmodeled | 0.033 | 1.000 | 0.073 | 0.037 | 0.03 | 1.000 | 0.993 | 0.050 | 0.79 |

Compute time per call, p50 / worst p99 (µs):

| Module | nominal | processing-2x | processing-4x | actuators-2x | actuators-2x-unmodeled |
|---|---|---|---|---|---|
| controller | 554 / 2208 | 575 / 2260 | 564 / 1800 | 561 / 2361 | 536 / 1284 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 203 / 1498 | 208 / 1348 | 209 / 1307 | 202 / 1383 | 191 / 1243 |
| exploration | 84 / 466 | 91 / 762 | 85 / 403 | 90 / 563 | 90 / 310 |
| perception | 422 / 2106 | 435 / 2188 | 428 / 1545 | 426 / 1824 | 403 / 936 |
| stack_control | 554 / 2266 | 575 / 2307 | 564 / 1817 | 561 / 2375 | 535 / 1294 |
| stack_scan | 672 / 2622 | 686 / 3136 | 685 / 2531 | 671 / 2892 | 631 / 1975 |
