## Ablation: processing and actuator latency

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| nominal | 12/12 | 39.06 | 23.01 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| processing-2x | 12/12 | 39.52 | 23.01 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| processing-4x | 12/12 | 39.54 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| actuators-2x | 12/12 | 37.86 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| actuators-2x-unmodeled | 12/12 | 38.96 | 23.01 | 14.57 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| nominal | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |
| processing-2x | 0.018 | 1.000 | 0.043 | 0.024 | 0.02 | 1.000 | 0.994 | 0.027 | 0.78 |
| processing-4x | 0.018 | 1.000 | 0.029 | 0.029 | 0.03 | 1.000 | 0.994 | 0.027 | 0.78 |
| actuators-2x | 0.020 | 1.000 | 0.045 | 0.026 | 0.02 | 1.000 | 0.994 | 0.029 | 0.77 |
| actuators-2x-unmodeled | 0.020 | 1.000 | 0.061 | 0.027 | 0.03 | 1.000 | 0.994 | 0.052 | 0.78 |

Compute time per call, p50 / worst p99 (µs):

| Module | nominal | processing-2x | processing-4x | actuators-2x | actuators-2x-unmodeled |
|---|---|---|---|---|---|
| controller | 490 / 834 | 501 / 841 | 499 / 841 | 495 / 844 | 470 / 837 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 0 | 0 / 1 | 0 / 0 |
| estimation_update | 183 / 1397 | 187 / 1435 | 188 / 1446 | 187 / 1464 | 174 / 1431 |
| exploration | 96 / 239 | 106 / 229 | 106 / 224 | 106 / 221 | 106 / 232 |
| perception | 367 / 617 | 370 / 624 | 369 / 617 | 369 / 626 | 352 / 612 |
| stack_control | 490 / 834 | 500 / 842 | 498 / 842 | 494 / 844 | 469 / 837 |
| stack_scan | 587 / 2090 | 605 / 2071 | 605 / 2050 | 605 / 2105 | 564 / 2086 |
