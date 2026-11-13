# Evaluation results

Produced by `tools/run_evaluation.sh` at commit f8037da. The methodology, the definitions of every column and the analysis are in [docs/evaluation.md](../../docs/evaluation.md); each sweep's config is in [configs/](configs/) and every run's full result in `<sweep>/runs.json`.

## Baseline: the full stack against the oracles

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| stack | 33/33 | 40.06 | 23.83 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| oracle | 33/33 | 40.26 | 23.79 | 14.55 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| stack | 0.020 | 1.000 | 0.040 | 0.024 | 0.02 | 1.000 | 0.993 | 0.026 | 0.78 |
| oracle | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.027 | 0.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | stack | oracle |
|---|---|---|
| controller | 380 / 751 | 412 / 746 |
| estimation_predict | 0 / 1 | – |
| estimation_update | 162 / 1242 | – |
| exploration | 80 / 217 | 90 / 204 |
| perception | 290 / 595 | – |
| stack_control | 380 / 751 | 412 / 747 |
| stack_scan | 479 / 1823 | 9 / 180 |

## Ablation: the path-tracking controller

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 12/12 | 40.51 | 23.06 | 14.60 | – | 1.25 | 0.00 | 2.5 |
| stanley | 12/12 | 37.88 | 23.00 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| mpc | 12/12 | 39.06 | 23.01 | 14.56 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 0.017 | 1.000 | 0.040 | 0.023 | 0.02 | 1.000 | 0.994 | 0.117 | 1.09 |
| stanley | 0.018 | 1.000 | 0.035 | 0.023 | 0.02 | 1.000 | 0.994 | 0.106 | 0.94 |
| mpc | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |

Compute time per call, p50 / worst p99 (µs):

| Module | pure_pursuit | stanley | mpc |
|---|---|---|---|
| controller | 20 / 26 | 11 / 14 | 413 / 745 |
| estimation_predict | 0 / 0 | 0 / 0 | 0 / 0 |
| estimation_update | 164 / 1297 | 175 / 1324 | 159 / 1288 |
| exploration | 82 / 199 | 95 / 206 | 92 / 208 |
| perception | 307 / 571 | 343 / 569 | 311 / 565 |
| stack_control | 20 / 27 | 12 / 14 | 413 / 745 |
| stack_scan | 503 / 1921 | 564 / 1941 | 508 / 1859 |

## Ablation: perfect perception and perfect state

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| none | 12/12 | 39.06 | 23.01 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| perception | 12/12 | 39.09 | 23.00 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| state | 12/12 | 39.22 | 22.97 | 14.55 | 0.00 | 0.00 | 0.00 | 0.0 |
| both | 12/12 | 39.25 | 22.96 | 14.55 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| none | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |
| perception | 0.020 | 1.000 | 0.040 | 0.024 | 0.02 | 1.000 | 0.994 | 0.027 | 0.78 |
| state | 0.004 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.029 | 0.76 |
| both | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.029 | 0.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | none | perception | state | both |
|---|---|---|---|---|
| controller | 448 / 750 | 488 / 823 | 499 / 826 | 463 / 827 |
| estimation_predict | 0 / 0 | 0 / 0 | – | – |
| estimation_update | 170 / 1310 | 185 / 1480 | – | – |
| exploration | 88 / 207 | 104 / 208 | 105 / 214 | 104 / 225 |
| perception | 341 / 566 | – | 369 / 599 | – |
| stack_control | 448 / 751 | 488 / 825 | 499 / 827 | 463 / 828 |
| stack_scan | 554 / 1931 | 194 / 1609 | 413 / 728 | 9 / 181 |

## Ablation: sensor noise

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| noise-0 | 12/12 | 39.04 | 23.01 | 14.58 | – | 0.00 | 0.00 | 0.0 |
| noise-0.5 | 12/12 | 39.09 | 23.01 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| noise-1 | 12/12 | 39.06 | 23.01 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| noise-2 | 12/12 | 39.05 | 23.03 | 14.55 | – | 0.00 | 0.00 | 0.0 |
| noise-4 | 12/12 | 39.02 | 23.02 | 14.58 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| noise-0 | 0.004 | 1.000 | 0.018 | 0.011 | 0.01 | 1.000 | 0.994 | 0.028 | 0.77 |
| noise-0.5 | 0.009 | 1.000 | 0.022 | 0.015 | 0.02 | 1.000 | 0.994 | 0.027 | 0.77 |
| noise-1 | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |
| noise-2 | 0.037 | 1.000 | 0.096 | 0.046 | 0.05 | 1.000 | 0.991 | 0.027 | 0.78 |
| noise-4 | 0.086 | 0.999 | 0.152 | 0.108 | 0.14 | 1.000 | 0.811 | 0.034 | 0.82 |

Compute time per call, p50 / worst p99 (µs):

| Module | noise-0 | noise-0.5 | noise-1 | noise-2 | noise-4 |
|---|---|---|---|---|---|
| controller | 496 / 845 | 496 / 841 | 497 / 837 | 497 / 836 | 470 / 833 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 0 | 0 / 0 |
| estimation_update | 191 / 1481 | 193 / 1485 | 191 / 1455 | 180 / 1320 | 117 / 1031 |
| exploration | 97 / 230 | 105 / 215 | 106 / 222 | 105 / 216 | 105 / 228 |
| perception | 327 / 563 | 357 / 599 | 368 / 618 | 337 / 548 | 314 / 493 |
| stack_control | 496 / 845 | 496 / 842 | 496 / 837 | 496 / 836 | 470 / 834 |
| stack_scan | 559 / 2111 | 602 / 2142 | 606 / 2107 | 571 / 1902 | 470 / 1532 |

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

## Ablation: friction mismatch (the planner keeps the nominal tires' mu)

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| grip-110 | 12/12 | 39.05 | 22.95 | 14.53 | – | 0.00 | 0.00 | 0.0 |
| grip-100 | 12/12 | 39.06 | 23.01 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| grip-90 | 12/12 | 39.06 | 23.08 | 14.62 | – | 0.00 | 0.00 | 0.0 |
| grip-80 | 12/12 | 39.06 | 23.20 | 14.70 | – | 0.00 | 0.00 | 0.0 |
| grip-70 | 6/12 | 39.20 | 25.03 | 15.39 | – | 21.17 | 1.08 | 53.2 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| grip-110 | 0.019 | 1.000 | 0.069 | 0.024 | 0.03 | 1.000 | 0.994 | 0.031 | 0.82 |
| grip-100 | 0.020 | 1.000 | 0.043 | 0.025 | 0.03 | 1.000 | 0.994 | 0.027 | 0.77 |
| grip-90 | 0.017 | 1.000 | 0.060 | 0.025 | 0.02 | 1.000 | 0.994 | 0.049 | 0.75 |
| grip-80 | 0.022 | 1.000 | 0.155 | 0.035 | 0.04 | 1.000 | 0.994 | 0.107 | 0.80 |
| grip-70 | 0.139 | 1.000 | 0.252 | 1.233 | 0.53 | 0.995 | 0.809 | 0.514 | 6.75 |

Compute time per call, p50 / worst p99 (µs):

| Module | grip-110 | grip-100 | grip-90 | grip-80 | grip-70 |
|---|---|---|---|---|---|
| controller | 494 / 835 | 496 / 838 | 496 / 837 | 490 / 843 | 470 / 1312 |
| estimation_predict | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| estimation_update | 190 / 1432 | 190 / 1399 | 190 / 1406 | 192 / 1406 | 173 / 1630 |
| exploration | 100 / 223 | 105 / 212 | 105 / 208 | 106 / 217 | 105 / 215 |
| perception | 367 / 619 | 369 / 614 | 370 / 619 | 370 / 631 | 347 / 764 |
| stack_control | 493 / 836 | 496 / 838 | 496 / 838 | 490 / 843 | 470 / 1313 |
| stack_scan | 599 / 2058 | 606 / 2057 | 605 / 2042 | 607 / 2032 | 573 / 2273 |

Runs that did not finish:

- grip-70: off the track by 5.0 m (3 runs)
- grip-70: stalled for 10 s (3 runs)
