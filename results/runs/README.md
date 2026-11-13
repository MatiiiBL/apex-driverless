# Evaluation results

Produced by `tools/run_evaluation.sh` at commit d51c587. The methodology, the definitions of every column and the analysis are in [docs/evaluation.md](../../docs/evaluation.md); each sweep's config is in [configs/](configs/) and every run's full result in `<sweep>/runs.json`.

## Baseline: the full stack against the oracles

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| stack | 32/33 | 39.87 | 23.68 | 14.56 | 0.05 | 0.09 | 0.06 | 0.8 |
| oracle | 32/33 | 39.89 | 23.77 | 14.54 | – | 0.06 | 0.03 | 0.4 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| stack | 0.047 | 1.000 | 0.091 | 0.053 | 0.08 | 1.000 | 0.990 | 0.150 | 6.16 |
| oracle | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.024 | 6.75 |

Compute time per call, p50 / worst p99 (µs):

| Module | stack | oracle |
|---|---|---|
| controller | 384 / 1726 | 428 / 1547 |
| estimation_predict | 0 / 1 | – |
| estimation_update | 155 / 1059 | – |
| exploration | 61 / 225 | 93 / 687 |
| perception | 293 / 662 | – |
| stack_control | 384 / 1731 | 428 / 1579 |
| stack_scan | 467 / 1702 | 10 / 287 |

Runs that did not finish:

- stack: stalled for 10 s (1 run)
- oracle: off the track by 5.0 m (1 run)

## Ablation: the path-tracking controller

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 12/12 | 40.67 | 23.07 | 14.60 | – | 1.17 | 0.00 | 2.3 |
| stanley | 12/12 | 38.05 | 23.00 | 14.56 | – | 0.17 | 0.00 | 0.3 |
| mpc | 12/12 | 38.87 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 0.029 | 1.000 | 0.090 | 0.034 | 0.03 | 1.000 | 0.994 | 0.116 | 1.12 |
| stanley | 0.037 | 1.000 | 0.082 | 0.041 | 0.04 | 1.000 | 0.993 | 0.106 | 1.01 |
| mpc | 0.031 | 1.000 | 0.083 | 0.036 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |

Compute time per call, p50 / worst p99 (µs):

| Module | pure_pursuit | stanley | mpc |
|---|---|---|---|
| controller | 21 / 37 | 12 / 35 | 455 / 1306 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 168 / 990 | 187 / 1364 | 162 / 1161 |
| exploration | 67 / 183 | 78 / 1194 | 76 / 326 |
| perception | 338 / 686 | 385 / 2037 | 345 / 1097 |
| stack_control | 21 / 38 | 12 / 38 | 455 / 1309 |
| stack_scan | 536 / 1573 | 615 / 3285 | 542 / 1830 |

## Ablation: perfect perception and perfect state

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| none | 12/12 | 38.87 | 23.00 | 14.56 | 0.06 | 0.00 | 0.00 | 0.0 |
| perception | 12/12 | 38.92 | 23.00 | 14.55 | 0.05 | 0.00 | 0.00 | 0.0 |
| state | 12/12 | 38.91 | 22.95 | 14.54 | 0.00 | 0.00 | 0.00 | 0.0 |
| both | 12/12 | 38.92 | 22.94 | 14.54 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| none | 0.031 | 1.000 | 0.083 | 0.036 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |
| perception | 0.035 | 1.000 | 0.064 | 0.037 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |
| state | 0.004 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.024 | 0.77 |
| both | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.024 | 0.77 |

Compute time per call, p50 / worst p99 (µs):

| Module | none | perception | state | both |
|---|---|---|---|---|
| controller | 468 / 870 | 500 / 984 | 509 / 880 | 488 / 898 |
| estimation_predict | 0 / 1 | 0 / 1 | – | – |
| estimation_update | 180 / 1073 | 203 / 1187 | – | – |
| exploration | 68 / 222 | 80 / 228 | 109 / 230 | 111 / 352 |
| perception | 362 / 632 | – | 393 / 669 | – |
| stack_control | 468 / 871 | 500 / 986 | 508 / 881 | 488 / 898 |
| stack_scan | 567 / 1632 | 218 / 1338 | 433 / 783 | 11 / 204 |

## Ablation: sensor noise

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| noise-0 | 12/12 | 38.90 | 23.02 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| noise-0.5 | 12/12 | 38.87 | 22.98 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| noise-1 | 12/12 | 38.87 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| noise-2 | 12/12 | 38.96 | 23.01 | 14.54 | – | 0.00 | 0.00 | 0.0 |
| noise-4 | 0/12 | – | – | – | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| noise-0 | 0.010 | 1.000 | 0.037 | 0.019 | 0.02 | 1.000 | 0.994 | 0.024 | 0.78 |
| noise-0.5 | 0.017 | 1.000 | 0.051 | 0.023 | 0.02 | 1.000 | 0.994 | 0.024 | 0.78 |
| noise-1 | 0.031 | 1.000 | 0.083 | 0.036 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |
| noise-2 | 0.060 | 1.000 | 0.169 | 0.065 | 0.06 | 1.000 | 0.989 | 0.025 | 0.78 |
| noise-4 | 0.020 | 0.029 | 0.059 | – | – | 1.000 | 0.863 | – | 0.00 |

Compute time per call, p50 / worst p99 (µs):

| Module | noise-0 | noise-0.5 | noise-1 | noise-2 | noise-4 |
|---|---|---|---|---|---|
| controller | 535 / 879 | 534 / 875 | 523 / 1352 | 521 / 1563 | – |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 195 / 1182 | 193 / 1187 | 193 / 1153 | 177 / 1212 | 13 / 126 |
| exploration | 79 / 225 | 85 / 220 | 85 / 219 | 87 / 432 | 2 / 155 |
| perception | 364 / 608 | 396 / 612 | 402 / 1054 | 366 / 1112 | 380 / 1251 |
| stack_control | 535 / 880 | 533 / 876 | 523 / 1360 | 520 / 1569 | 0 / 2 |
| stack_scan | 598 / 1803 | 630 / 1746 | 638 / 1781 | 583 / 2112 | 396 / 1284 |

Runs that did not finish:

- noise-4: mission failed: no exploration path for 3.02 s (12 runs)

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

## Ablation: friction mismatch (the planner keeps the nominal tires' mu)

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| grip-110 | 12/12 | 38.89 | 22.96 | 14.52 | – | 0.00 | 0.00 | 0.0 |
| grip-100 | 12/12 | 38.87 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| grip-90 | 12/12 | 38.93 | 23.07 | 14.61 | – | 0.00 | 0.00 | 0.0 |
| grip-80 | 12/12 | 38.90 | 23.18 | 14.69 | – | 0.08 | 0.00 | 0.2 |
| grip-70 | 5/12 | 39.02 | 25.34 | 15.26 | – | 24.25 | 1.92 | 67.7 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| grip-110 | 0.035 | 1.000 | 0.159 | 0.038 | 0.04 | 1.000 | 0.993 | 0.033 | 0.83 |
| grip-100 | 0.031 | 1.000 | 0.083 | 0.036 | 0.03 | 1.000 | 0.994 | 0.024 | 0.78 |
| grip-90 | 0.034 | 1.000 | 0.123 | 0.039 | 0.03 | 1.000 | 0.993 | 0.041 | 0.75 |
| grip-80 | 0.132 | 1.000 | 0.313 | 0.152 | 0.11 | 1.000 | 0.988 | 0.100 | 0.98 |
| grip-70 | 0.444 | 1.000 | 0.638 | 2.019 | 0.64 | 0.998 | 0.776 | 0.888 | 6.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | grip-110 | grip-100 | grip-90 | grip-80 | grip-70 |
|---|---|---|---|---|---|
| controller | 606 / 1111 | 615 / 1630 | 585 / 1610 | 572 / 1071 | 581 / 1982 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 223 / 1296 | 224 / 1275 | 216 / 1833 | 213 / 1285 | 172 / 1462 |
| exploration | 93 / 236 | 96 / 239 | 95 / 1149 | 92 / 230 | 90 / 354 |
| perception | 459 / 813 | 462 / 1255 | 447 / 1091 | 433 / 821 | 398 / 914 |
| stack_control | 606 / 1116 | 615 / 1661 | 585 / 1623 | 572 / 1075 | 580 / 1984 |
| stack_scan | 714 / 2055 | 717 / 2200 | 701 / 2703 | 686 / 2018 | 620 / 2397 |

Runs that did not finish:

- grip-70: off the track by 5.0 m (4 runs)
- grip-70: stalled for 10 s (3 runs)
