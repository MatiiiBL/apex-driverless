# Evaluation results

Produced by `tools/run_evaluation.sh` at commit 8e568dc. The methodology, the definitions of every column and the analysis are in [docs/evaluation.md](../../docs/evaluation.md); each sweep's config is in [configs/](configs/) and every run's full result in `<sweep>/runs.json`.

## Baseline: the full stack against the oracles

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| stack | 33/33 | 36.75 | 23.83 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| oracle | 33/33 | 36.91 | 23.78 | 14.55 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| stack | 0.019 | 1.000 | 0.038 | 0.024 | 0.02 | 1.000 | 0.993 | 0.028 | 0.78 |
| oracle | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.029 | 0.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | stack | oracle |
|---|---|---|
| controller | 368 / 748 | 408 / 748 |
| estimation_predict | 0 / 0 | – |
| estimation_update | 157 / 1224 | – |
| exploration | 89 / 230 | 104 / 236 |
| perception | 282 / 557 | – |
| stack_control | 368 / 748 | 408 / 748 |
| stack_scan | 463 / 1882 | 10 / 208 |

## Ablation: the path-tracking controller

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 12/12 | 37.52 | 23.06 | 14.60 | – | 1.17 | 0.00 | 2.3 |
| stanley | 12/12 | 35.73 | 23.00 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| mpc | 12/12 | 35.93 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| pure_pursuit | 0.017 | 1.000 | 0.044 | 0.022 | 0.02 | 1.000 | 0.994 | 0.116 | 1.08 |
| stanley | 0.015 | 1.000 | 0.036 | 0.021 | 0.02 | 1.000 | 0.994 | 0.105 | 0.88 |
| mpc | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |

Compute time per call, p50 / worst p99 (µs):

| Module | pure_pursuit | stanley | mpc |
|---|---|---|---|
| controller | 21 / 26 | 12 / 15 | 464 / 835 |
| estimation_predict | 0 / 0 | 0 / 0 | 0 / 1 |
| estimation_update | 165 / 1339 | 176 / 1358 | 168 / 1418 |
| exploration | 101 / 232 | 114 / 234 | 115 / 248 |
| perception | 325 / 580 | 366 / 584 | 350 / 624 |
| stack_control | 21 / 26 | 12 / 15 | 464 / 836 |
| stack_scan | 536 / 1992 | 587 / 1996 | 554 / 2053 |

## Ablation: perfect perception and perfect state

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| none | 12/12 | 35.93 | 23.00 | 14.56 | 0.04 | 0.00 | 0.00 | 0.0 |
| perception | 12/12 | 35.92 | 23.02 | 14.56 | 0.05 | 0.00 | 0.00 | 0.0 |
| state | 12/12 | 36.10 | 22.97 | 14.55 | 0.00 | 0.00 | 0.00 | 0.0 |
| both | 12/12 | 36.08 | 22.96 | 14.55 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| none | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| perception | 0.018 | 1.000 | 0.029 | 0.023 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| state | 0.004 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.032 | 0.76 |
| both | 0.002 | 1.000 | 0.000 | 0.000 | 0.00 | 1.000 | 1.000 | 0.032 | 0.76 |

Compute time per call, p50 / worst p99 (µs):

| Module | none | perception | state | both |
|---|---|---|---|---|
| controller | 510 / 839 | 541 / 845 | 560 / 841 | 521 / 845 |
| estimation_predict | 0 / 1 | 0 / 1 | – | – |
| estimation_update | 199 / 1373 | 213 / 1606 | – | – |
| exploration | 110 / 245 | 126 / 251 | 128 / 255 | 129 / 247 |
| perception | 390 / 627 | – | 415 / 642 | – |
| stack_control | 510 / 840 | 540 / 848 | 560 / 842 | 521 / 845 |
| stack_scan | 630 / 1986 | 224 / 1768 | 462 / 779 | 11 / 228 |

## Ablation: sensor noise

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| noise-0 | 12/12 | 35.96 | 23.01 | 14.58 | – | 0.00 | 0.00 | 0.0 |
| noise-0.5 | 12/12 | 35.94 | 23.01 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| noise-1 | 12/12 | 35.93 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| noise-2 | 12/12 | 35.93 | 23.03 | 14.55 | – | 0.00 | 0.00 | 0.0 |
| noise-4 | 12/12 | 35.89 | 23.01 | 14.60 | – | 0.42 | 0.00 | 0.8 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| noise-0 | 0.007 | 1.000 | 0.023 | 0.014 | 0.01 | 1.000 | 0.994 | 0.031 | 0.77 |
| noise-0.5 | 0.011 | 1.000 | 0.025 | 0.016 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| noise-1 | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| noise-2 | 0.038 | 1.000 | 0.082 | 0.048 | 0.05 | 1.000 | 0.991 | 0.030 | 0.78 |
| noise-4 | 0.137 | 0.998 | 0.186 | 0.159 | 0.18 | 1.000 | 0.799 | 0.039 | 1.06 |

Compute time per call, p50 / worst p99 (µs):

| Module | noise-0 | noise-0.5 | noise-1 | noise-2 | noise-4 |
|---|---|---|---|---|---|
| controller | 573 / 891 | 562 / 841 | 562 / 841 | 565 / 847 | 527 / 842 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 212 / 1501 | 210 / 1556 | 210 / 1519 | 199 / 1434 | 131 / 1149 |
| exploration | 126 / 246 | 131 / 254 | 130 / 250 | 131 / 260 | 129 / 255 |
| perception | 383 / 576 | 407 / 599 | 420 / 627 | 383 / 569 | 352 / 516 |
| stack_control | 573 / 891 | 562 / 842 | 562 / 842 | 564 / 848 | 526 / 842 |
| stack_scan | 646 / 2148 | 670 / 2254 | 679 / 2210 | 640 / 2094 | 525 / 1763 |

## Ablation: processing and actuator latency

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| nominal | 12/12 | 35.93 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| processing-2x | 12/12 | 36.30 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| processing-4x | 12/12 | 36.32 | 23.00 | 14.57 | – | 0.00 | 0.00 | 0.0 |
| actuators-2x | 12/12 | 34.76 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| actuators-2x-unmodeled | 12/12 | 35.87 | 23.01 | 14.57 | – | 0.00 | 0.00 | 0.0 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| nominal | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| processing-2x | 0.023 | 1.000 | 0.038 | 0.028 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| processing-4x | 0.017 | 1.000 | 0.038 | 0.027 | 0.03 | 1.000 | 0.994 | 0.030 | 0.78 |
| actuators-2x | 0.021 | 1.000 | 0.043 | 0.026 | 0.02 | 1.000 | 0.994 | 0.031 | 0.78 |
| actuators-2x-unmodeled | 0.020 | 1.000 | 0.054 | 0.025 | 0.02 | 1.000 | 0.994 | 0.054 | 0.78 |

Compute time per call, p50 / worst p99 (µs):

| Module | nominal | processing-2x | processing-4x | actuators-2x | actuators-2x-unmodeled |
|---|---|---|---|---|---|
| controller | 553 / 842 | 558 / 841 | 560 / 853 | 561 / 846 | 532 / 840 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 200 / 1579 | 206 / 1488 | 207 / 1543 | 204 / 1533 | 193 / 1535 |
| exploration | 124 / 255 | 132 / 262 | 130 / 273 | 131 / 250 | 131 / 263 |
| perception | 416 / 630 | 419 / 636 | 420 / 642 | 419 / 631 | 401 / 632 |
| stack_control | 552 / 842 | 558 / 842 | 560 / 853 | 561 / 847 | 532 / 841 |
| stack_scan | 660 / 2285 | 673 / 2224 | 673 / 2231 | 671 / 2196 | 633 / 2234 |

## Ablation: friction mismatch (the planner keeps the nominal tires' mu)

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| grip-110 | 12/12 | 35.86 | 22.95 | 14.53 | – | 0.00 | 0.00 | 0.0 |
| grip-100 | 12/12 | 35.93 | 23.00 | 14.56 | – | 0.00 | 0.00 | 0.0 |
| grip-90 | 12/12 | 35.98 | 23.08 | 14.62 | – | 0.00 | 0.00 | 0.0 |
| grip-80 | 12/12 | 36.00 | 23.20 | 14.70 | – | 0.00 | 0.00 | 0.0 |
| grip-70 | 6/12 | 35.79 | 24.71 | 15.93 | – | 25.50 | 1.83 | 69.3 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| grip-110 | 0.021 | 1.000 | 0.067 | 0.028 | 0.03 | 1.000 | 0.994 | 0.033 | 0.82 |
| grip-100 | 0.018 | 1.000 | 0.035 | 0.024 | 0.02 | 1.000 | 0.994 | 0.031 | 0.77 |
| grip-90 | 0.017 | 1.000 | 0.050 | 0.024 | 0.02 | 1.000 | 0.994 | 0.052 | 0.75 |
| grip-80 | 0.024 | 1.000 | 0.158 | 0.038 | 0.04 | 1.000 | 0.994 | 0.109 | 0.80 |
| grip-70 | 0.142 | 1.000 | 0.285 | 1.561 | 0.63 | 0.989 | 0.759 | 0.750 | 6.75 |

Compute time per call, p50 / worst p99 (µs):

| Module | grip-110 | grip-100 | grip-90 | grip-80 | grip-70 |
|---|---|---|---|---|---|
| controller | 558 / 838 | 559 / 841 | 560 / 844 | 556 / 841 | 525 / 1344 |
| estimation_predict | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 | 0 / 1 |
| estimation_update | 198 / 1504 | 199 / 1502 | 200 / 1505 | 199 / 1485 | 174 / 1941 |
| exploration | 123 / 244 | 130 / 257 | 130 / 254 | 131 / 249 | 130 / 251 |
| perception | 418 / 627 | 419 / 648 | 419 / 636 | 420 / 638 | 398 / 669 |
| stack_control | 558 / 838 | 559 / 841 | 560 / 845 | 556 / 841 | 524 / 1346 |
| stack_scan | 660 / 2175 | 665 / 2191 | 667 / 2175 | 665 / 2184 | 630 / 2602 |

Runs that did not finish:

- grip-70: off the track by 5.0 m (2 runs)
- grip-70: stalled for 10 s (4 runs)
