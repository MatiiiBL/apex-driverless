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
