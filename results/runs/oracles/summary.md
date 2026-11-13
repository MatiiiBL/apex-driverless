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
