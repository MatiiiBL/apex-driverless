## Noise 3 with a 13 m exploration start radius

| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle (s) | Cones / run | Off-course / run | Penalties (s / run) |
|---|---|---|---|---|---|---|---|---|
| range-30 | 7/12 | 34.02 | 23.14 | 14.04 | – | 31.58 | 18.17 | 244.8 |
| range-20 | 6/12 | 33.76 | 21.95 | 13.97 | – | 33.42 | 16.00 | 226.8 |

| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall | Tracking RMS, racing (m) | Max offset from centerline (m) |
|---|---|---|---|---|---|---|---|---|---|
| range-30 | 0.070 | 0.961 | 0.129 | 0.149 | 2.80 | 1.000 | 0.948 | 0.358 | 6.75 |
| range-20 | 0.181 | 0.995 | 0.282 | 0.749 | 1.59 | 0.999 | 0.912 | 0.945 | 6.75 |

Compute time per call, p50 / worst p99 (µs):

| Module | range-30 | range-20 |
|---|---|---|
| controller | 404 / 1372 | 443 / 1304 |
| estimation_predict | 0 / 1 | 0 / 1 |
| estimation_update | 175 / 1764 | 154 / 1415 |
| exploration | 78 / 245 | 70 / 190 |
| perception | 279 / 777 | 298 / 605 |
| stack_control | 404 / 1372 | 443 / 1330 |
| stack_scan | 495 / 2356 | 506 / 1994 |

Runs that did not finish:

- range-30: off the track by 5.0 m (4 runs)
- range-30: stalled for 10 s (1 run)
- range-20: mission failed: no exploration path for 3.02 s (1 run)
- range-20: off the track by 5.0 m (4 runs)
- range-20: stalled for 10 s (1 run)
