# Estimation results

Produced by `build/release/bench/estimation/apex_estimation_bench` (docs/estimation.md explains the setup). Lap 1 maps, laps 2-3 localize against the frozen map. `world+perception`: the simulator's vehicle, IMU, wheel and steering models, and its LiDAR and camera through ConePerception, with the pure-pursuit test driver (racing at 10 m/s^2 lateral); `world`: the same, but cone observations made from the true cones by the synthetic perception model; `synthetic`: the kinematic drive at full racing speed (22 m/s, 14 m/s^2) with the synthetic perception. Every drive is estimated twice: with the SLAM's yaw-rate bias state (bias on, the default) and without it (off).

| source | seed | bias | cones | map RMS (m) | batch RMS (m) | dup | missed | pose RMS lap 1 (m) | pose RMS racing (m) | yaw RMS racing (deg) | pose NEES lap 1 / racing | assoc. precision | recall | velocity NEES | recoveries |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| world+perception | 1 | on | 203 | 0.019 | 0.020 | 0 | 0 | 0.015 | 0.030 | 0.02 | 0.59 / 2.60 | 1.0000 | 0.9882 | 2.82 | 2 |
| world+perception | 1 | off | 203 | 0.019 | 0.020 | 0 | 0 | 0.013 | 0.030 | 0.02 | 0.63 / 2.74 | 1.0000 | 0.9883 | 2.82 | 2 |
| world+perception | 2 | on | 180 | 0.017 | 0.016 | 0 | 0 | 0.015 | 0.039 | 0.02 | 0.59 / 7.22 | 1.0000 | 0.9785 | 1.72 | 4 |
| world+perception | 2 | off | 180 | 0.015 | 0.016 | 0 | 0 | 0.014 | 0.039 | 0.02 | 0.62 / 7.19 | 1.0000 | 0.9785 | 1.72 | 4 |
| world+perception | 3 | on | 109 | 0.013 | 0.014 | 0 | 0 | 0.014 | 0.043 | 0.02 | 0.72 / 2.93 | 1.0000 | 0.9726 | 1.71 | 2 |
| world+perception | 3 | off | 109 | 0.014 | 0.014 | 0 | 0 | 0.015 | 0.043 | 0.02 | 0.73 / 2.97 | 1.0000 | 0.9726 | 1.71 | 2 |
| world+perception | 4 | on | 186 | 0.009 | 0.009 | 0 | 0 | 0.014 | 0.023 | 0.02 | 0.51 / 0.67 | 1.0000 | 0.9829 | 2.08 | 1 |
| world+perception | 4 | off | 186 | 0.009 | 0.009 | 0 | 0 | 0.020 | 0.023 | 0.02 | 0.59 / 0.73 | 1.0000 | 0.9829 | 2.08 | 1 |
| world+perception | 5 | on | 183 | 0.014 | 0.014 | 0 | 0 | 0.012 | 0.013 | 0.01 | 0.47 / 0.92 | 1.0000 | 0.9949 | 2.29 | 0 |
| world+perception | 5 | off | 183 | 0.014 | 0.014 | 0 | 0 | 0.029 | 0.013 | 0.02 | 0.77 / 1.02 | 1.0000 | 0.9949 | 2.29 | 0 |
| world | 1 | on | 203 | 0.108 | 0.117 | 0 | 0 | 0.076 | 0.110 | 0.08 | 2.31 / 4.30 | 1.0000 | 0.9961 | 2.82 | 0 |
| world | 1 | off | 203 | 0.117 | 0.117 | 0 | 0 | 0.076 | 0.119 | 0.07 | 2.64 / 4.95 | 1.0000 | 0.9962 | 2.82 | 0 |
| world | 2 | on | 180 | 0.033 | 0.033 | 0 | 0 | 0.167 | 0.053 | 0.07 | 1.17 / 6.82 | 1.0000 | 0.9823 | 1.72 | 0 |
| world | 2 | off | 180 | 0.030 | 0.033 | 0 | 0 | 0.158 | 0.052 | 0.06 | 1.29 / 6.55 | 1.0000 | 0.9823 | 1.72 | 0 |
| world | 3 | on | 109 | 0.031 | 0.032 | 0 | 0 | 0.050 | 0.070 | 0.10 | 1.98 / 9.67 | 0.9999 | 0.9682 | 1.71 | 0 |
| world | 3 | off | 109 | 0.032 | 0.032 | 0 | 0 | 0.051 | 0.071 | 0.10 | 2.05 / 9.78 | 0.9999 | 0.9682 | 1.71 | 0 |
| world | 4 | on | 186 | 0.027 | 0.036 | 0 | 0 | 0.065 | 0.046 | 0.05 | 1.57 / 7.18 | 1.0000 | 0.9858 | 2.08 | 0 |
| world | 4 | off | 186 | 0.030 | 0.036 | 0 | 0 | 0.090 | 0.048 | 0.05 | 2.22 / 7.28 | 1.0000 | 0.9859 | 2.08 | 0 |
| world | 5 | on | 183 | 0.053 | 0.051 | 0 | 0 | 0.137 | 0.053 | 0.05 | 2.12 / 1.74 | 1.0000 | 0.9998 | 2.29 | 0 |
| world | 5 | off | 183 | 0.054 | 0.051 | 0 | 0 | 0.272 | 0.053 | 0.05 | 5.09 / 1.96 | 1.0000 | 0.9998 | 2.29 | 0 |
| synthetic | 1 | on | 203 | 0.054 | 0.047 | 0 | 0 | 0.037 | 0.056 | 0.08 | 0.62 / 1.75 | 1.0000 | 0.9997 | 1.80 | 0 |
| synthetic | 1 | off | 203 | 0.054 | 0.047 | 0 | 0 | 0.026 | 0.057 | 0.08 | 0.61 / 1.78 | 1.0000 | 0.9997 | 1.80 | 0 |
| synthetic | 2 | on | 180 | 0.020 | 0.018 | 0 | 0 | 0.120 | 0.021 | 0.04 | 1.57 / 1.05 | 0.9999 | 0.9995 | 1.49 | 0 |
| synthetic | 2 | off | 180 | 0.019 | 0.018 | 0 | 0 | 0.160 | 0.020 | 0.04 | 2.11 / 1.05 | 0.9999 | 0.9995 | 1.49 | 0 |
| synthetic | 3 | on | 109 | 0.037 | 0.036 | 0 | 0 | 0.038 | 0.038 | 0.07 | 0.72 / 1.78 | 1.0000 | 0.9999 | 1.78 | 0 |
| synthetic | 3 | off | 109 | 0.037 | 0.036 | 0 | 0 | 0.033 | 0.038 | 0.07 | 0.71 / 1.75 | 1.0000 | 0.9999 | 1.78 | 0 |
| synthetic | 4 | on | 186 | 0.042 | 0.043 | 0 | 0 | 0.073 | 0.047 | 0.05 | 1.17 / 1.52 | 0.9999 | 0.9996 | 1.31 | 0 |
| synthetic | 4 | off | 186 | 0.045 | 0.043 | 0 | 0 | 0.076 | 0.049 | 0.05 | 1.20 / 1.53 | 0.9999 | 0.9996 | 1.31 | 0 |
| synthetic | 5 | on | 183 | 0.040 | 0.038 | 0 | 0 | 0.023 | 0.040 | 0.07 | 0.36 / 1.51 | 1.0000 | 0.9996 | 1.84 | 0 |
| synthetic | 5 | off | 183 | 0.039 | 0.038 | 0 | 0 | 0.018 | 0.039 | 0.07 | 0.36 / 1.50 | 1.0000 | 0.9996 | 1.84 | 0 |

Pose NEES is 3-D (expected 3); velocity NEES over (vx, vy, yaw rate), expected 3.

## Latency against landmark count

One cone update (association and EKF update) and one IMU prediction, wall clock, Release build. A synthetic drive maps an oval of 388 cones at 10 m/s.

| landmarks | obs/scan | update p50 (ms) | update p99 (ms) | predict p50 (us) | predict p99 (us) |
|---|---|---|---|---|---|
| 0-50 | 9.2 | 0.022 | 0.045 | 0.17 | 0.21 |
| 50-100 | 8.9 | 0.061 | 0.106 | 0.17 | 0.17 |
| 100-150 | 8.8 | 0.117 | 0.173 | 0.17 | 0.17 |
| 150-200 | 8.8 | 0.206 | 0.280 | 0.17 | 0.21 |
| 200-250 | 8.7 | 0.296 | 0.426 | 0.17 | 0.21 |
| 250-300 | 8.8 | 0.448 | 0.631 | 0.17 | 0.21 |
| 300-350 | 8.9 | 0.640 | 0.980 | 0.17 | 0.25 |
| 350-400 | 9.0 | 0.825 | 1.209 | 0.17 | 0.25 |
