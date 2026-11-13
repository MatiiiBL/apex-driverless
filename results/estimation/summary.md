# Estimation results

Produced by `build/release/bench/estimation/apex_estimation_bench` (docs/estimation.md explains the setup). Lap 1 maps, laps 2-3 localize against the frozen map. `world+perception`: the simulator's vehicle, IMU, wheel and steering models, and its LiDAR and camera through ConePerception, with the pure-pursuit test driver (racing at 10 m/s^2 lateral); `world`: the same, but cone observations made from the true cones by the synthetic perception model; `synthetic`: the kinematic drive at full racing speed (22 m/s, 14 m/s^2) with the synthetic perception. Every drive is estimated twice: with the SLAM's yaw-rate bias state (bias on, the default) and without it (off).

| source | seed | bias | cones | map RMS (m) | batch RMS (m) | dup | missed | pose RMS lap 1 (m) | pose RMS racing (m) | yaw RMS racing (deg) | pose NEES lap 1 / racing | assoc. precision | recall | velocity NEES | recoveries |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| world+perception | 1 | on | 203 | 0.028 | 0.020 | 0 | 0 | 0.023 | 0.037 | 0.02 | 0.83 / 2.57 | 1.0000 | 0.9863 | 2.82 | 2 |
| world+perception | 1 | off | 203 | 0.030 | 0.020 | 0 | 0 | 0.042 | 0.038 | 0.02 | 1.11 / 2.72 | 1.0000 | 0.9863 | 2.82 | 2 |
| world+perception | 2 | on | 180 | 0.008 | 0.016 | 0 | 0 | 0.033 | 0.037 | 0.02 | 0.51 / 6.10 | 1.0000 | 0.9750 | 1.72 | 4 |
| world+perception | 2 | off | 180 | 0.010 | 0.016 | 0 | 0 | 0.040 | 0.038 | 0.02 | 0.63 / 6.11 | 1.0000 | 0.9749 | 1.72 | 4 |
| world+perception | 3 | on | 109 | 0.027 | 0.014 | 0 | 0 | 0.092 | 0.051 | 0.04 | 2.07 / 3.61 | 1.0000 | 0.9679 | 1.71 | 2 |
| world+perception | 3 | off | 109 | 0.028 | 0.014 | 0 | 0 | 0.091 | 0.050 | 0.04 | 2.16 / 3.35 | 1.0000 | 0.9684 | 1.71 | 2 |
| world+perception | 4 | on | 186 | 0.037 | 0.009 | 0 | 0 | 0.062 | 0.040 | 0.03 | 0.87 / 1.17 | 1.0000 | 0.9798 | 2.08 | 1 |
| world+perception | 4 | off | 186 | 0.039 | 0.009 | 0 | 0 | 0.086 | 0.041 | 0.02 | 1.22 / 1.23 | 1.0000 | 0.9798 | 2.08 | 1 |
| world+perception | 5 | on | 183 | 0.016 | 0.014 | 0 | 0 | 0.066 | 0.015 | 0.02 | 1.56 / 0.95 | 1.0000 | 0.9940 | 2.29 | 0 |
| world+perception | 5 | off | 183 | 0.016 | 0.014 | 0 | 0 | 0.146 | 0.015 | 0.02 | 3.29 / 1.00 | 1.0000 | 0.9940 | 2.29 | 0 |
| world | 1 | on | 203 | 0.084 | 0.117 | 0 | 0 | 0.115 | 0.086 | 0.07 | 1.67 / 2.94 | 1.0000 | 0.9956 | 2.82 | 0 |
| world | 1 | off | 203 | 0.096 | 0.117 | 0 | 0 | 0.099 | 0.097 | 0.06 | 2.06 / 3.55 | 1.0000 | 0.9956 | 2.82 | 0 |
| world | 2 | on | 180 | 0.093 | 0.033 | 0 | 0 | 0.126 | 0.107 | 0.10 | 1.89 / 7.57 | 1.0000 | 0.9802 | 1.72 | 0 |
| world | 2 | off | 180 | 0.105 | 0.033 | 0 | 0 | 0.123 | 0.118 | 0.10 | 2.10 / 7.55 | 1.0000 | 0.9805 | 1.72 | 0 |
| world | 3 | on | 109 | 0.036 | 0.032 | 0 | 0 | 0.054 | 0.073 | 0.10 | 1.25 / 8.84 | 0.9998 | 0.9652 | 1.71 | 0 |
| world | 3 | off | 109 | 0.039 | 0.032 | 0 | 0 | 0.058 | 0.075 | 0.10 | 1.32 / 9.04 | 0.9998 | 0.9649 | 1.71 | 0 |
| world | 4 | on | 186 | 0.025 | 0.036 | 0 | 0 | 0.101 | 0.047 | 0.06 | 1.40 / 7.26 | 1.0000 | 0.9844 | 2.08 | 0 |
| world | 4 | off | 186 | 0.028 | 0.036 | 0 | 0 | 0.156 | 0.048 | 0.06 | 2.44 / 7.35 | 1.0000 | 0.9847 | 2.08 | 0 |
| world | 5 | on | 183 | 0.036 | 0.051 | 0 | 0 | 0.389 | 0.037 | 0.04 | 5.85 / 2.15 | 1.0000 | 0.9998 | 2.29 | 0 |
| world | 5 | off | 183 | 0.942 | 0.051 | 16 | 0 | 0.681 | 1.133 | 1.30 | 62.02 / 414.65 | 0.9999 | 0.9189 | 2.29 | 2 |
| synthetic | 1 | on | 203 | 0.055 | 0.047 | 0 | 0 | 0.139 | 0.058 | 0.07 | 0.84 / 1.39 | 1.0000 | 0.9998 | 1.80 | 0 |
| synthetic | 1 | off | 203 | 0.057 | 0.047 | 0 | 0 | 0.101 | 0.060 | 0.07 | 0.75 / 1.44 | 1.0000 | 0.9998 | 1.80 | 0 |
| synthetic | 2 | on | 180 | 0.025 | 0.018 | 0 | 0 | 0.350 | 0.027 | 0.05 | 2.49 / 1.45 | 0.9999 | 0.9996 | 1.49 | 0 |
| synthetic | 2 | off | 180 | 0.027 | 0.018 | 0 | 0 | 0.374 | 0.028 | 0.05 | 3.32 / 1.47 | 0.9999 | 0.9996 | 1.49 | 0 |
| synthetic | 3 | on | 109 | 0.078 | 0.036 | 0 | 0 | 0.040 | 0.079 | 0.12 | 1.10 / 2.37 | 1.0000 | 0.9998 | 1.78 | 0 |
| synthetic | 3 | off | 109 | 0.077 | 0.036 | 0 | 0 | 0.043 | 0.077 | 0.11 | 1.14 / 2.29 | 1.0000 | 0.9998 | 1.78 | 0 |
| synthetic | 4 | on | 186 | 0.058 | 0.043 | 0 | 0 | 0.112 | 0.065 | 0.06 | 0.85 / 1.61 | 0.9999 | 0.9996 | 1.31 | 0 |
| synthetic | 4 | off | 186 | 0.060 | 0.043 | 0 | 0 | 0.110 | 0.066 | 0.06 | 0.88 / 1.62 | 0.9999 | 0.9996 | 1.31 | 0 |
| synthetic | 5 | on | 183 | 0.124 | 0.038 | 0 | 0 | 0.130 | 0.125 | 0.12 | 1.11 / 2.44 | 1.0000 | 0.9996 | 1.84 | 0 |
| synthetic | 5 | off | 183 | 0.122 | 0.038 | 0 | 0 | 0.138 | 0.122 | 0.12 | 1.17 / 2.43 | 1.0000 | 0.9996 | 1.84 | 0 |

Pose NEES is 3-D (expected 3); velocity NEES over (vx, vy, yaw rate), expected 3.

## Latency against landmark count

One cone update (association and EKF update) and one IMU prediction, wall clock, Release build. A synthetic drive maps an oval of 388 cones at 10 m/s.

| landmarks | obs/scan | update p50 (ms) | update p99 (ms) | predict p50 (us) | predict p99 (us) |
|---|---|---|---|---|---|
| 0-50 | 9.2 | 0.019 | 0.042 | 0.17 | 0.21 |
| 50-100 | 8.9 | 0.058 | 0.099 | 0.17 | 0.21 |
| 100-150 | 8.8 | 0.110 | 0.163 | 0.17 | 0.21 |
| 150-200 | 8.8 | 0.180 | 0.253 | 0.17 | 0.21 |
| 200-250 | 8.7 | 0.270 | 0.389 | 0.17 | 0.21 |
| 250-300 | 8.8 | 0.416 | 0.568 | 0.17 | 0.21 |
| 300-350 | 8.9 | 0.561 | 0.763 | 0.17 | 0.21 |
| 350-400 | 9.0 | 0.741 | 1.040 | 0.17 | 0.21 |
