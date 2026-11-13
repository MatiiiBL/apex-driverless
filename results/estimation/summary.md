# Estimation results

Produced by `build/release/bench/estimation/apex_estimation_bench` (docs/estimation.md explains the setup). Lap 1 maps, laps 2-3 localize against the frozen map. `world+perception`: the simulator's vehicle, IMU, wheel and steering models, and its LiDAR and camera through ConePerception, with the pure-pursuit test driver (racing at 10 m/s^2 lateral); `world`: the same, but cone observations made from the true cones by the synthetic perception model; `synthetic`: the kinematic drive at full racing speed (22 m/s, 14 m/s^2) with the synthetic perception.

| source | seed | cones | map RMS (m) | batch RMS (m) | dup | missed | pose RMS lap 1 (m) | pose RMS racing (m) | yaw RMS racing (deg) | pose NEES lap 1 / racing | assoc. precision | recall | velocity NEES | recoveries |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| world+perception | 1 | 203 | 0.030 | 0.020 | 0 | 0 | 0.042 | 0.038 | 0.02 | 1.11 / 2.72 | 1.0000 | 0.9863 | 2.82 | 2 |
| world+perception | 2 | 180 | 0.010 | 0.016 | 0 | 0 | 0.040 | 0.038 | 0.02 | 0.63 / 6.11 | 1.0000 | 0.9749 | 1.72 | 4 |
| world+perception | 3 | 109 | 0.028 | 0.014 | 0 | 0 | 0.091 | 0.050 | 0.04 | 2.16 / 3.35 | 1.0000 | 0.9684 | 1.71 | 2 |
| world+perception | 4 | 186 | 0.039 | 0.009 | 0 | 0 | 0.086 | 0.041 | 0.02 | 1.22 / 1.23 | 1.0000 | 0.9798 | 2.08 | 1 |
| world+perception | 5 | 183 | 0.016 | 0.014 | 0 | 0 | 0.146 | 0.015 | 0.02 | 3.29 / 1.00 | 1.0000 | 0.9940 | 2.29 | 0 |
| world | 1 | 203 | 0.096 | 0.117 | 0 | 0 | 0.099 | 0.097 | 0.06 | 2.06 / 3.55 | 1.0000 | 0.9956 | 2.82 | 0 |
| world | 2 | 180 | 0.105 | 0.033 | 0 | 0 | 0.123 | 0.118 | 0.10 | 2.10 / 7.55 | 1.0000 | 0.9805 | 1.72 | 0 |
| world | 3 | 109 | 0.039 | 0.032 | 0 | 0 | 0.058 | 0.075 | 0.10 | 1.32 / 9.04 | 0.9998 | 0.9649 | 1.71 | 0 |
| world | 4 | 186 | 0.028 | 0.036 | 0 | 0 | 0.156 | 0.048 | 0.06 | 2.44 / 7.35 | 1.0000 | 0.9847 | 2.08 | 0 |
| world | 5 | 183 | 0.942 | 0.051 | 16 | 0 | 0.681 | 1.133 | 1.30 | 62.02 / 414.65 | 0.9999 | 0.9189 | 2.29 | 2 |
| synthetic | 1 | 203 | 0.057 | 0.047 | 0 | 0 | 0.101 | 0.060 | 0.07 | 0.75 / 1.44 | 1.0000 | 0.9998 | 1.80 | 0 |
| synthetic | 2 | 180 | 0.027 | 0.018 | 0 | 0 | 0.374 | 0.028 | 0.05 | 3.32 / 1.47 | 0.9999 | 0.9996 | 1.49 | 0 |
| synthetic | 3 | 109 | 0.077 | 0.036 | 0 | 0 | 0.043 | 0.077 | 0.11 | 1.14 / 2.29 | 1.0000 | 0.9998 | 1.78 | 0 |
| synthetic | 4 | 186 | 0.060 | 0.043 | 0 | 0 | 0.110 | 0.066 | 0.06 | 0.88 / 1.62 | 0.9999 | 0.9996 | 1.31 | 0 |
| synthetic | 5 | 183 | 0.122 | 0.038 | 0 | 0 | 0.138 | 0.122 | 0.12 | 1.17 / 2.43 | 1.0000 | 0.9996 | 1.84 | 0 |

Pose NEES is 3-D (expected 3); velocity NEES over (vx, vy, yaw rate), expected 3.

## Latency against landmark count

One cone update (association and EKF update) and one IMU prediction, wall clock, Release build. A synthetic drive maps an oval of 388 cones at 10 m/s.

| landmarks | obs/scan | update p50 (ms) | update p99 (ms) | predict p50 (us) | predict p99 (us) |
|---|---|---|---|---|---|
| 0-50 | 9.2 | 0.056 | 0.246 | 0.42 | 1.29 |
| 50-100 | 8.9 | 0.198 | 0.589 | 0.42 | 2.29 |
| 100-150 | 8.8 | 0.361 | 1.917 | 0.42 | 3.12 |
| 150-200 | 8.8 | 0.687 | 2.336 | 0.42 | 3.71 |
| 200-250 | 8.7 | 1.131 | 2.793 | 0.42 | 4.25 |
| 250-300 | 8.8 | 1.661 | 3.609 | 0.42 | 4.38 |
| 300-350 | 8.9 | 2.079 | 5.068 | 0.42 | 4.96 |
| 350-400 | 9.0 | 2.900 | 5.497 | 0.42 | 5.08 |
