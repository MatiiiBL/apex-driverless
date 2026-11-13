# Closed-loop evaluation

Simulator vehicle, true state as the estimate, reference = centerline with a speed profile at 85% of the nominal static grip; two laps from a standing start, metrics over the flying second lap. Compute time per call, one core.

## Tracks

| track | controller | flying lap (s) | profile (s) | lateral RMS / max (m) | steer rate RMS (rad/s) | outside bounds (s) | cones | off course | compute p50 / p99 (us) | CPU p50 / p99 (us) | QP failures |
|---|---|---|---|---|---|---|---|---|---|---|---|
| technical | pure_pursuit | 23.06 | 23.01 | 0.105 / 0.24 | 0.23 | 0.00 | 0 | 0 | 23 / 34 | 24 / 31 | 0 |
| flowing | pure_pursuit | 27.18 | 27.11 | 0.089 / 0.21 | 0.19 | 0.00 | 0 | 0 | 23 / 33 | 23 / 30 | 0 |
| mixed | pure_pursuit | 27.50 | 27.43 | 0.096 / 0.23 | 0.19 | 0.00 | 0 | 0 | 23 / 28 | 23 / 29 | 0 |
| seed 1 | pure_pursuit | 30.54 | 30.48 | 0.109 / 0.26 | 0.24 | 0.00 | 0 | 0 | 23 / 27 | 23 / 27 | 0 |
| seed 2 | pure_pursuit | 26.47 | 26.40 | 0.097 / 0.22 | 0.24 | 0.00 | 0 | 0 | 23 / 24 | 23 / 27 | 0 |
| seed 3 | pure_pursuit | 15.95 | 15.93 | 0.104 / 0.27 | 0.24 | 0.00 | 0 | 0 | 23 / 27 | 23 / 28 | 0 |
| seed 4 | pure_pursuit | 27.13 | 27.05 | 0.088 / 0.23 | 0.18 | 0.00 | 0 | 0 | 23 / 27 | 23 / 27 | 0 |
| seed 5 | pure_pursuit | 27.48 | 27.41 | 0.097 / 0.28 | 0.23 | 0.00 | 0 | 0 | 23 / 29 | 24 / 30 | 0 |
| technical | stanley | 22.96 | 23.01 | 0.105 / 0.25 | 0.28 | 0.00 | 0 | 0 | 10 / 10 | 10 / 13 | 0 |
| flowing | stanley | 27.14 | 27.11 | 0.076 / 0.19 | 0.23 | 0.00 | 0 | 0 | 9 / 10 | 9 / 12 | 0 |
| mixed | stanley | 27.43 | 27.43 | 0.090 / 0.22 | 0.27 | 0.00 | 0 | 0 | 10 / 13 | 10 / 13 | 0 |
| seed 1 | stanley | 30.46 | 30.48 | 0.096 / 0.22 | 0.33 | 0.00 | 0 | 0 | 10 / 13 | 10 / 13 | 0 |
| seed 2 | stanley | 26.41 | 26.40 | 0.086 / 0.22 | 0.26 | 0.00 | 0 | 0 | 10 / 11 | 10 / 13 | 0 |
| seed 3 | stanley | 15.92 | 15.93 | 0.090 / 0.20 | 0.34 | 0.00 | 0 | 0 | 10 / 14 | 11 / 15 | 0 |
| seed 4 | stanley | 27.06 | 27.05 | 0.088 / 0.21 | 0.22 | 0.00 | 0 | 0 | 10 / 11 | 11 / 13 | 0 |
| seed 5 | stanley | 27.40 | 27.41 | 0.090 / 0.22 | 0.33 | 0.00 | 0 | 0 | 10 / 11 | 10 / 13 | 0 |
| technical | mpc | 23.07 | 23.01 | 0.032 / 0.09 | 0.22 | 0.00 | 0 | 0 | 330 / 579 | 330 / 519 | 0 |
| flowing | mpc | 27.14 | 27.11 | 0.022 / 0.07 | 0.17 | 0.00 | 0 | 0 | 295 / 363 | 296 / 364 | 0 |
| mixed | mpc | 27.47 | 27.43 | 0.028 / 0.09 | 0.20 | 0.00 | 0 | 0 | 268 / 337 | 268 / 336 | 0 |
| seed 1 | mpc | 30.53 | 30.48 | 0.025 / 0.09 | 0.24 | 0.00 | 0 | 0 | 269 / 317 | 269 / 315 | 0 |
| seed 2 | mpc | 26.43 | 26.40 | 0.025 / 0.11 | 0.21 | 0.00 | 0 | 0 | 256 / 313 | 257 / 310 | 0 |
| seed 3 | mpc | 15.95 | 15.93 | 0.029 / 0.10 | 0.24 | 0.00 | 0 | 0 | 257 / 329 | 257 / 317 | 0 |
| seed 4 | mpc | 27.09 | 27.05 | 0.021 / 0.07 | 0.19 | 0.00 | 0 | 0 | 257 / 343 | 258 / 320 | 0 |
| seed 5 | mpc | 27.46 | 27.41 | 0.029 / 0.15 | 0.22 | 0.00 | 0 | 0 | 259 / 321 | 259 / 320 | 0 |

## Robustness (means over the three reference layouts; the controllers keep the nominal parameters)

| steer dead time (s) | controller | finished | mean flying lap (s) | mean lateral RMS (m) | worst lateral max (m) | penalties (s) |
|---|---|---|---|---|---|---|
| 0.06 | mpc | 3/3 | 25.94 | 0.061 | 0.16 | 0 |
| 0.06 | pure_pursuit | 3/3 | 25.93 | 0.075 | 0.23 | 0 |
| 0.06 | stanley | 3/3 | 25.87 | 0.067 | 0.20 | 0 |
| 0.09 | mpc | 3/3 | 26.10 | 0.133 | 0.40 | 0 |
| 0.09 | pure_pursuit | 3/3 | 25.95 | 0.055 | 0.26 | 0 |
| 0.09 | stanley | 2/3 | 25.29 | 0.076 | 0.44 | 0 |

| friction scale | controller | finished | mean flying lap (s) | mean lateral RMS (m) | worst lateral max (m) | penalties (s) |
|---|---|---|---|---|---|---|
| 0.8 | mpc | 3/3 | 26.35 | 0.194 | 0.56 | 0 |
| 0.8 | pure_pursuit | 2/3 | 25.47 | 0.165 | 0.39 | 0 |
| 0.8 | stanley | 3/3 | 26.02 | 0.156 | 0.48 | 0 |
| 0.9 | mpc | 3/3 | 26.01 | 0.072 | 0.20 | 0 |
| 0.9 | pure_pursuit | 3/3 | 25.97 | 0.112 | 0.41 | 0 |
| 0.9 | stanley | 3/3 | 25.90 | 0.095 | 0.23 | 0 |

Model mismatch 1: mass and yaw inertia +20%. 2: tire stiffness factor B -25%.

| model mismatch | controller | finished | mean flying lap (s) | mean lateral RMS (m) | worst lateral max (m) | penalties (s) |
|---|---|---|---|---|---|---|
| 1 | mpc | 3/3 | 25.85 | 0.032 | 0.10 | 0 |
| 1 | pure_pursuit | 3/3 | 25.93 | 0.099 | 0.25 | 0 |
| 1 | stanley | 3/3 | 25.86 | 0.091 | 0.25 | 0 |
| 2 | mpc | 3/3 | 26.04 | 0.082 | 0.21 | 0 |
| 2 | pure_pursuit | 3/3 | 25.99 | 0.113 | 0.40 | 0 |
| 2 | stanley | 3/3 | 25.94 | 0.107 | 0.27 | 0 |
