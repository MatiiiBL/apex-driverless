# Closed-loop evaluation

Simulator vehicle, true state as the estimate, reference = centerline with a speed profile at 85% of the nominal static grip; two laps from a standing start, metrics over the flying second lap. Compute time per call, one core.

## Tracks

| track | controller | flying lap (s) | profile (s) | lateral RMS / max (m) | steer rate RMS (rad/s) | outside bounds (s) | cones | off course | compute p50 / p99 (us) | CPU p50 / p99 (us) | QP failures |
|---|---|---|---|---|---|---|---|---|---|---|---|
| technical | pure_pursuit | 23.08 | 23.01 | 0.072 / 0.15 | 0.24 | 0.00 | 0 | 0 | 16 / 18 | 16 / 19 | 0 |
| flowing | pure_pursuit | 27.19 | 27.11 | 0.065 / 0.16 | 0.20 | 0.00 | 0 | 0 | 16 / 24 | 16 / 25 | 0 |
| mixed | pure_pursuit | 27.52 | 27.43 | 0.067 / 0.16 | 0.21 | 0.00 | 0 | 0 | 16 / 24 | 16 / 25 | 0 |
| seed 1 | pure_pursuit | 30.57 | 30.48 | 0.076 / 0.17 | 0.26 | 0.00 | 0 | 0 | 16 / 27 | 17 / 27 | 0 |
| seed 2 | pure_pursuit | 26.49 | 26.40 | 0.068 / 0.16 | 0.24 | 0.00 | 0 | 0 | 16 / 24 | 16 / 25 | 0 |
| seed 3 | pure_pursuit | 15.97 | 15.93 | 0.073 / 0.20 | 0.26 | 0.00 | 0 | 0 | 16 / 24 | 17 / 26 | 0 |
| seed 4 | pure_pursuit | 27.14 | 27.05 | 0.061 / 0.16 | 0.20 | 0.00 | 0 | 0 | 16 / 24 | 17 / 25 | 0 |
| seed 5 | pure_pursuit | 27.49 | 27.41 | 0.068 / 0.19 | 0.24 | 0.00 | 0 | 0 | 16 / 25 | 17 / 26 | 0 |
| technical | stanley | 22.96 | 23.01 | 0.105 / 0.25 | 0.28 | 0.00 | 0 | 0 | 9 / 13 | 9 / 14 | 0 |
| flowing | stanley | 27.14 | 27.11 | 0.076 / 0.19 | 0.23 | 0.00 | 0 | 0 | 8 / 13 | 9 / 14 | 0 |
| mixed | stanley | 27.43 | 27.43 | 0.090 / 0.22 | 0.27 | 0.00 | 0 | 0 | 8 / 13 | 9 / 14 | 0 |
| seed 1 | stanley | 30.46 | 30.48 | 0.096 / 0.22 | 0.33 | 0.00 | 0 | 0 | 8 / 13 | 9 / 14 | 0 |
| seed 2 | stanley | 26.41 | 26.40 | 0.086 / 0.22 | 0.26 | 0.00 | 0 | 0 | 8 / 13 | 9 / 14 | 0 |
| seed 3 | stanley | 15.92 | 15.93 | 0.090 / 0.20 | 0.34 | 0.00 | 0 | 0 | 8 / 14 | 9 / 15 | 0 |
| seed 4 | stanley | 27.06 | 27.05 | 0.088 / 0.21 | 0.22 | 0.00 | 0 | 0 | 8 / 13 | 9 / 14 | 0 |
| seed 5 | stanley | 27.40 | 27.41 | 0.090 / 0.22 | 0.33 | 0.00 | 0 | 0 | 8 / 13 | 9 / 14 | 0 |
| technical | mpc | 23.02 | 23.01 | 0.020 / 0.07 | 0.22 | 0.00 | 0 | 0 | 326 / 632 | 326 / 628 | 0 |
| flowing | mpc | 27.12 | 27.11 | 0.017 / 0.05 | 0.18 | 0.00 | 0 | 0 | 319 / 632 | 319 / 632 | 0 |
| mixed | mpc | 27.43 | 27.43 | 0.018 / 0.06 | 0.21 | 0.00 | 0 | 0 | 320 / 643 | 321 / 640 | 0 |
| seed 1 | mpc | 30.50 | 30.48 | 0.019 / 0.06 | 0.24 | 0.00 | 0 | 0 | 322 / 634 | 323 / 634 | 0 |
| seed 2 | mpc | 26.41 | 26.40 | 0.019 / 0.06 | 0.21 | 0.00 | 0 | 0 | 322 / 620 | 323 / 616 | 0 |
| seed 3 | mpc | 15.93 | 15.93 | 0.021 / 0.06 | 0.25 | 0.00 | 0 | 0 | 328 / 666 | 329 / 666 | 0 |
| seed 4 | mpc | 27.06 | 27.05 | 0.016 / 0.05 | 0.19 | 0.00 | 0 | 0 | 322 / 624 | 322 / 623 | 0 |
| seed 5 | mpc | 27.42 | 27.41 | 0.019 / 0.07 | 0.23 | 0.00 | 0 | 0 | 323 / 638 | 323 / 638 | 0 |

## Robustness (means over the three reference layouts; the controllers keep the nominal parameters)

| steer dead time (s) | controller | finished | mean flying lap (s) | mean lateral RMS (m) | worst lateral max (m) | penalties (s) |
|---|---|---|---|---|---|---|
| 0.06 | mpc | 3/3 | 25.91 | 0.056 | 0.15 | 0 |
| 0.06 | pure_pursuit | 3/3 | 25.94 | 0.050 | 0.16 | 0 |
| 0.06 | stanley | 3/3 | 25.87 | 0.067 | 0.20 | 0 |
| 0.09 | mpc | 3/3 | 26.02 | 0.124 | 0.35 | 0 |
| 0.09 | pure_pursuit | 3/3 | 25.96 | 0.042 | 0.17 | 0 |
| 0.09 | stanley | 2/3 | 25.29 | 0.076 | 0.44 | 0 |

| friction scale | controller | finished | mean flying lap (s) | mean lateral RMS (m) | worst lateral max (m) | penalties (s) |
|---|---|---|---|---|---|---|
| 0.8 | mpc | 3/3 | 26.29 | 0.191 | 0.54 | 0 |
| 0.8 | pure_pursuit | 3/3 | 26.13 | 0.284 | 2.79 | 34 |
| 0.8 | stanley | 3/3 | 26.02 | 0.156 | 0.48 | 0 |
| 0.9 | mpc | 3/3 | 25.96 | 0.055 | 0.12 | 0 |
| 0.9 | pure_pursuit | 3/3 | 25.98 | 0.081 | 0.33 | 0 |
| 0.9 | stanley | 3/3 | 25.90 | 0.095 | 0.23 | 0 |

Model mismatch 1: mass and yaw inertia +20%. 2: tire stiffness factor B -25%.

| model mismatch | controller | finished | mean flying lap (s) | mean lateral RMS (m) | worst lateral max (m) | penalties (s) |
|---|---|---|---|---|---|---|
| 1 | mpc | 3/3 | 25.81 | 0.021 | 0.07 | 0 |
| 1 | pure_pursuit | 3/3 | 25.95 | 0.069 | 0.18 | 0 |
| 1 | stanley | 3/3 | 25.86 | 0.091 | 0.25 | 0 |
| 2 | mpc | 3/3 | 25.99 | 0.067 | 0.14 | 0 |
| 2 | pure_pursuit | 3/3 | 25.99 | 0.082 | 0.31 | 0 |
| 2 | stanley | 3/3 | 25.94 | 0.107 | 0.27 | 0 |
