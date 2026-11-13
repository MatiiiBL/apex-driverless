# MPC QP: Riccati interior point vs OSQP

Every QP of the MPC over 1000 calls on the 'mixed' track, per horizon N (stages of 50 ms), absolute tolerance 1e-06. Times per QP in microseconds; OSQP cold and warm include its setup (the KKT factorization), which an LTV MPC pays at every call. Max diff: largest difference between the two solvers' states and inputs.

| N | ours cold p50 / p99 (us) | iters | ours warm p50 / p99 | iters | OSQP cold p50 / p99 | iters | OSQP warm p50 / p99 | iters | OSQP solve-only p50 | max diff |
|---|---|---|---|---|---|---|---|---|---|---|
| 20 | 293 / 1683 | 7.4 | 145 / 792 | 3.7 | 1856 / 6726 | 141.7 | 1321 / 5669 | 92.8 | 1589 | 3.8e-03 |
| 30 | 465 / 1716 | 7.9 | 225 / 1304 | 3.9 | 2747 / 8760 | 140.2 | 1990 / 6944 | 95.1 | 2382 | 5.7e-03 |
| 40 | 715 / 4103 | 8.3 | 363 / 2314 | 4.1 | 4240 / 23264 | 149.1 | 2937 / 21912 | 96.1 | 3667 | 3.6e-03 |
| 50 | 1015 / 11315 | 8.5 | 493 / 8031 | 4.3 | 5976 / 74545 | 148.9 | 4405 / 42745 | 99.8 | 5159 | 4.9e-03 |
| 60 | 1000 / 3169 | 8.6 | 533 / 1595 | 4.6 | 5594 / 11451 | 152.0 | 4090 / 9413 | 100.9 | 4903 | 4.9e-03 |
