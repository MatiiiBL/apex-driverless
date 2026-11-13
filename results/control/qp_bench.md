# MPC QP: Riccati interior point vs OSQP

Every QP of the MPC over 1000 calls on the 'mixed' track, per horizon N (stages of 50 ms), absolute tolerance 1e-06. Times per QP in microseconds; OSQP cold and warm include its setup (the KKT factorization), which an LTV MPC pays at every call. Max diff: largest difference between the two solvers' states and inputs.

| N | ours cold p50 / p99 (us) | iters | ours warm p50 / p99 | iters | OSQP cold p50 / p99 | iters | OSQP warm p50 / p99 | iters | OSQP solve-only p50 | max diff |
|---|---|---|---|---|---|---|---|---|---|---|
| 20 | 95 / 262 | 7.4 | 93 / 258 | 7.3 | 638 / 1372 | 141.7 | 487 / 1242 | 92.6 | 554 | 3.1e-03 |
| 30 | 185 / 462 | 7.9 | 163 / 422 | 7.4 | 1058 / 2543 | 139.9 | 813 / 1908 | 95.0 | 920 | 2.8e-03 |
| 40 | 281 / 627 | 8.3 | 234 / 585 | 7.5 | 1557 / 3771 | 149.1 | 1138 / 2482 | 96.0 | 1375 | 2.8e-03 |
| 50 | 363 / 824 | 8.5 | 318 / 749 | 7.6 | 1974 / 4916 | 149.1 | 1505 / 3128 | 99.7 | 1735 | 3.8e-03 |
| 60 | 458 / 990 | 8.6 | 398 / 915 | 7.8 | 2513 / 6393 | 152.1 | 1855 / 3777 | 101.0 | 2236 | 4.6e-03 |
