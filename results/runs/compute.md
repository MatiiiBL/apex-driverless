# Compute

Measured once at commit f8037da, after the evaluation's sweeps, with nothing else running on the
machine (load average 1.2 at the start): an Apple M4 (4 performance and 6 efficiency cores,
16 GB), macOS 27.0.1, Apple clang 21.0.0, the Release preset (`-O3 -DNDEBUG`). The programs ran
one after the other, each once, in 1 min 41 s in all; each is single-threaded except the
scaling part of `apex_runner_bench`. Times are wall-clock per call (`Stopwatch` and
`LatencyStats`, docs/design.md, section 6).

```
make PRESET=release build
build/release/bench/runner/apex_runner_bench          # 3 laps with the MPC on each reference layout
build/release/bench/sim/apex_sim_bench --out sim.json
build/release/bench/perception/apex_perception_eval --out perception
build/release/bench/estimation/apex_estimation_bench --out estimation
build/release/bench/planning/apex_planning_exploration_eval --out exploration.json
build/release/bench/planning/apex_planning_raceline_eval --out raceline.json
build/release/bench/planning/apex_planning_qp_bench --out planning_qp.json
build/release/bench/control/apex_control_closed_loop_eval --out control
build/release/bench/control/apex_control_qp_bench --out control
```

The module benchmarks wrote to a scratch directory; the modules' own results files are
unchanged.

## Against the budgets of docs/design.md, section 6

| Work | Budget (p99) | Module benchmark, p50 / p99 | Full stack, p50 / p99 |
|---|---|---|---|
| Perception, one LiDAR scan | 10 ms | 198 / 244 µs | 202 / 269 µs |
| Estimation, one cone update (200 landmarks) | 5 ms | 294 / 423 µs | 117 / 567 µs |
| Estimation, one IMU prediction | 0.05 ms | 0.2 / 0.2 µs | 0.1 / 0.2 µs |
| Exploration plan | 10 ms | 38 / 83 µs | 54 / 90 µs |
| Raceline optimization (once) | 2 s | 22 ms (p50), 50 ms (max) | 13 to 38 ms |
| Controller (MPC), one command | 2 ms | 253 / 325 µs | 252 / 341 µs |
| Simulator, one tick with sensors | 10× real time over a run | 165× real time | 43.6× real time, stack included |

Every module is within its budget at p99; the tightest is the MPC, at 341 µs against 2 ms.

**Module benchmark**, each module's own program:

- Perception: `on_lidar` over the 1 204 scans of `apex_perception_eval` (8 generated tracks).
- Estimation: `apex_estimation_bench`'s cone updates while the map holds 200 to 250 landmarks
  (124 updates, 8.7 observations each); the prediction over every map size.
- Exploration: `plan()` 1 442 times on 20 generated tracks with nominal maps
  (`apex_planning_exploration_eval`; 34 / 70 µs with the worst maps, 48 / 83 µs mapping out to
  30 m).
- Raceline: 60 optimizations, on 20 tracks with the true track and two map levels
  (`apex_planning_raceline_eval`).
- Controller: the MPC's eight base runs of `apex_control_closed_loop_eval` (median p50, highest
  p99); Stanley 6.9 / 7.3 µs and pure pursuit 13.2 / 14.5 µs the same way.
- Simulator: one lap of `generated-1` with every sensor (`apex_sim_bench`): tick p50 1.0 µs,
  p99 12 µs, max 0.52 ms; a LiDAR scan (57 664 rays) p50 0.47 ms; a camera frame 5 µs; a vehicle
  model step 0.35 µs.

**Full stack**: `apex_runner_bench`, three laps with the MPC on each reference layout (87 to
97 s simulated per run), the highest p50 and p99 of the three runs; the raceline is optimized
once per run.

| Layout | Real-time factor | Simulator tick (mean) | Perception p50 / p99 | Cone update p50 / p99 | Exploration p50 / p99 | MPC p50 / p99 | Whole scan p50 / p99 |
|---|---|---|---|---|---|---|---|
| technical | 43.6× | 5.8 µs | 202 / 269 µs | 117 / 517 µs | 54 / 90 µs | 252 / 341 µs | 338 / 789 µs |
| flowing | 43.6× | 5.8 µs | 190 / 236 µs | 86 / 457 µs | 47 / 72 µs | 252 / 315 µs | 294 / 708 µs |
| mixed | 43.8× | 5.8 µs | 194 / 238 µs | 89 / 567 µs | 49 / 75 µs | 252 / 328 µs | 306 / 799 µs |

A whole scan (perception, estimation and planning together) costs at most 0.8 ms against the
LiDAR's 100 ms period, and a whole control step 342 µs against 20 ms. In the stack a cone update
carries about 19 observations (16 203 in 870 scans on technical) against 8.7 in the estimation
benchmark: the likely reason its p99 is higher at a similar map size.
The sweep pool runs the three layouts in 6.45 s on one thread, 4.23 s on two (1.53×, with three
runs for two threads) and 2.29 s on three (2.82×).

The evaluation's sweeps time every module too (results/runs/README.md), but with eight runs at
once: their p99 are 2.2 to 2.4 times those above, and are not used.

## Against the modules' own results

| Module | Here | The module's results file | Difference |
|---|---|---|---|
| sim | world 165× real time, tick p99 12 µs; LiDAR scan p50 0.47 ms; vehicle step 0.35 µs; camera frame p50 5 µs | 42.7×, 1.09 ms; 1.54 ms; 1.2 µs; 15 µs (results/sim/bench.json) | 3 to 4 times slower in the file, recorded at a load average of about 130 (results/sim/README.md) |
| perception | `on_lidar` 198 / 244 µs, max 310 µs | 215 / 484 µs, max 1 154 µs (results/perception/summary.md) | p99 twice as high in the file, recorded with five other workers running (docs/perception.md) |
| estimation | update at 200-250 landmarks 294 / 423 µs; prediction 0.2 µs | 296 / 426 µs; 0.2 µs (results/estimation/latency.json) | none |
| planning, exploration | 38 / 83 µs | 38 / 83 µs (results/planning/exploration.json) | none |
| planning, raceline | 22 ms p50, 50 ms max | 32 ms, 97 ms (results/planning/raceline.json) | 1.5 to 2 times slower in the file, measured on a shared machine (results/planning/README.md) |
| planning, QP | raceline QPs, median: 7.0 ms, OSQP 99 ms | 10.0 ms, 129 ms (results/planning/qp.json) | 1.4 times slower in the file; the same ratio to OSQP |
| control, closed loop | MPC 253 / 325 µs; Stanley 6.9 / 7.3 µs; pure pursuit 13.2 / 14.5 µs | 263 / 579 µs; 9.9 / 14.0 µs; 22.8 / 34.1 µs (results/control/closed_loop.md) | p99 up to 1.8 times higher in the file, and Stanley's and pure pursuit's p50 1.4 to 1.7 times |
| control, QP | horizon 30, warm-started: 148 / 194 µs | 163 / 422 µs (results/control/qp_bench.md) | p99 twice as high in the file |

Where a file is slower, it is mostly in the tails; the sim, perception and planning files were
recorded on a machine shared with other work, as their notes say. None of the differences
changes a conclusion against the budgets.
