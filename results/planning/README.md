# Planning results

Produced by the Release build on an Apple M4 (one core per run) with:

```sh
make PRESET=release build
./build/release/bench/planning/apex_planning_exploration_eval \
    --out results/planning/exploration.json --example results/planning/exploration_example.json
./build/release/bench/planning/apex_planning_raceline_eval \
    --out results/planning/raceline.json --example results/planning/raceline_example.json
./build/release/bench/planning/apex_planning_qp_bench --out results/planning/qp.json
uv run --project tools python tools/planning/plot_planning.py results/planning docs/img
```

The quality numbers are deterministic. The timings depend on the load of the machine, which
was shared with other jobs: quieter runs measured about half to two thirds of the times below,
the busiest up to five times. docs/planning.md explains the methods and the evaluations.

## Exploration (`exploration.json`)

20 generated tracks, a plan every 5 m of the lap from a pose up to 0.4 m and 0.1 rad off the
centerline, from the landmarks mapped so far (20 m range, front half-plane); 1,442 plans per
level. Nominal maps: 10 cm RMS position error, 10% of cones missing, 5% wrong colors, 33%
without a color, 2 clutter landmarks per 100 m. Worst case: 15 cm, 20%, 10%, 50%, 4.

| | Nominal | Worst case |
|---|---|---|
| Path found | 98.8% | 93.1% |
| Inside the true cone lines, of the paths found | 98.9% | 96.6% |
| Mean distance from the true centerline | 0.12 m | 0.19 m |
| A path's largest distance from it, 95th percentile | 0.61 m | 1.57 m |
| Mean path length | 16.4 m | 14.3 m |
| `plan()` latency p50 / p99 / max (budget p99 10 ms) | 60 / 193 / 1,733 us | 49 / 175 / 3,545 us |

`per_track` lists where each plan failed or left the track (`failed_at_m`, `outside_at_m`, m
from the staging pose); `--plan <seed> <s> <level> <file>` exports one for plotting, and
`--config <file>` evaluates another ExplorationConfig.

## Track model and raceline (`raceline.json`)

20 generated tracks; the lap-1 drive wanders up to 0.5 m around the centerline. Raceline with
the defaults: 0.5 m margin, grip factor 0.70, per-axle g-g-v envelope.

| | Nominal map | Worst-case map | True track |
|---|---|---|---|
| Model centerline's largest distance from the truth, median / max | 0.14 / 0.28 m | 0.25 / 0.43 m | |
| Model centerline's RMS distance, mean | 0.050 m | 0.082 m | |
| Raceline lap time gain over the centerline, mean (min-max) | 10.5% (6.8-14.1%) | 11.5% (7.1-15.1%) | 9.4% (4.8-13.6%) |
| Smallest gap between the car's body and a true cone line | 0.21 m | 0.19 m | 0.43 m |

Mean predicted lap on the true tracks: 22.6 s on the raceline, 25.0 s on the centerline.
Raceline optimization (three QPs, 92-195 points): p50 32 ms, max 97 ms over 60 runs (budget
2 s). `--config <file>` evaluates another RacelineConfig.

## QP solver against OSQP (`qp.json`)

Both to 1e-9, OSQP with solution polishing and its setup included; median of repeated solves.

| Problems | Ours | OSQP |
|---|---|---|
| Raceline QPs of 10 generated tracks (30) | 10 ms median (12-14 iterations) | 129 ms median (1,500 iterations) |
| Random dense n = 25 / 50 / 100 / 200 / 400 | 0.07 / 0.24 / 0.75 / 4.3 / 35 ms | 0.23 / 0.69 / 6.8 / 28 / 97 ms |

Largest disagreement: 8e-10 in relative objective; 3e-5 m in the raceline solutions, 5e-6 in
the random ones.
