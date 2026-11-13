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
was shared with other jobs: an earlier run under heavier load measured 3-15 times the times
below (exploration p99 1.0 ms, raceline optimization p50 91 ms). docs/planning.md explains
the methods and the evaluations.

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
| `plan()` latency p50 / p99 / max (budget p99 10 ms) | 35 / 70 / 112 us | 31 / 56 / 79 us |

`per_track` lists where each plan failed or left the track (`failed_at_m`, `outside_at_m`, m
from the staging pose); `--plan <seed> <s> <level> <file>` exports one for plotting.

## Track model and raceline (`raceline.json`)

20 generated tracks; the lap-1 drive wanders up to 0.5 m around the centerline.

| | Nominal map | Worst-case map | True track |
|---|---|---|---|
| Model centerline's largest distance from the truth, median / max | 0.14 / 0.28 m | 0.25 / 0.43 m | |
| Model centerline's RMS distance, mean | 0.050 m | 0.082 m | |
| Raceline lap time gain over the centerline, mean (min-max) | 12.8% (8.4-17.0%) | 13.7% (8.5-18.0%) | 12.7% (7.1-17.3%) |
| Smallest gap between the car's body and a true cone line | 0.02 m | 0.03 m | 0.22 m |

Raceline optimization (three QPs, 92-195 points): p50 28 ms, max 62 ms over 60 runs (budget
2 s).

## QP solver against OSQP (`qp.json`)

Both to 1e-9, OSQP with solution polishing and its setup included; median of repeated solves.

| Problems | Ours | OSQP |
|---|---|---|
| Raceline QPs of 10 generated tracks (30) | 6.8 ms median (13 iterations) | 84 ms median (1,300 iterations) |
| Random dense n = 25 / 50 / 100 / 200 / 400 | 0.06 / 0.17 / 0.61 / 3.2 / 25 ms | 0.16 / 0.57 / 5.2 / 21 / 72 ms |

Largest disagreement: 2e-10 in relative objective; 3.7e-4 m in the raceline solutions
(degenerate bounds), 5e-6 in the random ones.
