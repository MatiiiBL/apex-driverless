# Evaluation

How well does the car race a track it has never seen? This document answers with the
experiments of docs/design.md, section 8: the full stack on a fixed set of tracks and seeds,
compared run for run with the same car given perfect perception and state, and ablations of the
controller, the oracles, sensor noise, latency and grip. How a run is executed and scored, and
the exact definition of every metric, are in [runner.md](runner.md); how the car is assembled is
in [stack.md](stack.md). Every table here comes from `results/runs/`, every figure from
`tools/plot_evaluation.py`.

## Reproducing

```
make PRESET=release build
tools/run_evaluation.sh                 # every sweep, the summary and the figures
tools/run_evaluation.sh baseline        # one of them
```

Each sweep writes `results/runs/<sweep>/` (`runs.json` with every run's full result,
`summary.json` and `summary.md`); `tools/summarize_evaluation.py` gathers the summaries into
[results/runs/README.md](../results/runs/README.md) and `tools/plot_evaluation.py` draws
`docs/img/evaluation-*.png`. Runs are deterministic: the same build reproduces every number
except the wall-clock compute times.

## Method

**Tracks.** Eleven fixed circuits: the track generator's seeds 1 to 8 and its three reference
layouts.

| Track | Length (m) | Cones | Tightest radius (m) | Longest straight (m) | Hairpins | Chicanes | Sweepers | Other corners |
|---|---|---|---|---|---|---|---|---|
| gen1 | 431.5 | 203 | 7.3 | 60.5 | 3 | 5 | 1 | 1 |
| gen2 | 391.4 | 180 | 7.7 | 61.5 | 2 | 1 | 5 | 3 |
| gen3 | 231.1 | 109 | 7.2 | 59.5 | 1 | 2 | 2 | 1 |
| gen4 | 407.4 | 186 | 7.1 | 44.5 | 2 | 2 | 3 | 1 |
| gen5 | 394.6 | 183 | 7.2 | 33.5 | 3 | 2 | 3 | 2 |
| gen6 | 486.9 | 216 | 6.8 | 62.5 | 1 | 2 | 4 | 4 |
| gen7 | 405.2 | 195 | 7.1 | 31.0 | 4 | 1 | 1 | 5 |
| gen8 | 253.4 | 121 | 8.2 | 56.0 | 2 | 1 | 1 | 2 |
| technical | 315.0 | 152 | 6.4 | 26.5 | 3 | 2 | 1 | 1 |
| flowing | 473.5 | 201 | 7.0 | 61.0 | 1 | 1 | 8 | 2 |
| mixed | 414.5 | 186 | 6.8 | 39.0 | 2 | 1 | 3 | 4 |

(`apex track --seed N` or `--layout NAME` prints each line.)

**Runs.** A run is the mission of docs/design.md, section 1: staged 6 m behind the timing line,
ten laps (the first one exploring), then a stop, with every default of every module, the
default sensor noise and latencies (camera 40 ms, LiDAR 50 ms), the nominal car equal to the
true one and full grip. The run seed drives every random draw: each sensor's noise, the
camera's misses, confusions and false positives, and the oracles' noise. The baseline uses run
seeds 1 to 3 on all eleven tracks; the ablations, seeds 1 and 2 on six of them (gen1, gen3,
gen5 and the three layouts). A run ends as runner.md, "End of a run", describes; a DNF still
counts in every table, as not completed.

**The oracle.** Each baseline run is paired with the same track and seed driven with perfect
perception and perfect state (runner.md, "Oracles"). The *gap to the oracle* is the difference
of their mean racing laps: what estimation and perception cost in lap time, with the
planner and controller unchanged.

| Sweep | Variants | Runs | What it isolates |
|---|---|---|---|
| baseline | stack, oracle | 66 | completion, lap times, penalties, map, localization, association, tracking, compute |
| controllers | pure pursuit, Stanley, MPC | 36 | the path-tracking controller |
| oracles | none, perception, state, both | 48 | the cost of each half of the car's state estimation |
| noise | sensor noise ×0, ×0.5, ×1, ×2, ×4 | 60 | robustness to worse (or perfect) sensors |
| latency | nominal; processing ×2, ×4; actuators ×2 known to the controller, ×2 unknown | 60 | processing and actuator delay |
| friction | track grip 110 %, 100 %, 90 %, 80 %, 70 % of the tires' nominal | 60 | a planner that assumes more (or less) grip than there is |

The configs are in [results/runs/configs/](../results/runs/configs/); `ctest` checks that each
one parses and expands to the runs above.

**Compute times** are wall-clock and measured while three runs share the machine with other
work, so they are upper bounds on the module costs; the single-run figures against the budgets
of docs/design.md, section 6 are in runner.md, "Performance".

## Results

### Completion, lap times and penalties

The baseline's completion rate, lap 1 (exploration) against the racing laps, the penalties and
the gap to the oracle, over its 33 runs of the full stack.

### Mapping and localization

The map after lap 1 against the true cones (coverage, false and duplicate landmarks, colors,
position error) and the localization error while exploring and while racing on the frozen map.

### Data association

Precision and recall of the estimator's observation-to-landmark matches, by truth id.

### Tracking

The lateral error from the reference while racing, and the largest true distance from the
centerline.

### Compute

Each module's median and worst 99th-percentile time per call over the baseline.

### Ablations

Racing speed and completion for every variant of the five ablations, and the map and
localization error against the sensor-noise scale.

## Failure analysis

Every run that did not finish, or finished with penalties, by cause.

## Limitations

What the simulator does not model and the evaluation therefore cannot show.
