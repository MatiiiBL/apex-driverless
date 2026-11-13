# Evaluation

How well does the car race a track it has never seen? This document answers with the
experiments of docs/design.md, section 8: the full stack on a fixed set of tracks and seeds,
compared run for run with the same car given perfect perception and state, and ablations of the
controller, the oracles, sensor noise, latency and grip. How a run is executed and scored, and
the exact definition of every metric, are in [runner.md](runner.md); how the car is assembled is
in [stack.md](stack.md). Every table here comes from `results/runs/`, every figure from
`tools/plot_evaluation.py`; the numbers are those of commit d51c587.

![The default car on the mixed layout: the map after lap 1 and the path of all ten laps](img/evaluation-replay.png)

**In short.** On eleven unseen tracks with three seeds each, the full stack finished 32 of 33
ten-lap runs, knocking 3 cones and leaving the track twice in all, every one of them on the
same track (generated track 7) and traced to one open issue of the MPC. It maps every cone on
lap 1 (median error 2.0 cm, no false landmarks), associates 99.997% of its matches correctly,
then races laps that take 40% less time than the exploration lap, repeating them to within
0.02 s and to within 0.3% of the lap time its raceline predicts. Perfect perception and state would make it
0.05 s (0.2%) faster per racing lap. Two problems remain open, both described with their
configs below: the MPC can steer the wrong way just after its reference changes at speed, and
the estimator stops adding landmarks when the sensor noise is three times its default.

## Reproducing

```
make PRESET=release build
tools/run_evaluation.sh                 # every sweep, the summary and the figures (8 min)
tools/run_evaluation.sh baseline        # one of them
```

Each sweep writes `results/runs/<sweep>/` (`runs.json` with every run's full result,
`summary.json` and `summary.md`); `tools/summarize_evaluation.py` gathers the summaries into
[results/runs/README.md](../results/runs/README.md) and `tools/plot_evaluation.py` draws
`docs/img/evaluation-*.png`. The 330 runs took 7 min 48 s on 8 threads of an Apple M4
(`THREADS=8 tools/run_evaluation.sh`). Runs are deterministic: the same build reproduces every
number except the wall-clock compute times. The animation `docs/img/replay.gif` is one run
recorded with `apex replay` (config `results/runs/configs/replay.json`, the export in
`results/runs/replay/`) and drawn with `tools/plot_replay.py --gif --speedup 3 --fps 12`.

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
ten laps (the first one exploring), then a stop, with every default of every module (the MPC
among the controllers), the default sensor noise and latencies (camera 40 ms, LiDAR 50 ms), the
nominal car equal to the true one and full grip. The run seed drives every random draw: each
sensor's noise, the camera's misses, confusions and false positives, and the oracles' noise.
The baseline uses run seeds 1 to 3 on all eleven tracks; the ablations, seeds 1 and 2 on six
of them (gen1, gen3, gen5 and the three layouts). A run ends as runner.md, "End of a run",
describes; a DNF still counts in every table, as not completed.

**The oracle.** Each baseline run is paired with the same track and seed driven with perfect
perception and perfect state (runner.md, "Oracles"). The *gap to the oracle* is the difference
of their mean racing laps: what estimation and perception cost in lap time, with the planner
and controller unchanged.

| Sweep | Variants | Runs | What it isolates |
|---|---|---|---|
| baseline | stack, oracle | 66 | completion, lap times, penalties, map, localization, association, tracking, compute |
| controllers | pure pursuit, Stanley, MPC | 36 | the path-tracking controller |
| oracles | none, perception, state, both | 48 | the cost of each half of the car's state estimation |
| noise | sensor noise ×0, ×0.5, ×1, ×2, ×4 | 60 | robustness to worse (or perfect) sensors |
| latency | nominal; processing ×2, ×4; actuators ×2 known to the controller, ×2 unknown | 60 | processing and actuator delay |
| friction | track grip 110 %, 100 %, 90 %, 80 %, 70 % of the tires' nominal | 60 | a planner that assumes more (or less) grip than there is |

The configs are in [results/runs/configs/](../results/runs/configs/); `ctest` checks that each
one parses and expands to the runs above. The ablations compare each variant with a reference
variant (MPC, no oracle, noise ×1, nominal latency, full grip) run for run, on the same track
and seed.

## Results

### Completion, lap times and penalties

| | Completed | Lap 1 (s) | Racing lap, mean (s) | Cones / run | Off-course / run | Gap to oracle |
|---|---|---|---|---|---|---|
| full stack | 32/33 | 39.87 | 23.68 | 0.09 | 0.06 | 0.05 s |
| perfect perception and state | 32/33 | 39.69 | 23.63 | 0.06 | 0.03 | |

![Mean speed per lap: lap 1 explores, laps 2 to 10 race](img/evaluation-laps.png)

- **Lap 1 against the racing laps.** Exploring, the car drives what it has seen at up to
  12 m/s and must always be able to stop within it; racing, it follows the minimum-curvature
  raceline at the g-g-v envelope's speeds. The racing laps take 0.60 of lap 1 on average (0.55
  to 0.62 over the runs): 9.5 m/s against 15.8 m/s mean speed.
- **The racing laps are as planned.** Laps 3 to 10 of a run differ by 0.02 s on average (0.04 s
  at most) and are 0.07 to 0.29% slower than the raceline's predicted lap time. Lap 2 is 0.61 s
  slower: it starts at the exploration speed.
- **Perception and estimation cost almost nothing at racing pace**: 0.05 s per racing lap,
  0.2%. The racing laps localize against the frozen map at 2.4 cm (median RMS), which the MPC
  cannot tell from the truth.
- **Penalties**: 3 cones and 2 excursions over the 33 runs of the stack, and 2 cones and 1
  excursion with the oracles; every one is on gen7 and explained under
  [Failure analysis](#failure-analysis).

| Track | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Raceline's prediction (s) | Oracle's racing lap (s) | Completed |
|---|---|---|---|---|---|---|
| gen1 | 45.87 | 27.97 | 27.88 | 27.87 | 27.91 | 3/3 |
| gen2 | 40.90 | 24.83 | 24.74 | 24.74 | 24.79 | 3/3 |
| gen3 | 23.82 | 14.61 | 14.56 | 14.54 | 14.58 | 3/3 |
| gen4 | 42.83 | 26.39 | 26.27 | 26.26 | 26.35 | 3/3 |
| gen5 | 42.51 | 24.54 | 24.44 | 24.43 | 24.49 | 3/3 |
| gen6 | 47.15 | 27.96 | 27.85 | 27.81 | 27.91 | 3/3 |
| gen7 | 46.16 | 28.06 | 27.92 | 27.90 | 28.02 | 2/3 |
| gen8 | 28.16 | 16.73 | 16.66 | 16.65 | 16.69 | 3/3 |
| technical | 36.02 | 22.39 | 22.28 | 22.27 | 22.34 | 3/3 |
| flowing | 43.10 | 23.86 | 23.73 | 23.73 | 23.83 | 3/3 |
| mixed | 42.05 | 24.60 | 24.47 | 24.49 | 24.50 | 3/3 |

(Means over the full stack's runs of each track; the raceline's prediction is its speed
profile's lap time, `planning::profile_time`.)

### Mapping and localization

Over the 32 completed runs of the stack:

- **The map after lap 1** has a landmark for every one of the 109 to 216 cones of every track,
  no landmark without a cone behind it, and 99.97% of the colors right. Its position error is
  2.0 cm RMS at the median run and 4.7 cm on average; the average is pulled up by one run
  (gen6, seed 2: 58 cm, 8 duplicate landmarks; see below). Every other run maps to within 17 cm
  RMS.
- **Localization** while exploring (mapping): 6.8 cm RMS at the median run, 9.2 cm on average;
  while racing on the frozen map: 2.4 cm median, 5.2 cm average, heading 0.04° RMS.
- **What the error comes from.** With perfect perception the map error is unchanged (3.5 cm
  against 3.1 cm in the oracle sweep): perception's detections are not the limit. With perfect
  state the map is 0.4 cm (mapping with known poses): the map error is the pose error of the
  SLAM filter while mapping, which grows with the sensors' noise (below).

### Data association

Of 1 316 061 cone observations in the completed runs, 1 228 385 were matched to a confirmed
landmark, 1 228 351 of them to the landmark of their own cone: a **precision of 99.997%**. Of
the 1 238 543 observations of a cone that already had a landmark, 99.2% were matched to it
(**recall**); the rest were gated out or fed a tentative landmark. The truth ids only score the
association: the stack's truth-id test relabels them all and gets bit-identical commands
(stack.md).

### Tracking

The MPC holds the raceline to **2.5 cm RMS** lateral error on average over the racing laps
(4.7 cm in the worst run) and follows the exploration paths, replanned at every scan, to
3.0 cm. The largest instantaneous error while racing was 0.62 m, on gen7 (seed 2). The
controller ablation puts this in context:

| Controller | Completed | Racing lap vs MPC | Tracking RMS, racing | Cones / run |
|---|---|---|---|---|
| pure pursuit | 12/12 | +0.32% | 11.6 cm | 1.17 |
| Stanley | 12/12 | 0.00% | 10.6 cm | 0.17 |
| MPC | 12/12 | | 2.4 cm | 0 |

Pure pursuit cuts corners. In the two runs we traced (gen5 and mixed), five of its six cones
fell on the first racing lap and one in the stop, each with the car's center about 0.9 to
1.05 m off the centerline, where the raceline stays within about 0.55 m of it: the raceline
keeps half a meter between the car's body and the cone line, and pure pursuit cuts more than
that. (A knocked cone counts once, so later laps through the same spot add nothing.) Stanley's
two cones are one per seed on the technical layout, both during exploration, in a hairpin at
4 m/s with the car 0.9 m off the centerline.

### Compute

| Work | p50 | worst p99 | Budget (docs/design.md, section 6) |
|---|---|---|---|
| Perception, one LiDAR scan | 293 µs | 662 µs | 10 ms |
| Estimation, one cone update | 155 µs | 1.06 ms | 5 ms |
| Estimation, one IMU prediction | < 1 µs | 1 µs | 0.05 ms |
| Exploration plan | 61 µs | 225 µs | 10 ms |
| Raceline optimization (once) | 42 ms (mean) | 82 ms (max) | 2 s |
| MPC, one command | 384 µs | 1.73 ms | 2 ms |

Medians and worst 99th percentiles of the baseline's runs, measured with eight runs in
parallel while other builds ran on the machine: upper bounds. Even so every module is inside
its budget, and a run simulates 26 times faster than real time on average (19.5× at worst).
runner.md, "Performance", has the single-run measurement.

### Ablations

![Ablations: racing-lap change against the reference variant, and penalties](img/evaluation-ablations.png)

- **Oracles.** Perfect perception changes nothing measurable (−0.01% racing lap, map 3.5 cm
  against 3.1 cm). Perfect state makes racing laps 0.23% faster and the map exact (0.4 cm).
  None of the four variants knocked a cone in 48 runs.
- **Sensor noise.** From noiseless sensors to twice the default noise every run finishes with
  no penalty and the same lap times (within ±0.1%), while the map error grows from 1.0 to 6.0 cm
  and the racing localization error from 1.9 to 6.5 cm, roughly in proportion. At four times
  the default noise every run fails at 3.5 s, before driving off: an open estimation issue
  (below).
- **Latency.** Doubling or quadrupling the processing latency (LiDAR results 200 ms after the
  scan) or doubling the actuators' dead times changes neither completion nor lap times
  (±0.02%). An actuator delay the MPC does not know about doubles its tracking error (2.4 to
  5.0 cm) without a penalty. Lap 1 is 0.5 s slower with ×2 and ×4 processing latency, and 1.0 s
  faster with the doubled actuator delay known to the controller, which we have not explained.
- **Friction mismatch.** The planner keeps the tires' nominal grip and plans with 70% of it.
  With 10% more grip the racing laps are 0.2% faster; with 10% and 20% less, 0.3% and 0.8%
  slower, and 20% less costs one cone in 12 runs. At 70% of the nominal grip the envelope's
  margin is gone: 5 of 12 runs complete, 24 cones per run, and the 7 others go off the track
  or stall after one to four racing laps. Lap 1 is unaffected (exploring, the car plans with
  70% of the grip at no more than 12 m/s). Adapting the envelope to the measured grip is the obvious
  next step; the stack does not do it.

![Map and localization error against sensor noise](img/evaluation-noise.png)

## Failure analysis

Every run that did not finish, and every penalty, by cause.

**1. The MPC steers the wrong way just after its reference changes at speed (open, control).**
All of the baseline's penalties and both of its DNFs happen on gen7, at the same place: the
start straight, where the car is fast, into a right-hander 20 to 30 m past the timing line, and
always within two seconds of the moment the controller's reference changes (exploration path to
raceline, or raceline to the stopping reference).

| Run | What happens |
|---|---|
| stack, seed 3 | At 49.1 s the car switches from the exploration path (12 m/s) to the raceline (28 m/s, turning right 20 m ahead). The MPC steers left, 0.35 rad at 14.6 m/s, and keeps steering left while the path turns right; 6 m off the track the car spins and stalls. DNF. |
| oracle, seed 2 | At 300.4 s the car starts its stop at 27.8 m/s; the MPC steers left to full lock at 21 m/s and slides off. DNF. |
| stack, seed 2; oracle, seed 1 | The same during the stop, milder: 1 m off the line and one cone each. |

The references are valid (the raceline stays within 0.72 m of the true centerline, its free
widths are at least 1.1 m, and it is nearly identical between seed 3 and the clean seed 1) and
the estimates match the truth to about a centimeter, so the controller is at fault, not its
inputs. The control module is investigating. To reproduce: `{"track": {"seed": 7}, "seed": 3}` and
`{"track": {"seed": 7}, "seed": 2, "oracles": {"perception": true, "state": true}}` with
`apex run`.

**2. Above twice the default sensor noise the map stops growing (open, estimation).** At ×3
and ×4 noise, every run fails at 3.5 s with no exploration path: while the car is staged, the
estimator starts and confirms six landmarks, then rejects every other observation (at ×4: 512
observations, 79 associated, 6 new landmarks, 433 rejected; at ×2 the same 6 s give 30
landmarks). Scaling perception's covariance model by 16 changes nothing, so the cause is in
how the estimator admits new landmarks rather than in the observations' covariance. To
reproduce: `{"track": {"seed": 1}, "seed": 1, "sensor_noise_scale": 3.0}`.

**3. A map with duplicates on the longest track (observed, not yet traced).** gen6 (487 m),
seed 2, finishes with no penalty, but its map has 224 landmarks for 216 cones and 58 cm RMS
error, against 8 to 13 cm for the two other seeds. Duplicates with a large error are what pose
drift over a long first lap looks like when the car comes back to cones it mapped at the start;
we have not confirmed that this is what happened. The car still races cleanly, localizing
against that same map.

**4. Friction mismatch at 70% grip (expected).** The seven DNFs and the cones of the grip-70
variant are the planner using grip the track does not have (above).

**5. Controller ablation (characteristic).** Pure pursuit's 14 cones and Stanley's 2 are their
known behaviors described under Tracking.

## Limitations

- **The simulator's world is simple.** Flat ground, no LiDAR motion distortion, cones that stay
  in place when knocked, Gaussian sensor noise, a tire model with one friction coefficient
  per axle and no temperature or wear. The perception and estimation errors here are those of
  this world, not of a real track.
- **The car is the simulator's car.** The nominal model equals the truth unless an ablation
  says otherwise; real model mismatch is larger and less structured than the friction and
  latency ablations.
- **Eleven tracks and three seeds** measure typical behavior, not rare failures: the
  wrong-way steering of the MPC above appears in 4 of 66 baseline runs, all on one track.
- **Compute times** are those of an Apple M4 running several simulations at once (above).
