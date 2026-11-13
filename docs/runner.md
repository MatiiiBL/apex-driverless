# Runner: closed-loop runs, scoring and sweeps

`apex::runner` races a driver (the real `stack::Stack`, or a test stand-in) in the simulator,
scores the run from the ground truth, and runs experiments as sweeps of many independent runs.
It is the only code that sees both sides of the wall of docs/design.md, section 2: it owns the
`sim::World` and hands the car's software nothing but sensor messages, after their latency, and
the oracle inputs an ablation asks for. Results and their methodology are in
[evaluation.md](evaluation.md); the car's side of the loop is in [stack.md](stack.md).

| Header (`include/apex/runner/`) | What it holds |
|---|---|
| `config.hpp` | `RunConfig` and its JSON form: track, true and nominal car, noise and friction scales, latencies, oracles, module configs, DNF rules |
| `run.hpp` | `run_closed_loop` (any `stack::Driver`), `make_track`, `make_world_config`, the `RunObserver` hook |
| `run_stack.hpp` | `run_stack`: a run of the full stack built from the config |
| `delay_queue.hpp` | the delay line that models processing latency |
| `oracle.hpp` | the perfect-perception and perfect-state oracles |
| `metrics.hpp`, `result.hpp` | map, DNF and error metrics; `RunResult` and its JSON form |
| `recorders.hpp` | the CSV time series and the replay recorder |
| `sweep.hpp`, `summary.hpp` | sweeps on a thread pool; per-variant summaries and markdown tables |

## One run

A run is a `World` and a `Driver` in lockstep, one 1 ms tick at a time:

```
every tick (1 ms):
    messages = world.step()                  physics (RK4), sensors that fire on this tick,
                                             the RaceMonitor's lap timing and penalties
    each message -> delay line, due at stamp + its latency
    perfect-state oracle: driver.on_oracle_state(truth)
    every message now due -> driver.on_sensor / on_oracle_observations, oldest first
    every 20 ms: command = driver.control(now) -> world.command(command)
                 localization and tracking errors, observers
    end of run?   driver Finished or Failed; at rest after the last lap; a DNF rule
```

The driver never sees the RaceMonitor, the true state (unless the perfect-state oracle is on)
or the track. The command takes effect through the actuator model's dead times and lags, which
the vehicle model owns (docs/design.md, section 4.4).

**Latency.** Each message type has a processing latency, `RunConfig::latency`, whose defaults
are the design's: the camera's cone detector delivers 40 ms after the frame's stamp,
perception 50 ms after the scan's; IMU, wheel speeds and steering arrive at once. Latencies are
kept as whole ticks, so a 50 ms latency is exactly 50 ticks and never 49.999 ms that would round
the other way; `validate` rejects a latency that is not a whole number of milliseconds. The
`DelayQueue` is a binary heap ordered by due tick and then by push order, so messages due on the
same tick reach the driver in the order the sensors produced them: a camera frame stamped at
60 ms with 40 ms of latency is handed over before the IMU sample stamped at 100 ms. Each push
and pop costs O(log n) for the n messages in flight, about a dozen at the default rates.

A consequence the estimator must handle (docs/design.md, section 5.3): when a scan's cones
arrive, the IMU has already moved the filter 50 ms past the scan's stamp.

**Oracles.** An ablation replaces part of the car's software with the truth, through the
`Driver` interface rather than by letting the car reach into the simulator:

- *Perfect perception* drops LiDAR scans and camera frames. In their place, at each scan's
  stamp, `oracle_observations` reports every true cone perception could see: inside the
  LiDAR's azimuth limits, within perception's 40 m clustering range, and close enough that an
  unoccluded cone of its size would give perception's minimum of 3 returns above the ground
  threshold (`perception::expected_cone_returns`, from the beam geometry; big orange cones stay
  visible farther than small ones). Each comes with its true color and truth id, at its true
  body-frame position plus noise drawn from perception's own calibrated covariance model for
  that range and that return count (docs/perception.md, "Covariance"), which is also the
  covariance reported. So the oracle removes perception's misses, false positives, merges,
  color errors and occlusion, but not its nominal noise: the estimator sees the statistics it is
  tuned for. The observations reach the driver with the LiDAR's latency, as perception's output
  would.
- *Perfect state* hands the driver the true state every tick, stamped with the tick's time and
  with zero covariance. The stack then maps with the true poses (see stack.md).

The oracle's noise has its own `Rng` stream split from the run seed, apart from the world's
streams, so turning an oracle on never shifts the simulator's noise.

**End of a run.** In the order they are checked after each tick:

| Outcome | Rule | Result |
|---|---|---|
| finished | the race officials timed every lap and the car is below 0.1 m/s, or the driver reports `Finished` | `completed`, `stopped` |
| not stopped | every lap timed but the car still rolls `stop_timeout` (30 s) later | `completed`, not `stopped` |
| mission failed | the driver reports `Failed` | DNF, with the mission's reason |
| ended early | the driver reports `Finished` before the officials timed every lap | DNF |
| non-finite command | `control()` returned NaN or infinity | DNF |
| off the track | the CoG more than 5 m outside a cone line | DNF |
| time limit | 600 s of simulated time | DNF |
| stalled | slower than 0.5 m/s for 10 s | DNF |

Laps are the officials', not the driver's: the stack counts its own laps to decide when to stop,
and a stack whose count runs ahead of the officials' ends the run early and is scored as such.

## Metrics

All from the truth the runner holds (docs/design.md, section 8); none of it reaches the car.

- **Lap times and penalties** are the RaceMonitor's: laps between timing-line crossings, 2 s
  per cone knocked (counted once per cone), 10 s per off-course excursion. `score` = sum of the
  lap times + penalties.
- **Map error** is evaluated on the driver's `ConeMap` when lap 1 closes (the mission enters
  `MapFinalized`), or at the end of a run that never got there. Each landmark is paired with
  the true cone of its `truth_id` (the majority id of the observations behind it): `coverage`
  is the share of true cones with at least one landmark, `duplicates` counts extra landmarks of
  the same cone, `false_landmarks` those with no cone behind them, `color_accuracy` the share of
  matched landmarks with the true color, and `position` the RMS and maximum distance from a
  matched landmark to its cone.
- **Localization error** per mission state: at every control step, the distance between the
  estimated and the true CoG position and the absolute heading error. Phases matter because the
  error during lap 1 (mapping) and on the frozen map (racing) come from different mechanisms.
- **Tracking error** per mission state: the lateral distance of the estimated position from the
  reference the controller tracks at that step. It measures the controller against what it
  can know; the true distance from the centerline (`max_lateral_true`) bounds the combined
  effect of tracking and localization error. Projections onto the closed raceline search a
  15 m window around the previous one, so a tight hairpin cannot snap the projection to the
  other leg.
- **Association** counts, from the truth ids, how the stack's matching of observations to
  landmarks went: `precision` = correct / matched (of the observations matched to a landmark,
  the share matched to the landmark of their own cone), `recall` = correct / matchable (of the
  observations of a cone that already had a landmark, the share matched to it). The stack
  counts them as it goes, from `StateEstimator::last_associations()`.
- **Compute** is each module's `LatencyStats` summary (wall clock), plus the simulator's mean
  tick time and the run's real-time factor.

## Determinism

The same `RunConfig` gives a bit-identical `RunResult` on the same build, apart from the
wall-clock fields that `deterministic_json` leaves out. Every random draw comes from an `Rng`
stream split from the run seed; the world and the stack are single-threaded; the raceline's
compute time is charged as a fixed delay of simulated time, not measured (stack.md). The test
`RunClosedLoop.DeterministicAndRecorded` runs the same config twice and compares the results.

## Sweeps

A sweep (`SweepConfig`, JSON) is a base run crossed with tracks, seeds and *variants*. A variant
is a name and a JSON merge patch (RFC 7386) applied to the base's JSON before it is parsed, so
`{"stack": {"controller": {"kind": "stanley"}}}` changes the controller and nothing else, and a
patched true car still carries the nominal model along unless the patch sets `nominal` too.
`expand_sweep` validates every run before any starts and names it
`<variant>/<track>/s<seed>`; an error names the variant or run at fault.

`run_parallel` runs the configs on a pool of worker threads, each taking the next run not yet
taken from an atomic counter and writing its result into that run's own slot. Runs share no
mutable state (each owns its World, Stack and Rng streams; the log level is an atomic), so a
result never depends on the thread count or on which worker ran it. The tests check exactly
that, with stub runs (`RunParallel.*`) and with real ones (`FullStack.SweepResultsIndependentOf
ThreadCount`); under the tsan preset (`make PRESET=tsan test`) the same tests are the data-race
check. A run that throws becomes a result with `error` set; the sweep goes on.

`summarize` groups the results by variant. Means are over the runs that have the value (a run
that went off on lap 1 has no racing laps), penalties are per run, association is pooled
(sum of correct over sum of matched), compute is the mean p50 and the worst p99 over the runs.
The *gap to the oracle* pairs each run with the oracle variant's run on the same track and seed
and averages the difference of their mean racing laps over the pairs where both completed.

## Performance

`bench/runner/apex_runner_bench` races the full stack once on each reference layout and then
reruns the three as a sweep on 1, 2 and 3 threads. Release build at commit f8037da, Apple M4,
nothing else running (results/runs/compute.md has the machine and every module's benchmark):

```
make PRESET=release build
build/release/bench/runner/apex_runner_bench            # 3 laps with the MPC; [laps] [controller]
```

| Layout | Real-time factor | Simulator tick (mean) | Perception scan p50 / p99 | Estimator cone update p50 / p99 | Exploration plan p50 / p99 | MPC p50 / p99 |
|---|---|---|---|---|---|---|
| technical | 43.6× | 5.8 µs | 202 / 269 µs | 117 / 517 µs | 54 / 90 µs | 252 / 341 µs |
| flowing | 43.6× | 5.8 µs | 190 / 236 µs | 86 / 457 µs | 47 / 72 µs | 252 / 315 µs |
| mixed | 43.8× | 5.8 µs | 194 / 238 µs | 89 / 567 µs | 49 / 75 µs | 252 / 328 µs |

The MPC's command is most of a run's compute: with Stanley (7 µs a command;
`apex_runner_bench 3 stanley`) the same runs go 97.6 times faster than real time. The
simulator's tick includes the LiDAR's 57 600 rays every 100 ms, amortized; a whole scan through
perception, estimation and planning costs 0.3 ms at the median and 0.8 ms at p99, against a
100 ms period. The sweep pool runs the three layouts 2.8 times faster on three threads than on
one (each run is one thread, and three runs keep three threads busy).

## Recording a run

`RunObserver`s see the run after every control step and only read. `CsvLog` writes one row per
`log_period` (0.1 s by default) with the time, the mission state, the true and estimated state,
the command, the officials' centerline position, laps and cone hits, and the map size.
`ReplayRecorder` keeps what an animation needs: the true and estimated pose at 20 Hz, the map
every second while it is being built and once frozen, and the reference whenever it changes
(at most every 0.5 s while exploring), with positions rounded to the millimeter.

## The `apex` tool

```
apex track  --seed N | --layout NAME | --oval  --out track.json
apex run    --config run.json --out result.json [--log run.csv]
apex sweep  --config sweep.json --out DIR [--threads N]
apex replay --config run.json --out replay.json
```

Every config key is optional; unknown keys are errors that name the key. `apex sweep` writes
the sweep's config, every result (`runs.json`), and `summary.json` / `summary.md`;
`apex sweep --list` validates a sweep and prints its runs without running them. Usage errors
exit with status 2 and point to `--help`; failures (an unreadable file, an invalid config) exit
with 1; a run that does not finish is a result, not a failure, and exits with 0.

## Testing

`tests/runner/` drives the loop with `CenterlineDriver`, a stand-in that cheats: it takes the
true state through the perfect-state oracle and follows the true centerline with pure pursuit.
It can also misbehave on purpose (stall, veer off, report a failure, return NaN, finish early,
never stop), so each DNF rule is triggered by the behavior it exists to catch. The tests check
lap times against the RaceMonitor, the delivery time of every message type under custom
latencies, the oracles, determinism and both recorders; the full stack's closed-loop tests,
including the truth-id test, are described in stack.md.
