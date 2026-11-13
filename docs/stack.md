# Stack: the car's software assembled

`apex::stack` is the car: a `ConePerception`, a `StateEstimator`, the exploration planner, the
track model and raceline, and a `Controller`, behind the mission state machine of
docs/design.md, sections 1 and 5.6. It depends on the four modules and on `apex::core`, never on
the simulator; the runner ([runner.md](runner.md)) races it through the `stack::Driver`
interface, which lives here for that reason. Results are in [evaluation.md](evaluation.md).

| Header (`include/apex/stack/`) | What it holds |
|---|---|
| `driver.hpp` | `Driver`, the boundary to the closed loop; `MissionState`, `Oracles`, `AssociationCounts` |
| `stack.hpp`, `stack_config.hpp` | `Stack` and `StackConfig` (every module's config plus the stack's) |
| `mission.hpp` | `Mission`: the state machine alone, fed a summary of the car at each control step |
| `lap_counter.hpp` | the timing line from the mapped big orange cones, and lap counting on it |
| `braking.hpp` | the reference for the stop after the last lap |
| `known_pose_mapper.hpp` | cone mapping with known poses, for the perfect-state oracle |

## Message routing

| Input | Goes to |
|---|---|
| IMU, wheel speeds, steering | the estimator's velocity filter and pose prediction |
| camera frame | perception's buffer, for coloring the next scan |
| LiDAR scan | perception, then the estimator's cone update, then the lap counter's line, then a new exploration plan while the exploration path is the reference |
| `control(now)`, every 20 ms | lap counting on the estimate, the mission, the controller |

Sensor messages arrive after their processing latency (runner.md), so a scan's cones reach the
estimator 50 ms after their stamp; the estimator maps them through its own odometry history
(docs/design.md, section 5.3). The stack itself keeps no clock other than the stamps it is
given.

## The mission

`Mission` holds the decisions, `Stack` the modules; keeping them apart makes every transition
testable without a simulator (`tests/stack/mission_test.cpp`).

| State | Reference | Leaves when |
|---|---|---|
| `Staged` | none: holds 2 m/s² of brake | `start_delay` (0.5 s) has passed and the estimate is usable → `Exploring` |
| `Exploring` | the exploration path, replanned on every scan | the car crosses the timing line a second time and the map makes a track model and raceline → `MapFinalized` |
| `MapFinalized` | still the exploration path, on the frozen map | `raceline_delay` (1 s) later → `Racing` |
| `Racing` | the raceline (the controller is reset on entry) | the lap counter reaches `laps` → `Stopping` |
| `Stopping` | a braking reference along the current path | the estimated speed is below 0.1 m/s → `Finished` |
| `Finished` | none: holds 2 m/s² of brake | |
| `Failed` | none: brakes at 9 m/s² | |

Any driving state goes to `Stopping` once the lap count is reached, so a one-lap mission stops
straight from `Exploring` and never computes a raceline. Any state goes to `Failed` on a
non-finite estimate or command, on an unusable estimate (invalid, or a position standard
deviation along its major axis above 2 m) for more than 0.5 s, or on 3 s without a fresh
exploration path while following one. If the map cannot be made into a closed track when lap 1
closes, the car explores another lap and tries again at the next crossing rather than racing on
a broken model.

**The raceline delay.** The raceline is optimized at once when lap 1 closes, but the car keeps
following the exploration planner for `raceline_delay` of simulated time before switching, as
it would while a real computer worked on it. Charging a fixed delay instead of the measured
wall-clock time keeps runs deterministic: the same config gives the same switch time on any
machine and under any load. The measured optimization time is reported in the telemetry.

**Stopping.** After the last lap the reference becomes the current path (raceline or
exploration path) from the car's position on, with the speed profile of the same g-g-v
envelope the raceline is planned with (`planning::apply_speed_profile`): from the car's speed
to rest in the shortest distance the envelope allows, found by bisecting on the stopping
distance to 5 cm, then held at rest for 5 m. The car crosses the line at racing speed, often
just before a corner; braking at a fixed deceleration along a curving path would ask the tires
for more than the friction ellipse leaves while cornering. The first version did that at
5 m/s² and spun the car on the oval from 25 m/s; along the envelope's profile the stop takes
2 s and stays on the line. The profile is capped at the car's current speed, so a stop never
speeds up. The controller tracks it like any other reference.

## Counting laps

The stack counts laps itself, from its own position estimate crossing the timing line it mapped,
and never sees the race officials. `LapCounter` takes the line between the means of the big
orange landmarks on the left and on the right of the staging pose, within 15 m of it; before
any are mapped it assumes a line square to the staging heading 6 m ahead (the FS staging
distance), so the first crossing is counted even with a late detection. A crossing counts when
the estimated position moves across the line in the driving direction within the line's span
extended by 1.5 m at each end (localization error near a cone must not make the car pass
beside it), and at least 30 m of driving after the previous one (the line moves a little as the
map refines it, and a move just as the car passes must not count twice). The first crossing
starts lap 1 and starts recording the driven path; the second closes it.

The driven path, poses every 0.5 m from the first crossing on, is what `build_track_model`
uses to order the mapped cones into the two boundaries (docs/design.md, section 5.4).

## Oracles

Ablations replace a module with the truth (docs/design.md, section 8), always through the
`Driver` interface:

- **Perfect perception.** The runner sends observations of the true cones instead of scans
  and frames (runner.md); the stack feeds them to the estimator exactly as it would feed
  perception's output.
- **Perfect state.** The runner sends the true state every tick and the stack acts on it
  instead of the estimator's. The map must then come from the true poses too: an EKF-SLAM map
  is consistent with the filter's own pose estimate, not with the truth, so pairing the true
  pose with the estimator's map would misplace every landmark by the estimator's pose error.
  `KnownPoseMapper` maps with known poses (S. Thrun, W. Burgard and D. Fox, *Probabilistic
  Robotics*, MIT Press, 2005, ch. 9): each observation is moved into the world frame with the
  true pose at its stamp (from a 0.5 s history of the true states), matched to the nearest
  landmark within 0.75 m, and fused into it as an information-form average,
  Λ ← Λ + R⁻¹, η ← η + R⁻¹ z, mean = Λ⁻¹ η. A landmark appears in the map after two sightings,
  which keeps one-off false positives out; colors are votes, as in the estimator.

## Truth ids

Measurements carry the id of the true cone behind them for the evaluation (docs/design.md,
section 4.3). The stack passes them along and counts association quality with them: for each
scan, `StateEstimator::last_associations()` (or the known-pose mapper's) gives the landmark
each observation was matched to, and comparing the observation's truth id with the landmark's
gives the precision and recall the runner reports. Nothing else reads them. The test
`FullStack.TruthIdsNeverChangeACommand` enforces it: it races the full stack twice on the same
track, the second time with every truth id relabelled through a random permutation, and
requires every one of the thousands of control commands to be bit-identical.

## Telemetry

Through the `Driver` interface: the mission state and its transitions with times and reasons,
the estimate the car acts on, the map, the reference being tracked, the association counts,
and the wall-clock compute time of each module (perception per scan, estimation per
prediction and per cone update, exploration per plan, the raceline optimization, the
controller per command, and the stack's own work per scan and per control step). `Stack` adds
the lap count, the timing line, the track model, the raceline and the driven path.
