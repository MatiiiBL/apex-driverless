# Apex: design

Apex is an autonomous racing stack for a Formula Student Driverless (FSD) car, written from
scratch in C++20, with the simulator it races in. This document is the contract between its
modules: the records they exchange, the interfaces they implement, the conventions they share and
the numbers they must report. Module documents (`docs/<module>.md`) explain each module's
internals; this one explains how they fit.

## 1. The mission

The car races an FS *trackdrive*: ten laps of a closed circuit it has never seen, marked only by
cones: blue on the left, yellow on the right, four big orange cones at the timekeeping line. It
starts at rest, staged 6 m behind that line.

- **Lap 1, exploration.** Nothing is known ahead. The car detects cones with its LiDAR and
  camera, builds a cone map while localizing in it (SLAM), and plans a path through the cones it
  has seen, at a speed from which it can always stop within what it has seen.
- **Map finalization.** Back at the timekeeping line the map is closed: the car orders the cones
  into the two boundaries of the track and freezes the map.
- **Laps 2 to 10, racing.** It computes a minimum-curvature raceline and the fastest speed
  profile its tires, motors and aerodynamics allow, then tracks it with model predictive control
  (MPC), localizing against the frozen map.
- **Stop.** After the tenth lap it brakes to a standstill.

The score is the FS one: total time plus 2 s per cone knocked and 10 s per off-course
excursion. A run that stalls, leaves the track by more than 5 m, or exceeds the time limit is a
DNF (did not finish).

## 2. Architecture

```
             simulator (apex::sim)                         the car's software
 ┌────────────────────────────────────────┐   ┌───────────────────────────────────────────────┐
 │ Track ─ cones                          │   │  perception ── ConeObservations ─┐            │
 │ Vehicle: dynamic single-track model,   │   │  (LiDAR clusters + camera color) │            │
 │   Pacejka tires, aero, actuators       │──▶│                                  ▼            │
 │ Sensors: LiDAR, camera detector, IMU,  │   │  estimation: velocity filter + EKF-SLAM       │
 │   wheel speeds, steering angle         │   │      │ StateEstimate, ConeMap                  │
 │ RaceMonitor: laps, cone hits, OC       │   │      ▼                                        │
 └────────────────────────────────────────┘   │  planning: exploration path │ track model +   │
              ▲                               │            minimum-curvature raceline         │
              │ ControlCommand                │      │ Trajectory                              │
              └───────────────────────────────│  control: pure pursuit │ Stanley │ MPC        │
                                              │  stack: mission state machine                 │
                                              └───────────────────────────────────────────────┘
                         runner: closed-loop runs, scoring, experiment sweeps; apps: `apex` CLI
```

| Module | Library | Depends on | Purpose |
|---|---|---|---|
| core | `apex::core` | Eigen, nlohmann_json | Records, geometry, splines, trajectories, tracks, RNG, timing |
| sim | `apex::sim` | core | Track generator, vehicle dynamics, sensor models, World, RaceMonitor |
| perception | `apex::perception` | core | Cone detection in LiDAR, camera color fusion |
| estimation | `apex::estimation` | core | Velocity filter, EKF-SLAM, data association |
| planning | `apex::planning` | core | Exploration planner, track model, raceline, speed profiles |
| control | `apex::control` | core | Pure pursuit, Stanley, MPC and its QP solver |
| stack | `apex::stack` | core + the four above | The car's software assembled, mission logic |
| runner | `apex::runner` | core, sim, stack | Closed-loop runs, scoring, sweeps, results |
| apps | `apex` (executable) | runner | Command-line tool |

**The car's software never depends on the simulator.** perception, estimation, planning,
control and stack link only `apex::core` (CMake enforces it). Their *tests* may link
`apex::sim` to produce realistic data. Ground truth reaches the car's software only through the
`truth_id` evaluation fields (section 4.3) and the runner's explicit oracle modes (section 5.6).

Each module's public headers are in `include/apex/<module>/`, sources in `src/<module>/`, tests
in `tests/<module>/`, benchmarks in `bench/<module>/`, and its design notes in
`docs/<module>.md`. Namespaces are `apex` for core and `apex::<module>` for the rest.

## 3. Conventions

**Units.** SI everywhere: meters, seconds, radians, kilograms, newtons. Angles in degrees only in
documentation and in config keys that say so (`*_deg`).

**Frames.**
- *World* (= map frame): fixed, x/y on the ground, z up. The car is told its start pose, as an FS
  car defines its map frame at the staging position, so estimates are directly comparable with
  the truth, with no alignment.
- *Body*: origin at the center of gravity (CoG), x forward, y left, z up. ConeObservations,
  ImuSample and the body velocities in StateEstimate are in this frame.
- *Sensor*: the LiDAR frame sits at `LidarGeometry::mount` with axes parallel to the body's.
  Camera detections are a bearing from the body's +x axis and a ground-plane range from the
  camera mount.
- Headings and yaws are counter-clockwise from +x, wrapped to (-pi, pi] with `wrap_angle`.
  Curvature and lateral offsets are positive to the left.

**Time.** `Time` is seconds of simulated time since the run started. The World advances in 1 ms
ticks; every sensor fires on exact ticks (its rate divides 1 kHz) and stamps its message with the
tick's time. Nothing in the car's software reads the wall clock to make a decision; wall-clock
time is measured only to report compute cost (`Stopwatch`, `LatencyStats`).

**Randomness and determinism.** Every random draw goes through `apex::Rng`, seeded from the run
seed and split into one stream per consumer (`Rng::split`), so adding draws in one place never
shifts another's numbers. No `std::random_device`, no `<random>` distributions (their output
differs between standard libraries), no unordered containers iterated where order changes
results, no threads inside one run. Two draws never share the argument list of one call:
C++ leaves the order of function arguments unspecified, and GCC and Clang evaluate them in
opposite orders, so `Vec2(rng.normal(), rng.normal())` swaps the draws between compilers. Draw
into named variables first, or use a braced initializer (`Vec2{rng.normal(), rng.normal()}`),
whose elements are evaluated left to right. The same inputs give bit-identical results on the
same build; across platforms, libm can change the last bits.

**Errors.** Bad configuration or arguments throw `std::invalid_argument` at construction. Hot
paths (per message, per control tick) do not throw: they return `std::optional`, a flag, or a
safe fallback, and count the failure in their stats. Every module checks its outputs for
non-finite values.

**Style.** clang-format (`.clang-format`, `make format`), Google-based with 4-space indents and
100 columns. `snake_case` functions and variables, `PascalCase` types, `kConstant` constants,
`member_` for private data members. Comments explain why and the non-obvious how. Warnings are
errors: `-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast`.

## 4. Records

All in `include/apex/core/`. They are plain structs, cheap to copy except the vectors.

### 4.1 Geometry and paths

| Record | Header | Notes |
|---|---|---|
| `Vec2`, `Vec3`, `Mat2`, `Mat3` | types.hpp | Eigen fixed-size types |
| `Pose2` | types.hpp | x, y, yaw; `to_parent`, `to_local`, `compose`, `inverse`, `between` |
| `Spline2D` | spline.hpp | Chord-parametrized cubic spline, open or periodic; `resample` by arc length |
| `TrajectoryPoint`, `Trajectory` | trajectory.hpp | Dense path with speed profile and free-space widths; `at(s)`, `project(p[, hint, window])`, `wrap_s`, `delta_s` |
| `Cone`, `ConeColor`, `cone_shape` | types.hpp | Ground-truth cones and FS cone dimensions |
| `Track` | track.hpp | Closed centerline (s = 0 on the timing line), cones, start pose, timing line; `track_from_centerline`, `make_oval_track`, `validate_track`, JSON save/load |

A `Trajectory` is the currency between planning and control: the exploration path (open, ending
where the car must have stopped), the raceline (closed), and the track's centerline (closed,
widths to the cone lines). Its `width_left`/`width_right` are the free lateral space from the
path to each boundary; controllers use them as constraints.

### 4.2 Vehicle

`VehicleParams` (vehicle.hpp) describes a representative FS electric car: 240 kg, 1.55 m
wheelbase, AWD with an 80 kW power limit, Pacejka tires with peak mu 1.6, downforce
ClA = 3.5 m^2, steering limited to 0.40 rad at 1.5 rad/s with 30 ms dead time. The simulator
uses one copy as the truth; the car's software gets its own *nominal* copy, which experiments may
perturb. `VehicleState` is the truth the simulator integrates; `ControlCommand` is a target front
wheel angle and a target longitudinal acceleration (total tire force over mass: the vehicle does
not compensate drag or rolling resistance; the controller must).

### 4.3 Messages

`messages.hpp` defines everything that crosses a module boundary at run time:

| Record | Producer → consumer | Rate | Frame |
|---|---|---|---|
| `ImuSample` | sim → estimation | 200 Hz | body: specific force ax, ay; yaw rate |
| `WheelSpeeds` | sim → estimation | 100 Hz | front/rear axle circumferential speed |
| `SteeringSample` | sim → estimation | 100 Hz | measured front wheel angle |
| `LidarScan` | sim → perception | 10 Hz | LiDAR frame; returns only |
| `CameraFrame` | sim → perception | 20 Hz | bearing/range per detection |
| `SensorMessage` | variant of the five | | `stamp_of()` |
| `ConeObservations` | perception → estimation | per scan | body frame at the scan's stamp, with covariance |
| `StateEstimate` | estimation → planning, control | per message | world frame pose, body velocities |
| `ConeMap` | estimation → planning | per scan | landmarks with covariance, color votes, stable ids |
| `ControlCommand` | control → sim | 50 Hz | |

`truth_id` fields carry the ground-truth cone id behind a measurement (-1 for ground, clutter or
a false positive): the LiDAR tags each return, perception gives each observation its cluster's
majority id, estimation gives each landmark its observations' majority id. They exist only for
the evaluation (association precision, map error per cone). **No decision in the car's software
may read them**: a stack test shuffles every truth id and requires identical commands.

### 4.4 Rates and latency

| Event | Period | Notes |
|---|---|---|
| physics tick | 1 ms | RK4 |
| IMU | 5 ms | |
| wheel speeds, steering angle | 10 ms | |
| camera | 50 ms | delivered 40 ms after its stamp (detector latency) |
| LiDAR | 100 ms | delivered 50 ms after its stamp (perception latency) |
| control | 20 ms | the stack's `control()` |
| actuators | | steering 30 ms dead time + 30 ms lag; drive 20 ms + 50 ms |

The runner models processing latency by delivering LiDAR and camera messages to the stack after
their stamps (configurable; the defaults above). The estimator therefore receives cone
observations stamped 50 ms before its newest IMU sample, and must handle it (section 5.3).

## 5. Module contracts

The interface headers below are fixed: changing a signature, a record, or a behavior described
here needs the coordinator's agreement, recorded in this document in the same commit. Each
module owns the *fields and defaults* of its config structs (the headers ship them empty) and
adds JSON `to_json`/`from_json` for them in its own headers (missing keys keep defaults; unknown
keys are rejected with `reject_unknown_keys`).

### 5.1 sim (`include/apex/sim/`: world.hpp, race.hpp, sensors.hpp)

**Track generator** (`sim/track_gen.hpp`, the module's own): `Track generate_track(seed,
options)` produces random closed circuits that satisfy the FS trackdrive rules and the apex
limits: centerline length 200–500 m (default range), track width 3.5 m, minimum centerline radius
6 m, no straight longer than 80 m, at least 4 m of clearance between non-adjacent parts of the
track (no confusable parallel sections), hairpins, chicanes and sweepers in proportion. It builds
the result with `track_from_centerline` and is deterministic in its seed. It also provides a few
fixed reference layouts by name.

**Vehicle model.** Dynamic single-track (bicycle) model with states x, y, yaw, vx, vy, yaw rate,
plus actuator states (steering angle, longitudinal force). Lateral tire forces from the Pacejka
formula of `TireParams` with normal loads from static weight, aerodynamic downforce (split by
`aero_balance_front`) and longitudinal load transfer; combined slip limits each axle's lateral
force to what the friction ellipse leaves after its longitudinal force. Drive force limited by
power (`max_power / v`), `max_drive_force` and traction; braking by `max_brake_force` and
traction; drag and rolling resistance oppose motion. Steering: dead time, first-order lag, rate
and angle limits. Blends into a kinematic model below ~3 m/s, where slip angles are ill-defined.
The car does not reverse. RK4 at 1 ms. `VehicleState::ax, ay` are the body-frame accelerations
an IMU would measure.

**Sensors** (`simulate_lidar`, `simulate_camera`, plus IMU, wheel and steering models):
- LiDAR: one ray per channel (`LidarGeometry::elevations`) and azimuth step, cast against the 3D
  cone bodies (a cone is a right circular cone of `cone_shape`) and the flat ground at z = 0,
  nearest hit wins, so cones occlude each other. Range noise, dropout, tagged `truth_id`.
  Spatial culling so a scan costs well under its 100 ms period.
- Camera: a cone detector's output, not pixels: each unoccluded cone within field of view and
  range is detected with a probability that falls with range, with bearing and range noise
  (range error grows with distance, as monocular depth does), a color confusion probability that
  grows with range, and Poisson false positives.
- IMU: specific force and yaw rate with white noise and slowly drifting bias. Wheel speeds:
  circumferential speed with noise and longitudinal slip under drive and braking. Steering angle
  with noise.
All noise parameters live in `SensorNoise`, documented with defaults that are realistic for the
class of sensor (docs/sim.md cites the datasheet-level figures used). `scale_noise(noise,
factor)` multiplies every noise standard deviation and bias magnitude by `factor` and leaves
probabilities and rates unchanged (0 gives noiseless measurements), for the sensor-noise ablation.

**World** steps the vehicle, fires the sensors on their ticks, and updates the **RaceMonitor**
(lap timing on timeline crossings, cone hits with the body footprint, off-course detection with
the four wheel contact points), all from the truth.

### 5.2 perception (`include/apex/perception/perception.hpp`)

`ConePerception::on_lidar(scan, yaw_rate)` returns the cones in a scan: ground removal (the
ground is flat, but the method must not rely on knowing the mount height exactly: a fitted plane
or a robust height threshold), clustering, shape checks against cone dimensions (height, width,
point count expected at that range) to reject non-cones, a center estimate corrected for seeing
only the near face, and a covariance that grows with range and shrinks with point count. Color
comes from the camera frame closest in time (`on_camera` buffers them), matched by bearing after
compensating the rotation `yaw_rate * (t_camera - t_scan)`; unmatched cones are `Unknown`.
Camera-only cones beyond LiDAR range are optional and, if added, carry a larger covariance.

### 5.3 estimation (`include/apex/estimation/estimator.hpp`)

`StateEstimator` fuses IMU, wheel speeds and steering into body velocities (a filter that
estimates the IMU yaw-rate bias and stays consistent under wheel slip), predicts the pose with
them, and runs EKF-SLAM over the pose and cone landmarks. Data association: Mahalanobis gating
with nearest neighbor, and joint compatibility (JCBB or an equivalent) where several
observations compete for nearby landmarks. New landmarks are tentative until seen in several
scans, which suppresses clutter and false positives. Color is a vote count per landmark.

Late observations: an observation stamped t_scan arrives when the filter is at t_now > t_scan.
The estimator keeps a short history of its odometry and maps observations through the motion
between t_scan and t_now before updating (or an equivalent, documented, approach).

`set_mapping(false)` freezes the landmark set (no new landmarks; positions may keep being refined
or be fixed, documented) for the racing laps. The estimator must stay real-time with the 200+
landmarks of a long track: report the update cost against landmark count.

`last_associations()` gives, for the most recent `on_cones` call, the id of the confirmed landmark
each observation was associated to, or -1 if it was rejected, gated out, initialized a new
landmark, or fed a tentative one. It exists for the evaluation (association precision and recall)
and reflects the estimator's decisions, never drives them.

Evaluation tools (the module's own): a batch least-squares SLAM over a whole run's odometry and
observations with ground-truth associations, as the reference for the online map error, and the
NEES consistency check.

### 5.4 planning (`include/apex/planning/planning.hpp`)

- `ExplorationPlanner::plan(map, state)`: Delaunay triangulation (the module's own
  implementation) of the mapped cones near and ahead of the car; candidate paths as sequences of
  edge midpoints, searched (beam search or equivalent) and scored on track width, curvature,
  color consistency where colors are known, cone spacing and heading change; the best is smoothed
  and given a speed profile that ends at 0 m/s before the end of the known path, so the car can
  always stop within what it has seen. Must cope with missing cones, misclassified colors and
  false positives.
- `build_track_model(map, driven_path, config)`: closed, ordered left and right boundaries and a
  centerline with widths, once lap 1 is done.
- `optimize_raceline(track, nominal, config)`: minimum-curvature optimization of the lateral
  offset along the centerline within the widths (minus half the car's width and a margin), as a
  QP solved with the module's own solver, iterated (re-linearized) a few times; then the speed
  profile. The QP solver is checked against OSQP (test-only dependency) on the same problems.
- `apply_speed_profile`: forward-backward integration over the g-g-v envelope of the nominal car.
  Shared by the exploration path and the raceline.

### 5.5 control (`include/apex/control/controller.hpp`)

Three controllers behind `make_controller`:
- *Pure pursuit* with a speed-dependent lookahead, plus a feed-forward and PI speed loop that
  compensates drag and rolling resistance.
- *Stanley* (front-axle cross-track and heading error), with the same speed loop.
- *MPC*: linear time-varying MPC on a dynamic single-track error model along the reference
  (lateral error, heading error, velocities, steering), linearized at each step around the
  reference and the previous solution, with steering angle and rate limits, acceleration limits,
  the reference's free-space widths as lateral constraints (soft, with slack), and compensation
  of the known actuator dead time. Its QP is solved by the module's own structure-exploiting
  solver (a Riccati-recursion interior-point or ADMM method, warm-started), checked against OSQP
  on identical problems for correctness and benchmarked against it for speed.

All controllers must run comfortably inside the 20 ms control period (target: MPC p99 under
2 ms on one core of an Apple M4) and never return a non-finite command.

### 5.6 stack, runner and apps

**Stack** (`apex::stack`): the car's software assembled. It owns a `ConePerception`, a
`StateEstimator`, the planners and a `Controller`, routes `SensorMessage`s to them, and runs the
mission state machine of section 1: `Staged → Exploring → (lap closed) MapFinalized → Racing →
Stopping → Finished`, or `Failed` (lost localization, no path for too long, non-finite state). It
detects laps itself, from its own estimate crossing the timing line it mapped from the big orange
cones; it never sees the RaceMonitor. Its API is the stack module's to define (it is used only by
the runner and the apps), and it reports telemetry: mission state, current reference, compute time
of every module.

**Runner** (`apex::runner`): one closed-loop run = a `World` and a `Stack` in lockstep, with the
latency model of section 4.4, ended by the mission finishing or a DNF rule. `RunConfig` (JSON)
holds the track (seed or file), vehicle and nominal parameters, sensor noise scale, friction
scale, controller, laps, latencies, seed, and oracle switches for ablations: *perfect perception*
(observations from the truth), *perfect state* (estimate = truth). `RunResult` (JSON) holds lap
times, penalties, DNF reason, map error, localization error, association precision/recall,
tracking error, and every module's compute-time summary. Sweeps run many independent runs in
parallel on a thread pool (one run per thread, so each run stays deterministic).

**Apps**: the `apex` CLI: `apex track --seed N --out track.json`, `apex run --config run.json
--out result.json [--log log.csv]`, `apex sweep --config sweep.json --out results/`, plus the
replay export used for the README's animation.

## 6. Performance budgets

Measured with `Stopwatch` around each call, reported as `LatencyStats` percentiles on an Apple M4
(one core, Release). A module that misses its budget at p99 is a bug.

| Work | Period | Budget (p99) |
|---|---|---|
| Perception, one LiDAR scan | 100 ms | 10 ms |
| Estimation, one cone update (200 landmarks) | 100 ms | 5 ms |
| Estimation, one IMU prediction | 5 ms | 0.05 ms |
| Exploration plan | 100 ms | 10 ms |
| Raceline optimization (once) | | 2 s |
| Controller, one command | 20 ms | 2 ms |
| Simulator, one tick with sensors | 1 ms | faster than 10x real time over a run |

## 7. Testing

- Unit tests per module (GoogleTest, `ctest -L <module>`), including failure paths.
- **Reference oracles**: every fast path is checked against a slow, obviously-correct one: the
  LiDAR raycaster against brute force over all cones, the QP solvers against OSQP, the vehicle
  model against analytic steady-state cornering and the kinematic limit, EKF-SLAM against batch
  least squares, Delaunay against the empty-circumcircle property.
- **Closed-loop tests**: short runs on the oval and on generated tracks that must finish with no
  penalties (stack/runner tests).
- Sanitizers (ASan + UBSan) in CI; the full suite runs in well under a minute in the dev preset.
- Benchmarks (`bench/<module>/`, Release only) print machine-readable results that the docs
  quote.

## 8. Evaluation

Results are produced by `apex sweep` over a fixed set of generated tracks and seeds, written to
`results/` (JSON + a summary table), and plotted with `tools/` (Python, uv). Metrics:

| Metric | Definition |
|---|---|
| Completion | runs finishing 10 laps without DNF |
| Lap time | per lap; lap 1 (exploration) and best/mean of laps 2–10 |
| Penalties | cones knocked, off-course excursions, total penalty seconds |
| Gap to oracle | racing lap time vs the same run with perfect perception and state |
| Map error | RMS distance from each landmark to its true cone (by truth id), after lap 1 |
| Localization error | RMS position and yaw error vs truth, per phase |
| Association | precision/recall of observation-to-landmark matches, by truth id |
| Tracking | RMS and max lateral error from the reference |
| Compute | p50/p99 per module (section 6) |

Ablations: controller (pure pursuit, Stanley, MPC), perfect perception and state (oracles),
sensor noise scale, actuator and processing latency, friction mismatch between the planner's
nominal mu and the track's.

## 9. Build and CI

CMake ≥ 3.25 with presets (`dev`, `asan`, `tsan`, `release`) and a Makefile wrapper:
`make check` = clang-format check + build + all tests, and must pass on every commit.
Dependencies are fetched pinned by checksum: Eigen 3.4.1, nlohmann_json 3.12.0, GoogleTest 1.17.0,
and OSQP 1.0.0 (tests and benchmarks only). CI (GitHub Actions) builds and tests on Ubuntu with
GCC 14 and Clang 18 (ASan + release) and on macOS (Apple Clang, release), and checks formatting.
