# Estimation

`apex::estimation` turns the IMU, wheel speeds, steering angle and perception's cone
observations into the car's state (`StateEstimate`: pose in the map frame, body velocities) and a
cone map (`ConeMap`). The contract is docs/design.md, section 5.3; this document explains the
models, the choices behind them, and what they measurably achieve.

```
 IMU 200 Hz ──┐
 wheels 100 Hz├─▶ VelocityFilter ──twist──▶ OdometryBuffer ──motion──▶ EkfSlam ◀── cones 10 Hz
 steering ────┘   (vx, vy, r, bias)        (last 0.5 s)               pose + landmarks    │
                                                                           ▲              │
                                                                     Associator ◀─────────┘
                                                                     (gating + JCBB)
```

| File | Responsibility |
|---|---|
| `velocity_filter.hpp`, `lateral_model.hpp` | EKF over body velocities and gyro bias; the single-track slip relations it uses |
| `odometry.hpp`, `dead_reckoning.hpp` | SE(2) integration of the twist, its noise, the history for late scans |
| `ekf_slam.hpp` | EKF over pose and landmarks: predict, Joseph-form batch update, Schmidt update |
| `association.hpp`, `chi_square.hpp` | Mahalanobis gating, joint compatibility branch and bound, chi-square quantiles |
| `observation.hpp` | What the filter fuses for one of perception's cones |
| `estimator.hpp` | `StateEstimator`: everything above, landmark life cycle, recovery |
| `eval/` | Synthetic drives, the World driver, NEES, map and association metrics, batch SLAM |

## 1. Velocity filter

State x = (vx, vy, r, b): body velocity at the CoG, yaw rate, gyro bias. The accelerometer is an
input, not a measurement: planar rigid-body kinematics in the rotating body frame give

    vx' = ax + r vy,    vy' = ay − r vx,    r' = w_r,    b' = w_b

with r and b random walks (densities `yaw_accel_density`, `gyro_bias_walk`) and the
accelerometer's noise entering as process noise (`accel_noise_density`, which also has to cover
its turn-on bias). The gyro measures r + b on every IMU sample. Euler steps at 200 Hz; their
error is far below the sensor noise.

**Wheel speeds.** Each axle's circumferential speed is its ground speed along the wheels times
(1 + κ): rear vx (1 + κr), front (vx cos δ + (vy + lf r) sin δ)(1 + κf). The slip ratio is
predicted from the axle's longitudinal force coefficient, κ = (Fx / Fz) / 20, where Fx = m ax plus
drag and rolling resistance, split as the drivetrain or the brakes split it, and Fz includes
downforce and load transfer. Its prediction error (50 % of κ) is added to the wheel noise.
Samples whose normalized innovation exceeds 2.5 σ are down-weighted with Huber's weight
(Huber 1964), implemented as an inflated R so the update stays a Kalman update; this is what
keeps vx right through wheel spin and lock-up.

**Lateral pseudo-measurements.** Each steering sample adds two zero-valued measurements from the
single-track model (Rajamani 2012, ch. 2): at each axle, the lateral velocity in the wheel frame
is the longitudinal one times tan α, where α is the slip angle at which the axle produces the
lateral force the IMU says it must:

    Fyf + Fyr = m ay,    lf Fyf − lr Fyr = Iz r'

with r' a low-passed derivative of the gyro. α comes from inverting the magic formula of the
nominal tires on its rising branch, with the lateral capacity narrowed by the friction ellipse,
μ Fz √(1 − (Fx / μ Fz)²), because the longitudinal force takes part of the grip (combined slip,
Pacejka 2012). Without that term the filter was confidently wrong about vy whenever the car
braked into a corner on the simulator. Axles above 90 % of their lateral grip skip the update:
near the peak of the tire curve the slip angle no longer follows from the force. Together with
the gyro, these make vy and the bias observable while driving; at rest they pin r to zero, which
calibrates the bias on the grid.

**Correlated model error.** The slip predictions are only as good as the nominal tires, and their
error is not white: it lasts as long as the manoeuvre. Fusing 100 samples a second as if each
had an independent error averages it away and makes the filter overconfident exactly where the
model is worst. The model part of each wheel and lateral variance is therefore multiplied by the
number of samples per `slip_correlation_time` (0.3 s), so the samples sharing one model error
count as one. On synthetic drives this took the normalized vx error under 8 m/s² acceleration
from 7–9 (squared, per component; 1 is consistent) to about 1, and vy at the limit from about 6
to about 2. The textbook alternative, augmenting the state with the correlated error
(Bar-Shalom, Li and Kirubarajan 2001, ch. 8), would need a state per axle and per direction.

Every update uses the Joseph form. The defaults are the simulator's sensor class (gyro
0.002 rad/s, steering 0.002 rad, wheels 0.05 m/s, docs/sim.md) with margin: the bias walk is
twice the simulator's, and the accelerometer density 0.08 (m/s²)/√Hz is about twenty times its
white noise, to cover the turn-on bias and the model's approximations.

## 2. Odometry, and observations that arrive late

The pose moves by the filter's twist, integrated per IMU interval with the trapezoidal twist and
the exact SE(2) exponential (`integrate_twist`). The odometry's noise models the velocity error
as a first-order Gauss-Markov process of correlation time τ, whose integral grows like white
noise of density 2 τ Σv (Maybeck 1979, section 4.11), plus floors for what the filter does not
model (0.01 m/√s, 0.001 rad/√s). Most of the velocity error is wheel-noise jitter that averages
out within milliseconds: on synthetic drives the odometry's motion over one second is accurate
to 2–3 mm and 0.0003 rad. τ = 0.05 s keeps a margin for the slower errors of slip; the
covariance is still several times conservative over one second, and the test requires only that
it is never overconfident. The first defaults (τ = 0.2 s, floors twice as large, a start yaw
uncertainty of 0.002 rad instead of 0.0005) claimed about ten times the real drift in standard
deviation, and the SLAM leaned on the noisy observations instead: map RMS 0.164 m against 0.122 m
with the current values on the same synthetic drive (track seed 5). One part of the yaw-rate
error is far slower than τ, and white noise cannot describe it at any τ: the SLAM carries it as
a state (section 3).

**Late observations.** Cone observations are stamped with their LiDAR scan and arrive 50 ms later
(design 4.4). The estimator keeps the SLAM state behind the odometry: each IMU sample commits to
the EKF only the motion older than `history_duration` (0.5 s); the newer motion stays in an
`OdometryBuffer`. A scan stamped t is fused at t: the EKF is predicted to t with the buffered
motion up to t (splitting an increment if needed), updated, and the motion after t stays pending.
`estimate()` composes the EKF pose with the pending motion and its covariance. Because the
velocity filter never reads the SLAM state, this is exactly the computation of fusing on time,
reordered: the test runs the same drive with 0, 50 and 100 ms of latency and finds the maps and
poses equal to 1e-6 m. Scans older than the history are dropped and counted (`late_dropped`).
The alternative of re-running the filter from a stored state (Bar-Shalom's out-of-sequence
measurement methods) is unnecessary when the measurement order is known to be "IMU, then cones,
late": nothing needs undoing.

## 3. EKF-SLAM

State (x, y, ψ, β, l1x, l1y, …) with a dense covariance (Smith, Self and Cheeseman 1990;
Dissanayake et al. 2001), β being the slow yaw-rate error below. Prediction moves the pose by a
relative motion and touches only the 4 × 4 vehicle block and the 4 × n vehicle-landmark
cross-covariance, O(n). The update observes landmark positions in the body frame,
z = R(ψ)ᵀ(l − p), for all of a scan's pairings at once. H is zero outside the pose and the paired
landmarks, so W = P Hᵀ is formed from those columns only, and the Joseph form
P' = (I − KH)P(I − KH)ᵀ + KRKᵀ = M − (MHᵀ − KR)Kᵀ, M = P − KWᵀ, needs M only in the paired
columns: the n × n work is the single product P − [K T][W K]ᵀ (T = MHᵀ − KR, inner dimension 2m),
of which only the lower triangle is formed and then mirrored. P is exactly symmetric and stays
positive definite, which the tests check over long runs. The covariance's storage grows by
doubling and a removed landmark is replaced by the last one, so neither copies the matrix.

**The slow yaw-rate error.** The velocity filter's yaw rate is off by a few 1e-4 rad/s for many
seconds at a time: what its gyro bias estimate and its lateral model leave. Modelled as white
odometry noise, the yaw error it causes grows like √T; it really grows like T. On World seed 5
(synthetic observations, with new landmarks limited to 20 m as they then were) that cost the loop
closure: the yaw drifted by 0.015 rad over lap 1 while the covariance claimed a third of the 0.6 m
position error, and at the timing line the first cones fell outside even the new-landmark gate and
were mapped again, 16 duplicates. At the 30 m range the lap drifts less and closes either way,
and β halves its lap-1 pose error (0.272 → 0.137 m, section 6). β carries it as a
first-order Gauss-Markov process, the shaping filter for time-correlated noise (Maybeck 1979,
section 4.11):

    β' = φ β + w,   φ = exp(−T/τ),   var(w) = σ² (1 − φ²),   ψ' = ψ + Δψ − τ (1 − φ) β

with σ = 3·10⁻⁴ rad/s and τ = 10 s. The prediction integrates it into the yaw, so the yaw
variance grows like T² over seconds; the landmarks observe β through the drift they see; and,
being stationary, a value learned in one corner fades over τ instead of being extrapolated for a
lap. Both the decay and the integral compose exactly, so splitting a motion at a scan's stamp
changes nothing and late scans still equal on-time ones. `yaw_rate_bias_sigma = 0` leaves β out.

Two designs came first and were rejected on the evaluation below (World seed 1, synthetic
observations, lap-1 pose RMS; 0.099 m without β). This comparison and the parameter scan after it
were made with the 20 m new-landmark range:

| β model | seed 5 fixed | world seed 1 lap 1 | why |
|---|---|---|---|
| random walk on the residual of the filter's gyro bias estimate, prior σ 0.01 rad/s | yes | 0.303 m | β fitted the corner-to-corner yaw-rate errors as a bias and extrapolated them (0.77 m peak error) |
| the same, shifted by every change of the filter's estimate, fed the filter's bias information, with the gyro minus the filter's bias as the odometry's yaw rate | yes | peak 0.66 m | the SLAM's bias estimate was right to σ = 4·10⁻⁴, but the filter's yaw rate is better than gyro − bias: its lateral model knows the yaw rate while driving, even when its separate bias estimate wanders by 1.5·10⁻³ |
| Gauss-Markov on the filter's yaw-rate error (kept) | yes | 0.115 m | |

The kept model's parameters, from five neighbouring settings (sums over the 14 runs other than
world seed 5; without β: map 0.733 m, lap-1 pose 1.605 m, racing pose 0.874 m):

| σ (rad/s), τ (s) | map | lap-1 pose | racing pose | runs with lap-1 pose worse by > 1 cm | world seed 5 racing |
|---|---|---|---|---|---|
| 5e-4, 20 | 0.676 | 1.437 | 0.833 | 3 (up to +0.091) | 0.039 |
| 3e-4, 20 | 0.709 | 1.437 | 0.857 | 2 (up to +0.049) | 0.045 |
| 5e-4, 10 | 0.666 | 1.413 | 0.826 | 2 (up to +0.067) | 0.033 |
| 1e-3, 20 | 0.644 | 1.517 | 0.813 | 4, and maps worse on 1 | 0.029 |
| **3e-4, 10** | 0.694 | 1.443 | 0.846 | 2 (up to +0.038) | 0.037 |

3·10⁻⁴ rad/s is the size of the slow error measured on the World runs (2–9·10⁻⁴ rad/s); 10 s is
about a sequence of corners.

**Landmarks.** An observation that matches nothing and is outside every landmark's new-landmark
gate (below) starts a tentative landmark, if it is within 30 m and not within 0.5 m of another.
Tentative landmarks take part in association and updates like any other, but they are not in the
published map; a tentative seen in three scans is confirmed and gets the next id, which never
changes or is reused. Tentatives unseen for 0.35 s are dropped, which removes clutter and false
positives (a false positive is rarely seen twice in the same place). Each landmark counts color
votes; its color is the most-voted one, Unknown on a tie. `ConeMap::revision` increments whenever
the published map changes. The stats count why each unmatched observation started nothing:
left out of the joint hypothesis inside a gate (`unpaired_in_gate`), between the two gates
(`ambiguous`), beyond the new-landmark range (`too_far_to_map`), or too close to a landmark
(`too_close_to_map`).

The new-landmark range (`max_new_landmark_range`) is the observation range, 30 m. It was 20 m,
and that was the cause of two failures (section 6, "The new-landmark range"). Cones come into
view at the far end of the range, and every observation of a cone before it came within 20 m was
turned away, because it matched no landmark and was too far to start one. A car staged at rest
sees nothing come nearer: at three times the simulator's sensor noise, when perception no longer
finds the big orange cones beside the car, it mapped six cones and the planner found no path. On
the move the estimator effectively had a 20 m sensor, and lap 1 drifted more. The far
observations cost no false or duplicate landmarks in any run here: their covariance grows with
range, and a landmark needs three detections.

**Localization mode.** `set_mapping(false)` drops the tentatives and freezes the landmark set.
Updates then are Schmidt-Kalman updates (Schmidt 1966): the landmarks are "consider" states whose
estimates and covariance stay fixed, while the pose and its cross-covariances are updated
consistently, at O(n m) instead of O(n² m). The map is final after lap 1: refining it further
during the racing laps would gain little and let a misassociation at speed corrupt it.

**Loop closure** needs no special case: when the car returns to the first cones, their landmarks
are associated through the full covariance, whose cross terms carry the lap's drift, and the
update pulls the whole map together, provided the covariance has kept up with the lap's drift
(the yaw-rate bias state above). On the oval, re-observing the start produces no duplicate
landmarks (test `ClosesTheLoopOnTheOvalWithoutDuplicates`), nor on the drifting World seed 5
(`TheYawRateBiasStateKeepsADriftingLapFromDoublingTheMap`, which also checks that at the 20 m
range that loop fails without the state). Nothing re-associates a lap that has drifted beyond the
new-landmark gate: its first cones are mapped again, as on track 6 at the 20 m range (section 6).

**Recovery.** A slide the velocity filter misjudges can move the pose outside its own gates; then
no scan associates and the error never returns. When three scans in a row bring at least four
usable observations, none matches, and at least four landmarks lie ahead within the new-landmark
range, the pose covariance grows by (0.5 m, 0.05 rad), which lets joint compatibility find the map
again. Before
the combined-slip model, a simulator run lost its map this way in a racing lap and drifted past
0.8 m with no scan matching again; with the recovery, the runs that slid re-acquired the map and
finished the racing lap at 0.09–0.13 m position RMS.

## 4. Data association

Cones are 2–5 m apart, look alike, and the pose's uncertainty grows between loop closures: the
nearest landmark is often the wrong one. Association therefore has two stages.

**Individual gating.** For each observation and landmark the innovation ν = z − h(x) and its
covariance S = H P Hᵀ + R (with the pose-landmark cross terms) give d² = νᵀS⁻¹ν. A landmark is a
candidate if d² < χ²₂(0.999) = 13.8. Landmarks that cannot pass even the new-landmark gate are
culled before S is formed, with a bound from |ν| and the trace of S, which keeps the cost linear
in the landmark count. Each observation keeps its 4 best candidates.

**Joint compatibility (JCBB, Neira and Tardós 2001).** Among the gated pairings it finds the
largest set whose stacked innovation passes χ²₂ₖ(0.999) with its full joint covariance, ties
broken by the smaller joint distance. Because all innovations share the pose error, a hypothesis
that explains one observation with a shifted landmark must explain the others with the same
shift, which the neighbours of a dense cone row do not allow. The search is depth first, most
constrained observation first, starting from a greedy hypothesis so that an unambiguous scan costs
two passes; each extension appends two rows to a Cholesky factor of the joint covariance,
O(k²). A node budget (2000) bounds the worst case; when it runs out the best hypothesis so far,
always jointly compatible, is used (`search_exhausted`).

**Thresholds.** 0.999 makes a missed true pairing rare (1 in 1000 per observation) while keeping
gates small; the joint test at the same level rejects shifted hypotheses. An unmatched observation
starts a landmark only beyond χ²₂(0.99999) = 23.0 of every landmark. Between the two gates it is
ambiguous and dropped: a re-observation predicted badly enough to fail the gate is far more likely
than a new cone within 4.8 σ of a known one, and dropping it is what prevents duplicates.

The tests set the traps: two rows of bent cone lines seen from a pose 2 m off its estimate, where
individual nearest neighbour (ICNN) gets fewer than half of 12 pairings right and JCBB all of them;
two landmarks inside one gate, resolved by a third observation; clutter halfway between two cones
inside both gates, left unpaired.

## 5. Evaluation tools

- **Synthetic drive** (`eval/synthetic.hpp`): the car follows a track's centerline exactly, with a
  steady-state slip model for vy, and the IMU, wheels and steering are derived from that motion
  with noise, bias and a nonlinear wheel slip that grows without bound near the friction limit.
  Cone observations from the true cones: 180° field of view, 25 m range, detection probability
  0.97 falling to 0.6, range and bearing noise growing with range (2 cm + 0.2 %, 1 cm + 3 mrad),
  0.3 false positives per scan, color errors 1–10 %, 50 ms latency. No simulator needed, so it
  runs inside the tests.
- **World driver** (`eval/world_drive.hpp`): the real simulator's vehicle model and IMU, wheel and
  steering models, driven by pure pursuit (Coulter 1992) with a friction-circle speed controller.
  Cone observations come either from the World's LiDAR and camera through `ConePerception`, or
  from its true pose through the synthetic perception model above.
- **Batch least squares** (`eval/batch_slam.hpp`): Gauss-Newton over every scan pose and landmark
  with the true associations, whitened residuals and a sparse LDLᵀ factorization (Eigen
  `SimplicialLDLT`, AMD ordering) of the normal equations (Dellaert and Kaess 2006). The recorder
  feeds it the same dead reckoning and observation covariances the filter uses, so it is the best
  map those measurements allow. Its own test checks the χ² cost at the optimum.
- **NEES** (`eval/consistency.hpp`, Bar-Shalom et al. 2001, 5.4.2) and **map and association
  metrics** by truth id (`eval/map_metrics.hpp`). Truth ids are only passed along: shuffling them
  changes nothing but the truth ids (test `ShufflingTruthIdsChangesOnlyTheTruthIds`).

## 6. Results

```sh
make PRESET=release build
./build/release/bench/estimation/apex_estimation_bench   # ~40 s; writes results/estimation/
uv run --project tools python tools/estimation/plot_latency.py
# The stack runs below (~3 min each), e.g.:
./build/release/apps/apex sweep --config results/estimation/stack/range/sweep.json \
    --out results/estimation/stack/range --quiet
```

Five generated tracks (`generate_track(1..5)`, 109–203 cones), three laps each: lap 1 maps at
up to 8 m/s, mapping switches off one second into lap 2, laps 2–3 localize. Each drive is
estimated twice, with the yaw-rate bias state (the default) and without it. Three setups:

- **world+perception**: the simulator's vehicle model, IMU, wheels and steering, and its LiDAR and
  camera through `ConePerception` (World → ConePerception → StateEstimator). The car is driven by
  the pure-pursuit test driver, racing at up to 22 m/s and 10 m/s² lateral.
- **world**: the same drive, with cone observations made from the true cones by the synthetic
  perception model (noise up to 7 cm at range).
- **synthetic**: the kinematic synthetic drive at full racing speed (22 m/s, 14 m/s²).

Ranges over the five tracks with the default configuration (per-run numbers, with and without
the bias state, in `results/estimation/summary.md`):

| | world+perception | world | synthetic |
|---|---|---|---|
| map RMS by truth id, after lap 1 | 0.009–0.019 m | 0.027–0.108 m | 0.020–0.054 m |
| batch least-squares map RMS | 0.009–0.020 m | 0.032–0.117 m | 0.018–0.047 m |
| duplicates / missed (of 861 cones) | 0 / 0 | 0 / 0 | 0 / 0 |
| pose RMS lap 1 | 0.012–0.015 m | 0.050–0.167 m | 0.023–0.120 m |
| pose RMS racing laps | 0.013–0.043 m | 0.046–0.110 m | 0.021–0.056 m |
| yaw RMS racing laps | 0.01–0.02° | 0.05–0.10° | 0.04–0.08° |
| pose NEES lap 1 / racing laps (expected 3) | 0.5–0.7 / 0.7–7.2 | 1.2–2.3 / 1.7–9.7 | 0.4–1.6 / 1.0–1.8 |
| association precision / recall | 1.0000 / 0.973–0.995 | ≥ 0.9999 / 0.968–0.9998 | ≥ 0.9999 / ≥ 0.9995 |
| velocity RMS vx, vy, r | 0.02–0.05 m/s, 0.03–0.09 m/s, 0.003 rad/s | same signals | 0.03, 0.04 m/s, 0.002 rad/s |
| velocity NEES (expected 3, 95 % bounds ≈ 2.4–3.6) | 1.7–2.8 | same signals | 1.3–1.8 |
| recoveries | 0–4 per run | 0 | 0 |

**The new-landmark range.** Raising it from 20 m to the observation range, 30 m, took the sums
over the 15 runs from 0.730 to 0.516 m of map RMS, from 1.833 to 0.853 m of lap-1 pose RMS and from
0.883 to 0.683 m of racing pose RMS. The largest gains are on lap 1 with the real perception
(0.023–0.092 → 0.012–0.015 m) and on the drifting runs (world seed 5: 0.389 → 0.137 m; synthetic
seed 2: 0.350 → 0.120 m). Four runs got worse by more than 2 mm. Three are maps that had been
below the batch reference and are now at or still below it: world seed 1 (0.084 → 0.108 m, batch
0.117 m; racing pose 0.086 → 0.110 m), world seed 5 (0.036 → 0.053 m, batch 0.051 m) and
world+perception seed 2 (0.008 → 0.017 m, batch 0.016 m). The fourth is world seed 2's lap-1 pose
(0.126 → 0.167 m), whose map improved from 0.093 to 0.033 m. Racing pose NEES rose on three runs
(world+perception seed 2: 6.1 → 7.2; world seed 1: 2.9 → 4.3; world seed 3: 8.8 → 9.7) and fell
on the others. The synthetic perception sees 25 m, so on those drives only the 20–25 m band was
gained; the real perception reports cones beyond 25 m.

The whole stack (`apex sweep` over the baseline's 11 tracks × 3 seeds,
`results/estimation/stack/range`): at 20 m, 32 of 33 runs finish, and every number is identical to
main's baseline. At 30 m, 31 finish. Over the 30 runs that finish at both ranges:

| new-landmark range | map RMS, mean (worst) | duplicates | localization RMS lap 1 / racing, mean | racing lap, mean |
|---|---|---|---|---|
| 20 m | 0.049 m (0.582 m) | 8 | 0.096 / 0.054 m | 23.39 s |
| 30 m | 0.019 m (0.060 m) | 0 | 0.040 / 0.024 m | 23.38 s |

The runs that do not finish are all on track 7, at the start of the racing laps. At 20 m, seed 3
stalls (as on main). At 30 m, seed 3 finishes and seeds 1 and 2 fail: within 1.5 s of the switch
to racing, the commanded steering (0.2–0.3 rad at 13–15 m/s) spins the car. The estimate stays
1–4 cm from the truth until the car has stopped. Main's baseline also fails on that track with the
true state and true cones (the oracle variant: seed 2, off the track in its last lap). The
weakness is the racing start on that track, and which seed fails changes with centimetres of the
map.

Both failures the evaluation reported came from the 20 m range:

- **Three times the sensor noise** (`{"track": {"seed": 1}, "seed": 1, "sensor_noise_scale": 3}`):
  perception no longer finds the four big orange cones at 5.8–6.7 m beside the staged car (none in
  the first 60 scans; 22–33 of 60 at twice the noise). Within 20 m six small cones remain, while
  perception reports about 13 cones per scan at 21–29 m. The car is at rest, so none of those come
  nearer, and the estimator turned away 444 observations as too far to map in 3.5 s. The map stayed
  at six landmarks and the planner found no path. At 30 m the map has 20 landmarks after 3.5 s, 14
  of them beyond 20 m, at 1.8 cm RMS, and over two laps at that noise the map has every cone, no
  duplicates, 5 cm RMS, with the pose at 9 cm RMS on lap 1 and 7 cm racing (test
  `KeepsMappingAtThreeTimesTheSensorNoise`).
- **Track 6, World seed 2: 224 landmarks for 216 cones, 58 cm map RMS.** At 20 m the online pose
  error grew through lap 1 (0.15 m after 26 s, 0.29 m after 36 s, 1.0 m after 46 s) as the yaw
  error reached 8.6 mrad. At the timing line, eight of the first cones fell outside the
  new-landmark gate and were mapped again. With the map bent by a metre, the localization lost
  it at the same place in every racing lap. At 30 m the error peaks at 0.156 m on lap 1, and the
  map has 216 landmarks at 1.7 cm RMS. The stack's telemetry shows the cause: of about 41,300
  observations, 3,051–4,273 per run were rejected at 20 m and 1,062–1,078 at 30 m. The difference
  of about 2,000 is about nine observations per cone: the scans a cone takes on lap 1 to come
  from 30 m to 20 m. Seeds 1 and 3 on the same track, which closed their loops at 20 m, went from
  13 and 8 cm of map RMS to 5.5 and 4.1 cm.

At three times the noise and more the stack still stops after 3 s with "no exploration path"
(`results/estimation/stack/noise`, 6 tracks × 2 seeds; the map's coverage at that point rises from
0.032 to 0.083 with the range). The planner starts its search from an edge midpoint within 6 m of
the car (`exploration.start_radius`). With the big orange cones gone, the nearest edge midpoint
on seed 1 is 11.6 m away. With a 13 m start radius (`results/estimation/stack/noise_start`), 7 of
12 runs finish at 30 m against 6 at 20 m, with map RMS 0.070 against 0.181 m, lap-1 localization
0.129 against 0.282 m, racing localization 0.149 against 0.749 m, and no duplicates against 6.
They still hit 31–33 cones per run, and five runs per range leave the track or stall: that is
planning and control at that noise. At twice the noise all 12 runs finish at both ranges. Against
main's noise ablation, map RMS went from 0.060 to 0.038 m, lap-1 localization from 0.169 to
0.081 m and racing from 0.065 to 0.048 m.

**The bias state** no longer decides world seed 5 at the 30 m range: without it, that run's map
now closes too (0.054 m, no duplicates; at 20 m it had 16 duplicates and 0.942 m). The bias state
still halves that run's lap-1 pose error (0.272 → 0.137 m), and world+perception seed 5's
(0.029 → 0.012 m). Over the 15 runs, map RMS sums to 0.516 m with it against 0.527 m without, the
lap-1 pose RMS to 0.853 against 1.049 m, and the racing pose RMS to 0.683 against 0.692 m. One run
is worse with it by more than 1 cm: synthetic seed 1, lap-1 pose 0.026 → 0.037 m. It does not
bring the localization pose NEES closer to 3. In the racing laps the NEES is 0.7–7.2 with the bias
state and the same without it with the real perception. With the synthetic observations it is
1.7–9.7 against 2.0–9.8. That overconfidence has another cause (below).

What the numbers say:

- **With the real perception the map is as good as the measurements allow.** Perception's
  centers are good to 1–2 cm, and the map comes out at 0.9–1.9 cm RMS, within 1 mm of the batch
  reference on every track; localization on the frozen map holds the car to 1.3–4.3 cm.
  Association never paired an observation with the wrong cone (precision 1.0000 over every run);
  recall is lowest where perception's own recall falls off beyond 20 m.
- **The batch reference is not always better than the filter** on the simulator (world, seeds 1 and
  4). It is the maximum-likelihood map under the stated noise models, and those models treat the
  odometry between scans as independent, which slip and the gyro bias make untrue; on the
  synthetic drive, whose noise matches the models, the batch map is better on four tracks, by up
  to 7 mm, and 1 mm worse on seed 4.
- **Every loop closes without duplicates,** including world seed 5, whose lap-1 drift doubled its
  map with the 20 m range and without the bias state (section 3).
- **Localization is overconfident** with the synthetic observations (pose NEES up to 9.7 in the
  racing laps), less so with the real perception (up to 7.2): the Schmidt update treats the map's
  error correctly in principle, but the map's own covariance after lap 1 inherits the odometry
  model's optimism, and the yaw-rate error is not the part that matters there.
- **The velocity filter is slightly conservative** on the simulator's sensors (NEES 1.7–2.8
  against bounds of about 2.4–3.6), and consistent when its noise model matches the sensors
  (test `VelocityEstimateIsConsistent`, synthetic drives).

**Latency** (Release, Apple M4, one core; `results/estimation/latency.json`, figure below). On
the generated tracks, with up to 203 landmarks, one cone update (association and EKF update)
has p99 0.23–0.82 ms over whole runs, against the 5 ms budget at 200 landmarks; one IMU
prediction p99 0.17 µs against 0.05 ms. The scaling run maps an oval of 388 cones at
10 m/s (about 9 observations per scan):

| landmarks | 50–100 | 150–200 | 200–250 | 300–350 | 350–400 |
|---|---|---|---|---|---|
| update p50 / p99 (ms) | 0.06 / 0.11 | 0.21 / 0.28 | 0.30 / 0.43 | 0.64 / 0.98 | 0.83 / 1.21 |
| IMU prediction p99 (µs) | 0.17 | 0.21 | 0.21 | 0.25 | 0.25 |

The update grows as n², and the bias state's one extra row costs nothing measurable. The 30 m
new-landmark range made it about 20 % slower (0.69 / 0.99 ms at 350–400 landmarks with 20 m,
measured the same day): it now fuses the observations between 20 and 25 m that were turned
away before. Forming
only the lower triangle of the Joseph update (one product of inner dimension 2m instead of two
full ones) took it at 200–250 landmarks from p50 1.6 ms and p99 4.9 ms to 1.1 ms and 2.8 ms in
runs on a machine shared with five other builds (load average 40–120 on 10 cores); the table
above was measured with the machine nearly idle (load average 3–8), which is closer to what the
car's computer gives the estimator.

![Update and prediction cost against landmark count](img/estimation-latency.png)

## 7. Limits

- **The yaw-rate bias state is tuned on these five tracks,** with the 20 m new-landmark range of
  the time. σ and τ come from the scan in section 3; a longer track, a worse gyro or a nominal
  vehicle further from the true one could want other values. At 30 m it costs one run 1.1 cm of
  online lap-1 pose error (synthetic seed 1). A white-noise term sized for the lap's drift was
  tried before the state and rejected: at the one-second horizon of local tracking it is twenty
  times too large, and it made every other run worse (racing pose RMS 0.04 → 0.25 m with
  perception).
- **At three times the sensor noise and more the stack still stops at the start** (section 6),
  though the estimator keeps mapping: the planner's first edge must lie within 6 m, and
  perception then misses the cones that close. With the planner's start radius at 13 m the stack
  explores, and at that noise five of twelve runs leave the track or stall.
- **A lap that drifts beyond the new-landmark gate maps its first cones again,** and nothing
  merges them later: the estimator has no loop-closure search beyond the gates. In the stack's 33
  baseline runs at 30 m, the online pose error on lap 1 peaks at 0.34 m (track 6 seed 3, flowing
  seed 3), against 1.03 m on track 6 seed 2 at 20 m, which mapped eight cones again.
- **The cone source matters.** The tuning (gates, confirmation count, noise floors) holds for both
  the real perception and the synthetic stand-in, but the stand-in's 3–7 cm noise at range is
  what exposes the drift above; perception's 1–2 cm centers keep the map tight enough to close
  every loop here. Perception's own failure modes on harder scenes (merged or split clusters,
  correlated false positives) are not in these runs: it reported no false positives on them.
- **The World runs use a test driver,** pure pursuit at 10 m/s² of lateral acceleration, not the
  stack's MPC at the limit. Sliding beyond the single-track model is outside what the velocity
  filter models: above 90 % of grip the lateral update is skipped and vy is integrated from the
  IMU alone. The recovery handles the consequences, it does not prevent them.
- **The velocity filter is conservative on the simulator's sensors** (velocity NEES below its
  bounds in the table), on purpose: its noise model must also cover a nominal vehicle that differs
  from the true one, which the runner's model-mismatch ablations will do. It is consistent
  (NEES inside its 95 % bounds) when its noise model matches the sensors, on synthetic drives.
- **Dense EKF.** The update is O(n² m) in the state size n: fine for the 200 landmarks of a long
  track (table above), quadratic beyond. A sparse information filter or a sliding-window smoother
  would scale further; the batch solver here is the start of the latter.
- **Linearization.** EKF-SLAM is known to become overconfident in heading over long runs because
  its Jacobians are evaluated at changing estimates (Huang, Mourikis and Roumeliotis 2010). On
  these laps the map's NEES stays near or below its dimension, but nothing in the filter corrects
  for it (the first-estimates Jacobian would).
- **One hypothesis.** Association commits to the best joint hypothesis of each scan; a wrong one
  is not revisited. The new-landmark gate makes the common failure (a duplicate) rare, not
  impossible.
- **The localization map is frozen,** positions included (Schmidt update). A landmark mapped
  badly on lap 1 stays bad.

## References

- Bar-Shalom, Li and Kirubarajan, *Estimation with Applications to Tracking and Navigation*,
  Wiley, 2001.
- Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01,
  Carnegie Mellon University, 1992.
- Dellaert and Kaess, "Square Root SAM: Simultaneous localization and mapping via square root
  information smoothing", *International Journal of Robotics Research* 25(12), 2006.
- Dissanayake, Newman, Clark, Durrant-Whyte and Csorba, "A solution to the simultaneous
  localization and map building (SLAM) problem", *IEEE Transactions on Robotics and Automation*
  17(3), 2001.
- Huang, Mourikis and Roumeliotis, "Observability-based rules for designing consistent EKF SLAM
  estimators", *International Journal of Robotics Research* 29(5), 2010.
- Huber, "Robust estimation of a location parameter", *Annals of Mathematical Statistics* 35(1),
  1964.
- Maybeck, *Stochastic Models, Estimation, and Control*, vol. 1, Academic Press, 1979.
- Neira and Tardós, "Data association in stochastic mapping using the joint compatibility test",
  *IEEE Transactions on Robotics and Automation* 17(6), 2001.
- Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann, 2012.
- Press, Teukolsky, Vetterling and Flannery, *Numerical Recipes*, 3rd ed., Cambridge University
  Press, 2007.
- Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer, 2012.
- Schmidt, "Application of state-space methods to navigation problems", *Advances in Control
  Systems* 3, 1966.
- Smith, Self and Cheeseman, "Estimating uncertain spatial relationships in robotics", in
  *Autonomous Robot Vehicles*, Springer, 1990.
