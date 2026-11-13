# Simulator results

Produced by the Release build on an Apple M4 (one core per run) with:

```sh
make PRESET=release build
./build/release/bench/sim/apex_vehicle_validation --out results/sim/vehicle_validation.json
./build/release/bench/sim/apex_sim_bench --out results/sim/bench.json
```

The machine was shared with other jobs (load average ~130 on 10 cores) when `bench.json` was
recorded, so its tails (p99, max) include preemption; the medians were stable across runs. The
validation numbers are deterministic.

## Vehicle model against its references (`vehicle_validation.json`)

| Check | Reference | Result | Test threshold |
|---|---|---|---|
| Low-speed path curvature, 1-2 m/s, steer 0.1-0.4 rad | kinematic bicycle | 0.07% max error | 0.5% |
| Steady-state yaw rate at 2 m/s^2, 8-20 m/s, three cars | linear single-track | 0.19% max error | 2% |
| Coast-down from 25 m/s, 40 s | closed-form drag + rolling | 1.7e-12 m, 8.5e-14 m/s | 1e-8 relative |
| Flat-out acceleration, 40 kW car, 14-26 m/s | (P/v - drag - rolling) / m | 2.7e-15 m/s^2 | 1e-6 m/s^2 |
| Peak lateral acceleration, 10/15/20 m/s | mu (g + downforce / m) | 0.885-0.902 of it (0.903-0.947 at half friction) | > 0.85, never above |
| Step halved twice on a 5 s manoeuvre | RK4 order 4 | observed order 4.04 | 4 +/- 0.3 |

## Performance (`bench.json`)

| Work | Result |
|---|---|
| Track generation, 200 seeds | 14.0 candidates per track; p50 18.8 ms per track under load (4-5 ms on a quiet machine, dev build) |
| LiDAR scan, 57,664 rays, 245 poses on three tracks | p50 1.54 ms, p90 2.70 ms; 41,700 returns (1,000 on cones) |
| Same scans, every ray against every cone | p50 62 ms (culling: 42x faster on average) |
| Camera frame, 611 poses | p50 15 us, p99 104 us; 9.7 detections |
| Vehicle model step (RK4, 1 ms) | 1.2 us |
| World, one lap of `generated-1` with every sensor | 45.7 s simulated in 1.07 s: 42.7x real time; tick p50 3.0 us, p99 1.09 ms |
