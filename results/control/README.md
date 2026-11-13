# Control results

Written by the control benchmarks (Release build, `make PRESET=release build`), quoted in
[docs/control.md](../../docs/control.md):

| File | Written by | Contents |
|---|---|---|
| `qp_bench.json`, `qp_bench.md` | `build/release/bench/control/apex_control_qp_bench` | The MPC's QPs at horizons 20-60, solved by the Riccati interior-point method and by OSQP at the same tolerance: time per QP, iterations, KKT residuals, solution differences |
| `closed_loop.json`, `closed_loop.md` | `build/release/bench/control/apex_control_closed_loop_eval` | Pure pursuit, Stanley and MPC on the simulator's vehicle, two laps per track: lap times, lateral error, steering smoothness, penalties, compute time, robustness sweeps |
| `trace_<controller>.csv` | the same | Per control call on the "mixed" layout: time, station, lateral error, speed, reference speed, commands |
| `stack/`, `stack_oracles/` | `build/release/apps/apex sweep --config <dir>/sweep.json --out <dir>` | The full stack with each controller, four laps of generated tracks 1-8 and the oval, without and with both oracles: `summary.md`, every run's result in `runs.json` |

Both executables take `--out <dir>` (default `results/control`, relative to the working
directory) and are deterministic; their timings depend on the machine and its load.
