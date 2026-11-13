# Perception evaluation

`build/release/bench/perception/apex_perception_eval` (Release) over 1204 scans at poses along 8 generated tracks; synthetic sensors (exact ray caster, 2 cm range noise, 2% dropout). Reproduce and read with docs/perception.md.

| Range (m) | Cones in view | Recall | Recall of hit cones | Precision | Position RMS (cm) | Radial RMS (cm) | Tangential RMS (cm) | NEES | NEES > 5.99 |
|---|---|---|---|---|---|---|---|---|---|
| 0-10 | 5574 | 0.991 | 0.997 | 1.0000 | 0.7 | 0.6 | 0.4 | 1.88 | 2.2% |
| 10-20 | 7215 | 0.956 | 0.984 | 1.0000 | 1.4 | 1.1 | 0.9 | 2.34 | 8.3% |
| 20-30 | 9355 | 0.796 | 0.832 | 1.0000 | 1.8 | 1.4 | 1.2 | 1.75 | 4.6% |
| 30+ | 20026 | 0.006 | 0.007 | 1.0000 | 2.7 | 2.3 | 1.5 | 2.38 | 8.0% |
| all | 42170 | 0.474 | 0.537 | 1.0000 | 1.5 | 1.1 | 0.9 | 1.99 | 5.2% |

False positives per scan: 0.0000. NEES average: 1.99 (2 expected), above the chi-square 95% bound in 5.2% of cases (5% expected).

| Camera frame | Coverage | Accuracy | Mean confidence, right / wrong |
|---|---|---|---|
| 25 ms early, yaw compensated | 0.769 | 0.9416 | 0.59 / 0.61 |
| at the scan's stamp | 0.790 | 0.9469 | 0.67 / 0.64 |
| 25 ms early, not compensated | 0.762 | 0.9356 | 0.54 / 0.54 |
| 50 ms early, yaw compensated | 0.750 | 0.9376 | 0.56 / 0.56 |
| 50 ms early, not compensated | 0.755 | 0.9341 | 0.50 / 0.48 |
| 25 ms early, compensated; detector never confuses colors, no false positives | 0.759 | 0.9951 | 0.60 / 0.28 |

Big orange cones fitted as big: 565 of 676; small cones fitted as big: 19 of 19325.

`on_lidar` latency: p50 228 us, p99 476 us, max 894 us.

Covariance model fitted to these errors (`fit_variance_model`):

```json
{
  "radial": {
    "constant": 2.0928585017542978e-05,
    "inv_points": 0.000367265387630852,
    "range_sq": 1.4009668000332335e-07,
    "range_sq_inv_points": 0.0
  },
  "tangential": {
    "constant": 1.0632069153853655e-05,
    "inv_points": 0.0,
    "range_sq": 1.1088324739232119e-07,
    "range_sq_inv_points": 6.219143055771679e-07
  }
}
```
