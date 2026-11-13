# Perception evaluation

`build/release/bench/perception/apex_perception_eval` (Release) over 1204 scans at poses along 8 generated tracks; sim::simulate_lidar (2 cm range noise, 2% dropout) and sim::simulate_camera with their default noise. Reproduce and read with docs/perception.md.

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
| 25 ms early, yaw compensated | 0.668 | 0.9439 | 0.62 / 0.57 |
| at the scan's stamp | 0.700 | 0.9507 | 0.78 / 0.67 |
| 25 ms early, not compensated | 0.662 | 0.9452 | 0.58 / 0.49 |
| 50 ms early, yaw compensated | 0.654 | 0.9446 | 0.55 / 0.49 |
| 50 ms early, not compensated | 0.656 | 0.9399 | 0.50 / 0.45 |
| 25 ms early, compensated; detector never confuses colors, no false positives | 0.662 | 0.9975 | 0.62 / 0.03 |

Big orange cones fitted as big: 565 of 676; small cones fitted as big: 19 of 19325.

`on_lidar` latency: p50 215 us, p99 484 us, max 1154 us.

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
