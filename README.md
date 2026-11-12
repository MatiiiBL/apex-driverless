# Apex

An autonomous racing stack for a Formula Student Driverless car, written from scratch in C++20,
with the simulator it races in. On an unknown track marked only by cones, the car maps the
circuit on its first lap with LiDAR, camera and EKF-SLAM, then races the remaining laps on a
minimum-curvature raceline with model predictive control.

Work in progress: see [docs/design.md](docs/design.md).

```sh
make check   # build and run every test (CMake presets: dev, asan, tsan, release)
```
