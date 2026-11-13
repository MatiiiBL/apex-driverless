#pragma once

#include <span>

#include "apex/core/messages.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/sim/vehicle_model.hpp"

// Sensor models. Each simulate_* function is pure given its Rng, so tests of the car's modules
// can produce realistic measurements without running a World. The sim module owns the noise
// structs below and fills them with documented, realistic defaults (docs/sim.md, "Sensors").
namespace apex::sim {

// A 64-channel spinning automotive LiDAR, the class of the Velodyne HDL-64E or Hesai Pandar64,
// whose datasheets give a range precision of about 2 cm (1 sigma). Returns from dark, wet or
// grazing surfaces are lost now and then.
struct LidarNoise {
    double range_stddev = 0.02;  // m
    double dropout = 0.02;       // probability that a return is lost
};

// A cone detector (a single-stage CNN such as YOLO on a global-shutter camera, as FS teams run
// it) reporting bearing, monocular range from the cone's apparent size, color and a score.
// Detection probability falls off logistically with range, p0 / (1 + exp((r - r50) / w)). Range
// error grows as r^2, as depth from an object's pixel height does (its height in pixels goes as
// 1 / r). Colors are confused more often far away, where cones are a few pixels tall.
struct CameraNoise {
    double min_range = 2.0;                  // m: closer cones are hidden below the car's nose
    double detection_probability = 0.98;     // p0, near the car
    double detection_half_range = 22.0;      // r50 (m): half the cones in view are found here
    double detection_falloff = 3.0;          // w (m)
    double bearing_stddev = 0.003;           // rad, about 3 pixels
    double range_stddev = 0.05;              // m, at zero range
    double range_stddev_quadratic = 0.0015;  // 1/m: stddev = range_stddev + this * r^2
    double confusion_near = 0.01;            // probability of a wrong color at zero range
    double confusion_far = 0.10;             // ... at the camera's max_range (linear between)
    double confidence_stddev = 0.05;         // spread of the score around p(r)
    double false_positives = 0.3;            // mean per frame (Poisson), truth_id -1
};

// A MEMS IMU of the industrial class FS teams fit, at 200 Hz. The datasheet white noise of such
// parts is ~0.01 m/s^2 and ~0.001 rad/s per sample at this rate; on a car, chassis vibration
// dominates, hence the larger defaults. Each axis has a turn-on bias, drawn once per run, that
// then drifts as a random walk.
struct ImuNoise {
    double accel_stddev = 0.05;      // m/s^2, white noise per sample
    double gyro_stddev = 0.002;      // rad/s, white noise per sample
    double accel_bias = 0.02;        // m/s^2, standard deviation of the turn-on bias
    double gyro_bias = 0.002;        // rad/s (0.11 deg/s)
    double accel_bias_walk = 0.002;  // m/s^2 per sqrt(s), bias random walk
    double gyro_bias_walk = 5e-5;    // rad/s per sqrt(s)
};

// Hall-effect wheel speed sensors timing the edges of a toothed ring, averaged over each axle.
// Under drive and braking the tires slip: the wheels turn at the axle's ground speed times
// (1 + Fx / (slip_stiffness Fz)), so a slick reaching mu = 1.6 slips by about 8%.
struct WheelNoise {
    double stddev = 0.05;          // m/s per reading
    double slip_stiffness = 20.0;  // longitudinal force per unit slip ratio, over normal load
};

// Steering angle sensor on the column, mapped to the front wheel angle; the stddev includes the
// play between column and wheels.
struct SteeringNoise {
    double stddev = 0.002;  // rad (0.11 deg)
};

struct SensorNoise {
    LidarNoise lidar;
    CameraNoise camera;
    ImuNoise imu;
    WheelNoise wheels;
    SteeringNoise steering;
};

// Every noise standard deviation and bias magnitude multiplied by `factor`; probabilities,
// rates and physical parameters (dropout, detection and confusion probabilities, false-positive
// rate, slip stiffness) unchanged. Factor 0 gives noiseless measurements, 1 the defaults' level.
// For the runner's sensor-noise ablation. Throws std::invalid_argument for a negative or
// non-finite factor.
[[nodiscard]] SensorNoise scale_noise(const SensorNoise& noise, double factor);

// Throws std::invalid_argument naming the first noise parameter that is negative or non-finite.
void validate_noise(const SensorNoise& noise);

// The IMU's biases, which the World carries from one sample to the next.
struct ImuBias {
    double ax = 0.0;  // m/s^2
    double ay = 0.0;
    double yaw_rate = 0.0;  // rad/s
};

// A turn-on bias for each axis.
[[nodiscard]] ImuBias draw_imu_bias(const ImuNoise& noise, Rng& rng);

// One IMU sample of the truth: the body-frame specific force at the CoG (on flat ground, the
// state's ax, ay) and the yaw rate, plus bias and white noise. First advances the bias by its
// random walk over `period` seconds since the previous sample.
[[nodiscard]] ImuSample simulate_imu(const ImuNoise& noise, const VehicleState& state, Time stamp,
                                     double period, ImuBias& bias, Rng& rng);

// Each axle's circumferential wheel speed: the ground speed of the axle center along its wheels
// (vx cos d + (vy + lf r) sin d at the front, vx at the rear; left and right differences from
// yaw cancel in the average), times one plus the slip from that axle's longitudinal force in
// `tires`, plus noise. Never negative: the sensors measure speed, not direction.
[[nodiscard]] WheelSpeeds simulate_wheel_speeds(const VehicleParams& vehicle,
                                                const WheelNoise& noise, const VehicleState& state,
                                                const TireForces& tires, Time stamp, Rng& rng);

// The actual front wheel angle plus noise.
[[nodiscard]] SteeringSample simulate_steering(const SteeringNoise& noise,
                                               const VehicleState& state, Time stamp, Rng& rng);

// One LiDAR revolution against the cones (3D cone bodies over a flat ground plane at z = 0).
// One ray per channel and azimuth step, column by column from azimuth_min; the nearest hit
// within [min_range, max_range] is returned, so cones shadow each other and the ground behind
// them. Each return is then dropped with probability `dropout` or has Gaussian range noise
// added along its ray. truth_id is the cone's id, -1 for ground. Rays are tested only against
// the cones whose azimuth and elevation extent they cross, nearest first, which makes a
// 57,600-ray scan cost about a millisecond; simulate_lidar_brute_force (sim/lidar.hpp) is the
// reference it is tested against.
[[nodiscard]] LidarScan simulate_lidar(const LidarGeometry& geometry, const LidarNoise& noise,
                                       const VehicleState& state, std::span<const Cone> cones,
                                       Time stamp, Rng& rng);

// The cone detector's output for one camera frame. A cone in the horizontal field of view,
// between min_range and the camera's max_range, is a candidate unless the line of sight from
// the camera to the middle of its body passes through another cone. Each candidate is detected
// with the range-dependent probability of CameraNoise, with bearing and range noise, a color
// confusion and a score; then a Poisson number of false positives (truth_id -1) is spread
// uniformly over the field of view and range. Detections are sorted by bearing, as a detector
// lists them across the image, so their order reveals nothing about the truth.
[[nodiscard]] CameraFrame simulate_camera(const CameraGeometry& geometry, const CameraNoise& noise,
                                          const VehicleState& state, std::span<const Cone> cones,
                                          Time stamp, Rng& rng);

}  // namespace apex::sim
