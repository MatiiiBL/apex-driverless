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

struct LidarNoise {};
struct CameraNoise {};

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
[[nodiscard]] LidarScan simulate_lidar(const LidarGeometry& geometry, const LidarNoise& noise,
                                       const VehicleState& state, std::span<const Cone> cones,
                                       Time stamp, Rng& rng);

// The cone detector's output for one camera frame.
[[nodiscard]] CameraFrame simulate_camera(const CameraGeometry& geometry, const CameraNoise& noise,
                                          const VehicleState& state, std::span<const Cone> cones,
                                          Time stamp, Rng& rng);

}  // namespace apex::sim
