#pragma once

#include <cstdint>
#include <limits>
#include <type_traits>
#include <variant>
#include <vector>

#include "apex/core/angles.hpp"
#include "apex/core/messages.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/track.hpp"
#include "apex/core/vehicle.hpp"

// Synthetic drives for testing and evaluating the estimator without the simulator: a car that
// follows a track's centerline exactly, with sensor messages consistent with that motion. The
// truth is kinematic plus a steady-state tire slip model; the sensors add noise, bias, wheel
// slip, missed and false cone detections, color errors and the perception latency. This is a
// test fixture with ground truth, not part of the car's software.
namespace apex::estimation::eval {

// The speed profile: the fastest the lateral and longitudinal limits allow, slower on lap 1
// (exploration) than on the later laps (racing).
struct DriveProfile {
    int laps = 1;
    double exploration_speed = 8.0;          // m/s, lap 1
    double exploration_lateral_accel = 6.0;  // m/s^2
    double racing_speed = 22.0;              // m/s, laps 2+
    double racing_lateral_accel = 14.0;      // m/s^2
    double max_accel = 8.0;                  // m/s^2
    double max_brake = 12.0;                 // m/s^2
    double hold_at_start = 1.0;              // s at rest before driving off
};

// The sensor models of the synthetic drive. Defaults are of the order of the real sensors'.
struct SyntheticSensors {
    double accel_noise = 0.05;     // m/s^2 per sample
    double gyro_noise = 0.002;     // rad/s per sample
    double gyro_bias = 0.004;      // rad/s at the start
    double gyro_bias_walk = 5e-5;  // rad/s/sqrt(s)
    double wheel_noise = 0.03;     // m/s per sample
    // Slip ratio = mu_x / stiffness, growing without bound as mu_x approaches the tire's mu
    // (wheel spin and lock-up near the limit).
    double slip_stiffness = 20.0;
    double steering_noise = 0.002;  // rad per sample

    double cone_range = 25.0;             // m, from the LiDAR
    double cone_fov = deg_to_rad(180.0);  // centered on +x
    double detection_near = 0.97;         // detection probability at 0 m ...
    double detection_far = 0.6;           // ... and at cone_range (quadratic in between)
    double range_noise = 0.02;            // m, std along the ray ...
    double range_noise_per_m = 0.002;     // ... growing with range
    double tangential_noise = 0.01;       // m, std across the ray ...
    double bearing_noise = 0.003;         // rad, ... growing with range
    double false_positive_rate = 0.3;     // per scan (Poisson), uniform over the field of view
    double camera_range = 20.0;           // m: farther cones come out Unknown
    double camera_fov = deg_to_rad(100.0);
    double color_error_near = 0.01;  // probability of a wrong color at 0 m ...
    double color_error_far = 0.10;   // ... and at camera_range
    // Reported covariance / true covariance: 1 for an honest perception.
    double reported_covariance_scale = 1.0;
    double latency = 0.05;  // s from a scan's stamp to the delivery of its observations
};

struct TruthSample {
    Time stamp = 0.0;
    Pose2 pose;
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
    double ax = 0.0;  // body-frame specific force
    double ay = 0.0;
    double steer = 0.0;
    double gyro_bias = 0.0;
    double distance = 0.0;  // along the centerline since the start
    int lap = 1;            // 1 until the second timing-line crossing, and so on
};

using EstimatorMessage = std::variant<ImuSample, WheelSpeeds, SteeringSample, ConeObservations>;

struct TimedMessage {
    Time delivery = 0.0;
    EstimatorMessage message;
};

struct SyntheticRun {
    std::vector<TimedMessage> messages;  // in delivery order, as a World would interleave them
    std::vector<TruthSample> truth;      // every millisecond
    // Per track cone: when it was detected for the third time (+inf if never), i.e. from when a
    // landmark for it may be expected.
    std::vector<Time> mappable_since;
    Time lap1_end = std::numeric_limits<Time>::infinity();  // second timing-line crossing

    // The truth at the millisecond tick nearest t (clamped to the run).
    [[nodiscard]] const TruthSample& truth_at(Time t) const;
};

// One scan of the synthetic perception from `pose`: each true cone within cone_range and
// cone_fov of the LiDAR mount, detected with the range-dependent probability, its position
// perturbed by noise along and across the ray from the mount (and reported with that
// covariance), its color from the camera model; plus Poisson false positives. Stands in for the
// perception module, from the World's truth as well as from a synthetic drive.
[[nodiscard]] ConeObservations observe_cones(const Track& track, const Pose2& pose, Time stamp,
                                             const SyntheticSensors& sensors, Rng& rng);

// Counts each true cone's detections in `scan` and stamps the third one into mappable_since.
void note_detections(const ConeObservations& scan, std::vector<std::uint32_t>& detections,
                     std::vector<Time>& mappable_since);

// Throws std::invalid_argument for a profile that cannot be driven.
[[nodiscard]] SyntheticRun make_synthetic_run(const Track& track, const VehicleParams& vehicle,
                                              const DriveProfile& profile,
                                              const SyntheticSensors& sensors, std::uint64_t seed);

// Hands one message to the matching entry point of `sink`: a StateEstimator, a BatchRecorder,
// or anything else with on_imu, on_wheels, on_steering and on_cones.
template <class Sink>
void deliver(Sink& sink, const TimedMessage& message) {
    std::visit(
        [&](const auto& m) {
            using T = std::decay_t<decltype(m)>;
            if constexpr (std::is_same_v<T, ImuSample>) {
                sink.on_imu(m);
            } else if constexpr (std::is_same_v<T, WheelSpeeds>) {
                sink.on_wheels(m);
            } else if constexpr (std::is_same_v<T, SteeringSample>) {
                sink.on_steering(m);
            } else {
                sink.on_cones(m);
            }
        },
        message.message);
}

}  // namespace apex::estimation::eval
