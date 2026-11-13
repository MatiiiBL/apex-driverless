#pragma once

#include <cstddef>
#include <cstdint>

#include <nlohmann/json_fwd.hpp>

namespace apex::estimation {

// Tunables of the SLAM back end: motion noise, observation handling and landmark management.
// docs/estimation.md derives the defaults.
struct SlamConfig {
    // The pose moves by the velocity filter's estimate. Its error is modelled as a Gauss-Markov
    // process with this correlation time, whose integral grows like white noise of density
    // 2 tau Sigma_v, plus the floors below for what the filter does not model (accelerometer
    // bias, the IMU's offset from the CoG, the bias error's slow yaw drift). Most of the
    // velocity error is wheel-noise jitter that averages out within milliseconds; 50 ms leaves a
    // margin for the slower errors of slip.
    double velocity_correlation_time = 0.05;  // s
    double position_noise_density = 0.01;     // m/sqrt(s)
    double yaw_noise_density = 0.001;         // rad/sqrt(s)
    // The car is told its start pose (the map frame is defined there); this is how well.
    double initial_position_std = 0.005;  // m
    double initial_yaw_std = 0.0005;      // rad

    // Perception's covariances are scaled, then floored, before use.
    double observation_noise_scale = 1.0;
    double min_observation_std = 0.02;     // m
    double max_observation_range = 30.0;   // m, observations beyond are ignored
    double max_new_landmark_range = 20.0;  // m, only nearer ones may start a landmark
    double min_landmark_separation = 0.5;  // m, no new landmark nearer than this to another
    std::uint32_t confirm_detections = 3;  // scans a new landmark must be seen in
    double tentative_timeout = 0.35;       // s, unconfirmed landmarks unseen this long go
    std::size_t max_landmarks = 1000;      // the state stops growing here

    // Observations may be stamped up to this long before the newest IMU sample; older ones are
    // dropped. Odometry is kept for this span.
    double history_duration = 0.5;  // s
};

void to_json(nlohmann::json& j, const SlamConfig& config);
void from_json(const nlohmann::json& j, SlamConfig& config);

}  // namespace apex::estimation
