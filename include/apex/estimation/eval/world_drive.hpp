#pragma once

#include "apex/core/track.hpp"
#include "apex/estimation/eval/synthetic.hpp"
#include "apex/sim/world.hpp"

// Drives the real simulator for the estimation's evaluation. This target links apex::sim, as the
// tests do; the estimator itself never sees it.
namespace apex::estimation::eval {

// Runs a World around its track for profile.laps laps (timed by its RaceMonitor) and records
// what the estimator gets: the World's own IMU, wheel and steering messages, and cone
// observations made by observe_cones from the true pose at each LiDAR stamp, standing in for
// perception until it is merged, delivered `perception.latency` after their stamp.
//
// The car follows the centerline by pure pursuit on the true state (Coulter, "Implementation of
// the pure pursuit path tracking algorithm", CMU-RI-TR-92-01, Carnegie Mellon University,
// 1992), at the profile's speed limits with a braking preview, so the IMU and the wheels see the
// vehicle model's real dynamics: load transfer, combined slip, actuator lag. The truth is
// recorded every tick; TruthSample::gyro_bias is NaN, because the World keeps its IMU bias to
// itself. Throws std::invalid_argument for an invalid profile and std::runtime_error if the car
// does not finish within ten minutes of simulated time.
[[nodiscard]] SyntheticRun make_world_run(const Track& track, const sim::WorldConfig& config,
                                          const DriveProfile& profile,
                                          const SyntheticSensors& perception);

}  // namespace apex::estimation::eval
