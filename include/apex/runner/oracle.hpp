#pragma once

#include <span>

#include "apex/core/messages.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/runner/config.hpp"

// The ablation oracles of docs/design.md, section 8: ground truth in the form the car's software
// would otherwise compute it, handed over through stack::Driver::on_oracle_*.
namespace apex::runner {

// Perfect perception: what an ideal cone detector would report for a LiDAR scan stamped
// `stamp`, with the car in `truth`. Every cone inside the LiDAR's azimuth limits and within
// `config.max_range` that would give at least `config.min_points` returns if unoccluded
// (perception::expected_cone_returns: the range at which perception can see a cone at all), with
// its true color (confidence 1) and truth id. Occlusion is ignored. The position is the true
// body-frame center plus noise drawn from perception's nominal covariance model for that range
// and that expected return count, N(0, S) with S = perception::observation_covariance, and S is
// the covariance reported.
[[nodiscard]] ConeObservations oracle_observations(std::span<const Cone> cones,
                                                   const VehicleState& truth,
                                                   const LidarGeometry& lidar,
                                                   const PerceptionOracleConfig& config, Time stamp,
                                                   Rng& rng);

// Perfect state: the true state as a StateEstimate, with zero covariance.
[[nodiscard]] StateEstimate oracle_state(const VehicleState& truth, Time stamp);

}  // namespace apex::runner
