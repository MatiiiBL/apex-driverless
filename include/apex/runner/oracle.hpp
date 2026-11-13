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
// `stamp`, with the car in `truth`. Every cone within `config.max_range` of the LiDAR and
// inside its azimuth limits, occluded or not, with its true color (confidence 1) and truth id,
// at the true body-frame position plus N(0, sigma(r)^2 I), sigma(r) = sigma + sigma_per_meter r.
[[nodiscard]] ConeObservations oracle_observations(std::span<const Cone> cones,
                                                   const VehicleState& truth,
                                                   const LidarGeometry& lidar,
                                                   const PerceptionOracleConfig& config, Time stamp,
                                                   Rng& rng);

// Perfect state: the true state as a StateEstimate, with zero covariance.
[[nodiscard]] StateEstimate oracle_state(const VehicleState& truth, Time stamp);

}  // namespace apex::runner
