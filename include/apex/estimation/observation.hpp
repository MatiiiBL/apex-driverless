#pragma once

#include "apex/core/messages.hpp"
#include "apex/estimation/association.hpp"
#include "apex/estimation/slam_config.hpp"

namespace apex::estimation {

enum class ObservationStatus {
    Usable,
    Invalid,     // non-finite, or a covariance that is not positive definite
    OutOfRange,  // beyond max_observation_range
};

struct PreparedObservation {
    ObservationStatus status = ObservationStatus::Invalid;
    ObservationModel model;  // meaningful when status is Usable
};

// What the SLAM fuses for one cone of perception's output: its position and perception's
// covariance, symmetrized, scaled by observation_noise_scale, plus min_observation_std^2 on
// each axis (a noise floor: a cone's center is never known better than that, whatever a
// cluster's point count suggests). Shared by the estimator and the batch reference so that both
// see the same measurements.
[[nodiscard]] PreparedObservation prepare_observation(const ConeObservation& cone,
                                                      const SlamConfig& config);

}  // namespace apex::estimation
