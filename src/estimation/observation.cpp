#include "apex/estimation/observation.hpp"

namespace apex::estimation {

PreparedObservation prepare_observation(const ConeObservation& cone, const SlamConfig& config) {
    PreparedObservation out;
    if (!cone.position.allFinite() || !cone.covariance.allFinite()) return out;
    if (cone.position.norm() > config.max_observation_range) {
        out.status = ObservationStatus::OutOfRange;
        return out;
    }
    Mat2 r = config.observation_noise_scale * 0.5 * (cone.covariance + cone.covariance.transpose());
    r.diagonal().array() += config.min_observation_std * config.min_observation_std;
    // Sylvester's criterion for a symmetric 2 x 2.
    if (!(r(0, 0) > 0.0) || !(r(0, 0) * r(1, 1) - r(0, 1) * r(1, 0) > 0.0)) return out;
    out.status = ObservationStatus::Usable;
    out.model = {cone.position, r};
    return out;
}

}  // namespace apex::estimation
