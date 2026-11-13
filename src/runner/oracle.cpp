#include "apex/runner/oracle.hpp"

#include <algorithm>
#include <cmath>

namespace apex::runner {

ConeObservations oracle_observations(std::span<const Cone> cones, const VehicleState& truth,
                                     const LidarGeometry& lidar,
                                     const PerceptionOracleConfig& config, Time stamp, Rng& rng) {
    ConeObservations result;
    result.stamp = stamp;
    const Vec2 mount = lidar.mount.head<2>();
    for (const Cone& cone : cones) {
        const Vec2 body = truth.pose.to_local(cone.position);
        const Vec2 from_lidar = body - mount;
        const double range = from_lidar.norm();
        if (range < lidar.min_range || range > config.max_range) continue;
        const double azimuth = std::atan2(from_lidar.y(), from_lidar.x());
        if (azimuth < lidar.azimuth_min || azimuth > lidar.azimuth_max) continue;
        const double sigma = config.sigma + config.sigma_per_meter * range;
        ConeObservation o;
        o.position = body + Vec2{rng.normal(0.0, sigma), rng.normal(0.0, sigma)};
        // A floor keeps the covariance invertible for a noise-free oracle.
        o.covariance = Mat2::Identity() * std::max(sigma * sigma, 1e-6);
        o.color = cone.color;
        o.color_confidence = 1.0;
        o.truth_id = cone.id;
        result.cones.push_back(o);
    }
    return result;
}

StateEstimate oracle_state(const VehicleState& truth, Time stamp) {
    StateEstimate s;
    s.stamp = stamp;
    s.pose = truth.pose;
    s.vx = truth.vx;
    s.vy = truth.vy;
    s.yaw_rate = truth.yaw_rate;
    s.pose_covariance = Mat3::Zero();
    s.valid = true;
    return s;
}

}  // namespace apex::runner
