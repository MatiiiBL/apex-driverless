#include "apex/runner/oracle.hpp"

#include <cmath>
#include <cstddef>

#include <Eigen/Cholesky>

#include "apex/perception/cone_model.hpp"

namespace apex::runner {

namespace {

// Added to every covariance so it stays positive definite (and drawable) even for a covariance
// model configured to zero: 0.01 mm of standard deviation, far below any modeled term.
constexpr double kCovarianceFloor = 1e-10;  // m^2

}  // namespace

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
        const double returns = perception::expected_cone_returns(
            lidar, cone_shape(cone.color), range, lidar.mount.z(), config.ground_threshold);
        if (returns < static_cast<double>(config.min_points)) continue;

        const auto points = static_cast<std::size_t>(std::lround(returns));
        const Mat2 covariance =
            perception::observation_covariance(config.covariance, from_lidar, points) +
            kCovarianceFloor * Mat2::Identity();
        const Mat2 factor = covariance.llt().matrixL();
        ConeObservation o;
        o.position = body + factor * Vec2{rng.normal(), rng.normal()};
        o.covariance = covariance;
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
