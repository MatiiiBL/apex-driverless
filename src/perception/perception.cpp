#include "apex/perception/perception.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "apex/core/log.hpp"

namespace apex::perception {
namespace {

void validate(const VarianceModel& model, const char* name) {
    const bool non_negative = model.constant >= 0.0 && model.range_sq >= 0.0 &&
                              model.inv_points >= 0.0 && model.range_sq_inv_points >= 0.0;
    // A positive floor keeps every covariance positive definite.
    if (!non_negative || !(model.constant > 0.0)) {
        throw std::invalid_argument(std::string("CovarianceModel.") + name +
                                    ": terms must be non-negative, the constant positive");
    }
}

bool finite(const LidarPoint& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

}  // namespace

struct ConePerception::Impl {
    Impl(const PerceptionConfig& c, const SensorRig& r)
        : config(c),
          rig(r),
          ground(c.ground, r.lidar),
          clusterer(c.clustering, r.lidar),
          validator(c.cone_check, r.lidar, c.ground.threshold),
          fusion(c.camera, r.camera) {
        if (!(c.center_fit.range_sigma > 0.0) || c.center_fit.iterations < 0 ||
            !(c.center_fit.big_cone_height > 0.0) ||
            !std::isfinite(c.center_fit.big_cone_log_odds)) {
            throw std::invalid_argument(
                "CenterFitConfig: range_sigma and big_cone_height must be positive, iterations "
                ">= 0, big_cone_log_odds finite");
        }
        validate(c.covariance.radial, "radial");
        validate(c.covariance.tangential, "tangential");
    }

    ConeObservations on_lidar(const LidarScan& scan, double yaw_rate);
    void on_camera(const CameraFrame& frame);

    PerceptionConfig config;
    SensorRig rig;
    GroundEstimator ground;
    GridClusterer clusterer;
    ConeValidator validator;
    CameraFusion fusion;
    PerceptionStats stats;
    bool ground_ok = true;

    // Per-scan scratch, kept to avoid reallocating every scan.
    std::vector<ObstaclePoint> obstacles;
    Clusters clusters;
    std::vector<std::int32_t> truth_scratch;
    std::vector<Vec2> positions;
    std::vector<ColorMatch> colors;
};

void ConePerception::Impl::on_camera(const CameraFrame& frame) {
    ++stats.camera_frames;
    switch (fusion.add_frame(frame)) {
        case CameraBuffer::Insert::Added:
            break;
        case CameraBuffer::Insert::Replaced:
            ++stats.camera_frames_replaced;
            break;
        case CameraBuffer::Insert::Dropped:
            ++stats.camera_frames_dropped;
            break;
    }
}

ConeObservations ConePerception::Impl::on_lidar(const LidarScan& scan, double yaw_rate) {
    const Stopwatch watch;
    ConeObservations result;
    result.stamp = scan.stamp;
    ++stats.scans;
    stats.points += scan.points.size();
    stats.invalid_points += static_cast<std::size_t>(std::count_if(
        scan.points.begin(), scan.points.end(), [](const LidarPoint& p) { return !finite(p); }));

    const GroundFit fit = ground.fit(scan.points);
    if (!fit.ok) ++stats.ground_fallbacks;
    if (fit.ok != ground_ok) {
        ground_ok = fit.ok;
        if (fit.ok) {
            log(LogLevel::Info, "perception: ground plane fit recovered ({} inliers)", fit.inliers);
        } else {
            log(LogLevel::Warn,
                "perception: ground plane fit failed ({} seeds); keeping the "
                "previous plane",
                fit.seeds);
        }
    }
    extract_obstacles(scan.points, fit.plane, config.ground.threshold, config.clustering.max_range,
                      obstacles);
    stats.obstacle_points += obstacles.size();
    clusterer.cluster(obstacles, clusters);
    stats.clusters += clusters.size();

    const double sensor_height = fit.plane.sensor_height();
    const Vec2 mount = rig.lidar.mount.head<2>();
    for (std::size_t k = 0; k < clusters.size(); ++k) {
        const std::span<const std::uint32_t> members = clusters[k];
        const ClusterSummary summary = summarize_cluster(obstacles, members);
        const ConeCheck check = validator.check(summary, sensor_height);
        ++stats.cone_checks[static_cast<std::size_t>(check)];
        if (check != ConeCheck::Accepted) continue;

        const Vec2 center = fit_cone(obstacles, members, summary, config.center_fit).fit.center;
        ConeObservation cone;
        cone.position = center + mount;
        cone.covariance = observation_covariance(config.covariance, center, members.size());
        cone.truth_id = majority_truth_id(obstacles, members, truth_scratch);
        if (!cone.position.allFinite() || !cone.covariance.allFinite()) {
            ++stats.non_finite_estimates;
            continue;
        }
        result.cones.push_back(cone);
    }

    // Nearest first; the order is total, so equal inputs give equal outputs.
    std::sort(result.cones.begin(), result.cones.end(),
              [](const ConeObservation& a, const ConeObservation& b) {
                  const double da = a.position.squaredNorm();
                  const double db = b.position.squaredNorm();
                  if (da != db) return da < db;
                  if (a.position.y() != b.position.y()) return a.position.y() < b.position.y();
                  return a.position.x() < b.position.x();
              });

    positions.clear();
    for (const ConeObservation& cone : result.cones) positions.push_back(cone.position);
    if (!fusion.colorize(positions, scan.stamp, yaw_rate, colors)) ++stats.scans_without_camera;
    for (std::size_t i = 0; i < result.cones.size(); ++i) {
        if (colors[i].color == ConeColor::Unknown) continue;
        result.cones[i].color = colors[i].color;
        result.cones[i].color_confidence = colors[i].confidence;
        ++stats.colored_by_camera;
    }

    stats.cones_reported += result.cones.size();
    stats.lidar_time.add(watch.elapsed_seconds());
    return result;
}

ConePerception::ConePerception(const PerceptionConfig& config, const SensorRig& rig)
    : impl_(std::make_unique<Impl>(config, rig)) {}

ConePerception::~ConePerception() = default;
ConePerception::ConePerception(ConePerception&&) noexcept = default;
ConePerception& ConePerception::operator=(ConePerception&&) noexcept = default;

void ConePerception::on_camera(const CameraFrame& frame) {
    impl_->on_camera(frame);
}

ConeObservations ConePerception::on_lidar(const LidarScan& scan, double yaw_rate) {
    return impl_->on_lidar(scan, yaw_rate);
}

const PerceptionStats& ConePerception::stats() const {
    return impl_->stats;
}

}  // namespace apex::perception
