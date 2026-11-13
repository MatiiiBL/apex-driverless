#include "apex/perception/perception.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "apex/core/log.hpp"
#include "apex/perception/detector.hpp"

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

}  // namespace

struct ConePerception::Impl {
    Impl(const PerceptionConfig& c, const SensorRig& rig)
        : covariance(c.covariance),
          lidar_mount(rig.lidar.mount.head<2>()),
          detector(c.ground, c.clustering, c.cone_check, c.center_fit, rig.lidar),
          fusion(c.camera, rig.camera) {
        validate(c.covariance.radial, "radial");
        validate(c.covariance.tangential, "tangential");
    }

    ConeObservations on_lidar(const LidarScan& scan, double yaw_rate);
    void on_camera(const CameraFrame& frame);
    void count(const DetectionReport& report, std::size_t points);

    CovarianceModel covariance;
    Vec2 lidar_mount;
    ConeDetector detector;
    CameraFusion fusion;
    PerceptionStats stats;
    bool ground_ok = true;

    // Per-scan scratch, kept to avoid reallocating every scan.
    std::vector<DetectedCone> detected;
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

void ConePerception::Impl::count(const DetectionReport& report, std::size_t points) {
    ++stats.scans;
    stats.points += points;
    stats.invalid_points += report.invalid_points;
    stats.obstacle_points += report.obstacle_points;
    stats.clusters += report.clusters;
    for (std::size_t i = 0; i < kConeCheckCount; ++i) stats.cone_checks[i] += report.cone_checks[i];
    stats.non_finite_estimates += report.non_finite;
    if (!report.ground.ok) ++stats.ground_fallbacks;
    if (report.ground.ok == ground_ok) return;
    ground_ok = report.ground.ok;
    if (ground_ok) {
        log(LogLevel::Info, "perception: ground plane fit recovered ({} inliers)",
            report.ground.inliers);
    } else {
        log(LogLevel::Warn,
            "perception: ground plane fit failed ({} seeds); keeping the previous "
            "plane",
            report.ground.seeds);
    }
}

ConeObservations ConePerception::Impl::on_lidar(const LidarScan& scan, double yaw_rate) {
    const Stopwatch watch;
    ConeObservations result;
    result.stamp = scan.stamp;
    count(detector.detect(scan.points, detected), scan.points.size());

    for (const DetectedCone& d : detected) {
        ConeObservation cone;
        cone.position = d.center + lidar_mount;
        cone.covariance = observation_covariance(covariance, d.center, d.points);
        cone.truth_id = d.truth_id;
        if (!cone.covariance.allFinite()) {
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
