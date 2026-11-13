#include "scoring.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <map>
#include <utility>

#include <Eigen/Dense>

#include "apex/core/geometry.hpp"

namespace apex::perception::eval {
namespace {

constexpr double kChiSquare95 = 5.991;  // 2 degrees of freedom

// A cone's position in the LiDAR frame of a car at `body`.
Vec2 in_lidar(const Cone& cone, const Pose2& body, const LidarGeometry& lidar) {
    return body.to_local(cone.position) - lidar.mount.head<2>();
}

const Cone* cone_at(const Truth& truth, int index) {
    return index < 0 ? nullptr : &truth.cones[static_cast<std::size_t>(index)];
}

}  // namespace

DetectionBin& DetectionBin::operator+=(const DetectionBin& other) {
    in_view += other.in_view;
    hit += other.hit;
    detected += other.detected;
    reported += other.reported;
    false_positives += other.false_positives;
    error_sq += other.error_sq;
    radial_sq += other.radial_sq;
    tangential_sq += other.tangential_sq;
    nees += other.nees;
    nees_above_95 += other.nees_above_95;
    return *this;
}

RangeBins::RangeBins(std::vector<double> edges)
    : edges_(std::move(edges)), bins_(edges_.size() + 1) {}

DetectionBin& RangeBins::at_range(double range) {
    const auto above = std::upper_bound(edges_.begin(), edges_.end(), range);
    return bins_[static_cast<std::size_t>(above - edges_.begin())];
}

std::string RangeBins::label(std::size_t i) const {
    const auto edge = [&](std::size_t k) { return std::format("{:g}", edges_[k]); };
    if (edges_.empty()) return "all";
    if (i == 0) return "0-" + edge(0);
    if (i == edges_.size()) return edge(i - 1) + "+";
    return edge(i - 1) + "-" + edge(i);
}

DetectionBin RangeBins::total() const {
    DetectionBin sum;
    for (const DetectionBin& bin : bins_) sum += bin;
    return sum;
}

std::vector<int> match_observations(const ConeObservations& observations, const Truth& truth,
                                    double gate) {
    std::vector<int> matches(observations.cones.size(), -1);
    std::vector<char> claimed(truth.cones.size(), 0);
    for (std::size_t i = 0; i < observations.cones.size(); ++i) {
        const ConeObservation& o = observations.cones[i];
        if (o.truth_id < 0 || static_cast<std::size_t>(o.truth_id) >= truth.cones.size()) continue;
        const auto id = static_cast<std::size_t>(o.truth_id);
        const Vec2 cone = truth.body.to_local(truth.cones[id].position);
        if (claimed[id] != 0 || (o.position - cone).norm() > gate) continue;
        claimed[id] = 1;
        matches[i] = o.truth_id;
    }
    return matches;
}

void score_detections(const ConeObservations& observations, std::span<const int> matches,
                      const LidarScan& scan, const Truth& truth, const LidarGeometry& lidar,
                      RangeBins& bins) {
    std::map<std::int32_t, std::size_t> returns;
    for (const LidarPoint& p : scan.points) {
        if (p.truth_id >= 0) ++returns[p.truth_id];
    }
    std::vector<char> detected(truth.cones.size(), 0);
    for (const int match : matches) {
        if (match >= 0) detected[static_cast<std::size_t>(match)] = 1;
    }
    for (const Cone& cone : truth.cones) {
        const Vec2 p = in_lidar(cone, truth.body, lidar);
        const double azimuth = std::atan2(p.y(), p.x());
        const double range = p.norm();
        if (azimuth < lidar.azimuth_min || azimuth > lidar.azimuth_max || range < lidar.min_range ||
            range > lidar.max_range) {
            continue;
        }
        DetectionBin& bin = bins.at_range(range);
        ++bin.in_view;
        if (returns.contains(cone.id)) ++bin.hit;
        if (detected[static_cast<std::size_t>(cone.id)] != 0) ++bin.detected;
    }

    const Vec2 mount = lidar.mount.head<2>();
    for (std::size_t i = 0; i < observations.cones.size(); ++i) {
        const ConeObservation& o = observations.cones[i];
        DetectionBin& reported_bin = bins.at_range((o.position - mount).norm());
        ++reported_bin.reported;
        const Cone* cone = cone_at(truth, matches[i]);
        if (cone == nullptr) {
            ++reported_bin.false_positives;
            continue;
        }
        const Vec2 truth_body = truth.body.to_local(cone->position);
        const Vec2 radial = (truth_body - mount).normalized();
        const Vec2 error = o.position - truth_body;
        DetectionBin& bin = bins.at_range((truth_body - mount).norm());
        bin.error_sq += error.squaredNorm();
        bin.radial_sq += std::pow(error.dot(radial), 2);
        bin.tangential_sq += std::pow(cross(radial, error), 2);
        const double nees = error.dot(o.covariance.ldlt().solve(error));
        bin.nees += nees;
        bin.nees_above_95 += nees > kChiSquare95 ? 1 : 0;
    }
}

void score_colors(const ConeObservations& observations, std::span<const int> matches,
                  const Truth& truth, const Pose2& camera_body, const CameraGeometry& camera,
                  ColorScore& score) {
    for (std::size_t i = 0; i < observations.cones.size(); ++i) {
        const Cone* cone = cone_at(truth, matches[i]);
        if (cone == nullptr) continue;
        const Vec2 p = camera_body.to_local(cone->position) - camera.mount.head<2>();
        if (std::abs(std::atan2(p.y(), p.x())) > 0.5 * camera.horizontal_fov ||
            p.norm() > camera.max_range) {
            continue;
        }
        ++score.visible;
        const ConeObservation& o = observations.cones[i];
        if (o.color == ConeColor::Unknown) continue;
        ++score.colored;
        if (o.color == cone->color) {
            ++score.correct;
            score.confidence_correct += o.color_confidence;
        } else {
            score.confidence_wrong += o.color_confidence;
        }
    }
}

void score_detector(std::span<const DetectedCone> detected, const Truth& truth,
                    const LidarGeometry& lidar, double gate, DetectorScore& score) {
    std::vector<char> claimed(truth.cones.size(), 0);
    for (const DetectedCone& d : detected) {
        if (d.truth_id < 0 || static_cast<std::size_t>(d.truth_id) >= truth.cones.size()) continue;
        const auto id = static_cast<std::size_t>(d.truth_id);
        const Cone& cone = truth.cones[id];
        const Vec2 truth_lidar = in_lidar(cone, truth.body, lidar);
        const Vec2 error = d.center - truth_lidar;
        if (claimed[id] != 0 || error.norm() > gate) continue;
        claimed[id] = 1;
        // The covariance model is evaluated at the estimate, as at run time.
        const double range = d.center.norm();
        const Vec2 radial = d.center / range;
        score.radial.push_back({range, d.points, error.dot(radial)});
        score.tangential.push_back({range, d.points, cross(radial, error)});
        if (cone.color == ConeColor::BigOrange) {
            ++score.big_cones;
            score.big_as_big += d.big ? 1 : 0;
        } else {
            ++score.small_cones;
            score.small_as_big += d.big ? 1 : 0;
        }
    }
}

}  // namespace apex::perception::eval
