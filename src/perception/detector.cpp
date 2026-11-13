#include "apex/perception/detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace apex::perception {

ConeDetector::ConeDetector(const GroundConfig& ground, const ClusterConfig& clustering,
                           const ConeCheckConfig& cone_check, const CenterFitConfig& center_fit,
                           const LidarGeometry& geometry)
    : center_fit_(center_fit),
      ground_(ground, geometry),
      clusterer_(clustering, geometry),
      validator_(cone_check, geometry, ground.threshold) {
    if (!(center_fit.range_sigma > 0.0) || center_fit.iterations < 0 ||
        !(center_fit.big_cone_height > 0.0) || !std::isfinite(center_fit.big_cone_log_odds)) {
        throw std::invalid_argument(
            "CenterFitConfig: range_sigma and big_cone_height must be positive, iterations >= 0, "
            "big_cone_log_odds finite");
    }
}

DetectionReport ConeDetector::detect(std::span<const LidarPoint> points,
                                     std::vector<DetectedCone>& cones) {
    cones.clear();
    DetectionReport report;
    report.invalid_points = static_cast<std::size_t>(
        std::count_if(points.begin(), points.end(), [](const LidarPoint& p) {
            return !(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
        }));

    report.ground = ground_.fit(points);
    extract_obstacles(points, report.ground.plane, ground_.config().threshold,
                      clusterer_.config().max_range, obstacles_);
    report.obstacle_points = obstacles_.size();
    clusterer_.cluster(obstacles_, clusters_);
    report.clusters = clusters_.size();

    const double sensor_height = report.ground.plane.sensor_height();
    for (std::size_t k = 0; k < clusters_.size(); ++k) {
        const std::span<const std::uint32_t> members = clusters_[k];
        const ClusterSummary summary = summarize_cluster(obstacles_, members);
        const ConeCheck check = validator_.check(summary, sensor_height);
        ++report.cone_checks[static_cast<std::size_t>(check)];
        if (check != ConeCheck::Accepted) continue;

        const ShapedConeFit fit = fit_cone(obstacles_, members, summary, center_fit_);
        if (!fit.fit.center.allFinite()) {
            ++report.non_finite;
            continue;
        }
        cones.push_back({fit.fit.center, members.size(), fit.big,
                         majority_truth_id(obstacles_, members, truth_scratch_)});
    }
    return report;
}

}  // namespace apex::perception
