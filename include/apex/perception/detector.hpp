#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/types.hpp"
#include "apex/perception/clustering.hpp"
#include "apex/perception/cone_model.hpp"
#include "apex/perception/ground.hpp"

// The LiDAR half of the pipeline: from the returns of one scan to the cones in it, before
// covariances and colors. See docs/perception.md, "Pipeline".
namespace apex::perception {

struct DetectedCone {
    Vec2 center = Vec2::Zero();  // LiDAR frame (m)
    std::size_t points = 0;      // returns in the cone's cluster
    bool big = false;            // fitted as a big orange cone
    std::int32_t truth_id = -1;  // the cluster's majority truth id (evaluation only)
};

// What the stages made of one scan.
struct DetectionReport {
    GroundFit ground;
    std::size_t invalid_points = 0;   // non-finite returns, ignored
    std::size_t obstacle_points = 0;  // above the ground, within the clustering range
    std::size_t clusters = 0;
    std::array<std::size_t, kConeCheckCount> cone_checks{};  // clusters by check outcome
    std::size_t non_finite = 0;  // accepted clusters dropped for a non-finite center
};

// Ground removal (GroundEstimator), clustering (GridClusterer), the cone checks (ConeValidator)
// and the center fit (fit_cone), in that order. The only state one scan leaves for the next is
// the ground estimator's fallback plane; the rest is scratch space, kept so a scan allocates
// nothing once the buffers have grown.
class ConeDetector {
public:
    // Throws std::invalid_argument for an invalid config; each stage checks its own.
    ConeDetector(const GroundConfig& ground, const ClusterConfig& clustering,
                 const ConeCheckConfig& cone_check, const CenterFitConfig& center_fit,
                 const LidarGeometry& geometry);

    // Fills `cones` with the cones among `points`, in the clusterer's order.
    DetectionReport detect(std::span<const LidarPoint> points, std::vector<DetectedCone>& cones);

private:
    CenterFitConfig center_fit_;
    GroundEstimator ground_;
    GridClusterer clusterer_;
    ConeValidator validator_;
    std::vector<ObstaclePoint> obstacles_;
    Clusters clusters_;
    std::vector<std::int32_t> truth_scratch_;
};

}  // namespace apex::perception
