#include "apex/perception/detector.hpp"

#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "scene.hpp"

namespace apex::perception {
namespace {

ConeDetector default_detector(const LidarGeometry& geometry,
                              const CenterFitConfig& center_fit = {}) {
    return {GroundConfig{}, ClusterConfig{}, ConeCheckConfig{}, center_fit, geometry};
}

TEST(ConeDetector, ReportsEveryClusterAndSizesTheCones) {
    const LidarGeometry geometry;
    const std::vector<Cone> cones = {{0, {6.0, 1.5}, ConeColor::Blue},
                                     {1, {6.0, -1.5}, ConeColor::Yellow},
                                     {2, {9.0, 2.0}, ConeColor::BigOrange},
                                     {3, {9.0, -2.0}, ConeColor::BigOrange},
                                     {4, {16.0, 0.5}, ConeColor::Blue}};
    Rng rng(4);
    const LidarScan scan =
        testing::cast_lidar(geometry, Pose2{}, cones, {.range_sigma = 0.02}, rng);
    std::map<std::int32_t, std::size_t> returns;
    for (const LidarPoint& p : scan.points) returns[p.truth_id] += p.truth_id >= 0 ? 1 : 0;

    ConeDetector detector = default_detector(geometry);
    std::vector<DetectedCone> detected;
    const DetectionReport report = detector.detect(scan.points, detected);
    EXPECT_TRUE(report.ground.ok);
    EXPECT_EQ(report.invalid_points, 0U);
    EXPECT_EQ(std::accumulate(report.cone_checks.begin(), report.cone_checks.end(), 0U),
              report.clusters);
    EXPECT_EQ(report.cone_checks[static_cast<std::size_t>(ConeCheck::Accepted)],
              detected.size() + report.non_finite);

    ASSERT_EQ(detected.size(), cones.size());
    for (const DetectedCone& cone : detected) {
        ASSERT_GE(cone.truth_id, 0);
        const Cone& truth = cones[static_cast<std::size_t>(cone.truth_id)];
        EXPECT_LT((cone.center - truth.position).norm(), 0.04) << "cone " << truth.id;
        EXPECT_EQ(cone.big, truth.color == ConeColor::BigOrange) << "cone " << truth.id;
        // The returns of the cone minus those under the ground threshold (its widest 4 cm).
        EXPECT_LE(cone.points, returns[truth.id]);
        EXPECT_GT(cone.points, returns[truth.id] * 2 / 3);
    }
}

TEST(ConeDetector, FindsNothingInAnEmptyScan) {
    ConeDetector detector = default_detector(LidarGeometry{});
    std::vector<DetectedCone> detected = {DetectedCone{}};
    const DetectionReport report = detector.detect({}, detected);
    EXPECT_TRUE(detected.empty());
    EXPECT_FALSE(report.ground.ok);
    EXPECT_EQ(report.clusters, 0U);
}

TEST(ConeDetector, RejectsBadConfig) {
    CenterFitConfig config;
    config.range_sigma = 0.0;
    EXPECT_THROW(default_detector(LidarGeometry{}, config), std::invalid_argument);
    config = CenterFitConfig{};
    config.big_cone_log_odds = std::numeric_limits<double>::infinity();
    EXPECT_THROW(default_detector(LidarGeometry{}, config), std::invalid_argument);
    GroundConfig ground;
    ground.threshold = -1.0;
    EXPECT_THROW(ConeDetector(ground, ClusterConfig{}, ConeCheckConfig{}, CenterFitConfig{}, {}),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apex::perception
