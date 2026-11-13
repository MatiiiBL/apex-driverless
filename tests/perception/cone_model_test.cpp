#include "apex/perception/cone_model.hpp"

#include <cmath>
#include <numeric>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "apex/core/angles.hpp"

#include "scene.hpp"

namespace apex::perception {
namespace {

using testing::cast_lidar;
using testing::LidarSimOptions;

constexpr double kThreshold = 0.04;  // GroundConfig's default

struct ConeReturns {
    std::vector<ObstaclePoint> points;
    std::vector<std::uint32_t> members;
};

// The above-ground returns of one cone at `center` seen by a LiDAR at the origin, 0.2 m up.
ConeReturns scan_one_cone(const Vec2& center, ConeColor color, double range_sigma,
                          std::uint64_t seed) {
    const LidarGeometry geometry;
    const std::vector<Cone> cones = {{0, center, color}};
    Rng rng(seed);
    LidarSimOptions options;
    options.range_sigma = range_sigma;
    const LidarScan scan = cast_lidar(geometry, Pose2{}, cones, options, rng);
    ConeReturns out;
    extract_obstacles(scan.points, GroundPlane{0.0, 0.0, -options.sensor_height}, kThreshold, 50.0,
                      out.points);
    out.members.resize(out.points.size());
    std::iota(out.members.begin(), out.members.end(), 0U);
    return out;
}

ObstaclePoint at(double x, double y, double height) {
    return {static_cast<float>(x), static_cast<float>(y), static_cast<float>(height), -1};
}

ClusterSummary summarize(const std::vector<ObstaclePoint>& points) {
    std::vector<std::uint32_t> members(points.size());
    std::iota(members.begin(), members.end(), 0U);
    return summarize_cluster(points, members);
}

TEST(ExpectedConeReturns, MatchesTheRayCasterOnAverage) {
    const LidarGeometry geometry;
    for (const ConeColor color : {ConeColor::Yellow, ConeColor::BigOrange}) {
        for (const double range : {2.0, 4.0, 7.0, 10.0, 15.0, 20.0, 25.0, 30.0}) {
            // Average over bearings: a single bearing's count depends on the azimuth phase.
            double sum = 0.0;
            constexpr int kBearings = 9;
            for (int b = 0; b < kBearings; ++b) {
                const double bearing = -0.3 + 0.6 * b / (kBearings - 1) + 0.0007 * b;
                const Vec2 center = range * Vec2(std::cos(bearing), std::sin(bearing));
                sum += static_cast<double>(scan_one_cone(center, color, 0.0, 1).points.size());
            }
            const double measured = sum / kBearings;
            const double expected =
                expected_cone_returns(geometry, cone_shape(color), range, 0.2, kThreshold);
            EXPECT_NEAR(measured, expected, 0.15 * expected + 1.0)
                << to_string(color) << " at " << range << " m";
        }
    }
}

TEST(ExpectedConeReturns, IsZeroWhereNoChannelReachesTheCone) {
    LidarGeometry geometry;
    geometry.elevations = {deg_to_rad(5.0), deg_to_rad(6.0)};  // all above a cone 3 m+ away
    EXPECT_EQ(expected_cone_returns(geometry, cone_shape(ConeColor::Blue), 3.0, 0.2, kThreshold),
              0.0);
    EXPECT_EQ(
        expected_cone_returns(LidarGeometry{}, cone_shape(ConeColor::Blue), 0.05, 0.2, kThreshold),
        0.0);
}

TEST(FitConeCenter, RecoversFullyVisibleCones) {
    const CenterFitConfig config;
    for (const ConeColor color : {ConeColor::Blue, ConeColor::BigOrange}) {
        for (const double range : {3.0, 6.0, 10.0, 15.0, 20.0, 30.0}) {
            double fit_error = 0.0;
            double centroid_error = 0.0;
            constexpr int kTrials = 6;
            for (int trial = 0; trial < kTrials; ++trial) {
                const double bearing = -0.4 + 0.15 * trial;
                const Vec2 center = range * Vec2(std::cos(bearing), std::sin(bearing));
                const ConeReturns r =
                    scan_one_cone(center, color, 0.02, static_cast<std::uint64_t>(10 + trial));
                ASSERT_FALSE(r.points.empty());
                const Vec2 fit =
                    fit_cone_center(r.points, r.members, cone_shape(color), config).center;
                fit_error += (fit - center).norm() / kTrials;
                centroid_error +=
                    (summarize_cluster(r.points, r.members).centroid - center).norm() / kTrials;
            }
            const double tolerance = range <= 20.0 ? 0.025 : 0.05;
            EXPECT_LT(fit_error, tolerance) << to_string(color) << " at " << range << " m";
            // The centroid sits on the near face; the fit corrects it.
            if (range <= 20.0) {
                EXPECT_LT(fit_error, 0.5 * centroid_error) << range << " m";
            }
        }
    }
}

TEST(FitConeCenter, RecoversAHalfVisibleCone) {
    // Only the right half of the face (y below the center) is visible, as behind an occluder.
    const Vec2 center(8.0, 1.0);
    const ConeReturns full = scan_one_cone(center, ConeColor::Yellow, 0.02, 3);
    ConeReturns half;
    for (const ObstaclePoint& p : full.points) {
        if (p.y < center.y()) half.points.push_back(p);
    }
    half.members.resize(half.points.size());
    std::iota(half.members.begin(), half.members.end(), 0U);
    ASSERT_GT(half.points.size(), 10U);
    const ConeShape shape = cone_shape(ConeColor::Yellow);
    const Vec2 fit = fit_cone_center(half.points, half.members, shape, CenterFitConfig{}).center;
    EXPECT_LT((fit - center).norm(), 0.03);
    // A centroid offset along the line of sight would be off by about half the radius sideways.
    const Vec2 centroid = summarize_cluster(half.points, half.members).centroid;
    EXPECT_GT(std::abs((centroid - center).dot(Vec2(-center.y(), center.x()).normalized())), 0.03);
}

TEST(FitConeCenter, HandlesASingleReturn) {
    const std::vector<ObstaclePoint> points = {at(20.0, 0.0, 0.1)};
    const std::vector<std::uint32_t> members = {0};
    const ConeShape shape = cone_shape(ConeColor::Blue);
    const ConeFit fit = fit_cone_center(points, members, shape, CenterFitConfig{});
    // The center is behind the return by the cone's radius at that height, on the ray.
    EXPECT_NEAR(fit.center.x(), 20.0 + shape.base_radius * (1.0 - 0.1 / shape.height), 0.005);
    EXPECT_NEAR(fit.center.y(), 0.0, 1e-9);
    EXPECT_LT(fit.residual_sq, 1e-4);
}

TEST(FitCone, TellsBigConesFromSmallOnesWhereTheirTopIsUnseen) {
    const CenterFitConfig config;
    for (const ConeColor color : {ConeColor::Blue, ConeColor::BigOrange}) {
        const bool big = color == ConeColor::BigOrange;
        // At 1.5 m the highest channel passes below a big cone's top; at 10 m and beyond the
        // channels straddle it, so it is often missed too.
        for (const double range : {1.5, 3.0, 6.0, 10.0}) {
            for (int trial = 0; trial < 4; ++trial) {
                const double bearing = -0.3 + 0.2 * trial;
                const Vec2 center = range * Vec2(std::cos(bearing), std::sin(bearing));
                const ConeReturns r =
                    scan_one_cone(center, color, 0.02, static_cast<std::uint64_t>(30 + trial));
                const ClusterSummary summary = summarize_cluster(r.points, r.members);
                const ShapedConeFit fit = fit_cone(r.points, r.members, summary, config);
                EXPECT_EQ(fit.big, big) << to_string(color) << " at " << range << " m, trial "
                                        << trial << ", max height " << summary.max_height;
                EXPECT_LT((fit.fit.center - center).norm(), 0.03)
                    << to_string(color) << " at " << range << " m";
            }
        }
    }
}

TEST(FitCone, AReturnAboveASmallConeSettlesTheSize) {
    const CenterFitConfig config;
    const std::vector<ObstaclePoint> points = {at(5.0, 0.0, 0.10), at(5.0, 0.01, 0.42)};
    const std::vector<std::uint32_t> members = {0, 1};
    EXPECT_TRUE(fit_cone(points, members, summarize(points), config).big);
    const std::vector<ObstaclePoint> low = {at(5.0, 0.0, 0.10)};
    const std::vector<std::uint32_t> one = {0};
    EXPECT_FALSE(fit_cone(low, one, summarize(low), config).big);
}

TEST(ConeValidator, AcceptsConesAtEveryRange) {
    const ConeValidator validator(ConeCheckConfig{}, LidarGeometry{}, kThreshold);
    // A small cone gives one or two returns beyond 25 m, fewer than min_points; a big cone
    // still gives three at 30 m.
    for (const ConeColor color : {ConeColor::Blue, ConeColor::BigOrange}) {
        for (const double range : {1.5, 3.0, 6.0, 10.0, 15.0, 20.0, 25.0, 30.0}) {
            if (color == ConeColor::Blue && range > 25.0) continue;
            const Vec2 center = range * Vec2(std::cos(0.2), std::sin(0.2));
            const ConeReturns r = scan_one_cone(center, color, 0.02, 5);
            const ClusterSummary summary = summarize_cluster(r.points, r.members);
            EXPECT_EQ(validator.check(summary, 0.2), ConeCheck::Accepted)
                << to_string(color) << " at " << range
                << " m: " << to_string(validator.check(summary, 0.2));
        }
    }
}

TEST(ConeValidator, RejectsWhatCannotBeACone) {
    const ConeValidator validator(ConeCheckConfig{}, LidarGeometry{}, kThreshold);
    std::vector<ObstaclePoint> pole;  // a post taller than any cone
    for (int i = 0; i < 30; ++i) pole.push_back(at(8.0, 0.0, 0.05 + 0.03 * i));
    EXPECT_EQ(validator.check(summarize(pole), 0.2), ConeCheck::TooTall);

    std::vector<ObstaclePoint> wall;  // 1 m across the line of sight
    for (int i = 0; i < 40; ++i) wall.push_back(at(8.0, -0.5 + 0.025 * i, 0.1 + 0.005 * i));
    EXPECT_EQ(validator.check(summarize(wall), 0.2), ConeCheck::TooWide);

    std::vector<ObstaclePoint> deep;  // 0.6 m along it
    for (int i = 0; i < 40; ++i) deep.push_back(at(8.0 + 0.015 * i, 0.0, 0.1 + 0.005 * i));
    EXPECT_EQ(validator.check(summarize(deep), 0.2), ConeCheck::TooDeep);

    std::vector<ObstaclePoint> floating;  // compact, but half a meter off the ground
    for (int i = 0; i < 20; ++i) floating.push_back(at(8.0, 0.005 * i, 0.50 + 0.002 * i));
    EXPECT_EQ(validator.check(summarize(floating), 0.2), ConeCheck::Floating);

    // Three returns where a cone would give about forty.
    const std::vector<ObstaclePoint> sparse = {at(8.0, 0.0, 0.1), at(8.0, 0.01, 0.15),
                                               at(8.0, 0.02, 0.2)};
    EXPECT_EQ(validator.check(summarize(sparse), 0.2), ConeCheck::TooFewPoints);

    // A cone-sized blob with far more returns than even a big cone gives at 20 m.
    std::vector<ObstaclePoint> dense;
    for (int i = 0; i < 200; ++i) dense.push_back(at(20.0, 0.001 * i, 0.05 + 0.001 * i));
    EXPECT_EQ(validator.check(summarize(dense), 0.2), ConeCheck::TooManyPoints);
}

TEST(ConeValidator, RejectsBadConfig) {
    ConeCheckConfig config;
    config.max_excess_factor = 0.5;
    EXPECT_THROW(ConeValidator(config, LidarGeometry{}, kThreshold), std::invalid_argument);
    config = ConeCheckConfig{};
    config.min_points = 0;
    EXPECT_THROW(ConeValidator(config, LidarGeometry{}, kThreshold), std::invalid_argument);
    LidarGeometry no_channels;
    no_channels.elevations.clear();
    EXPECT_THROW(ConeValidator(ConeCheckConfig{}, no_channels, kThreshold), std::invalid_argument);
}

TEST(ObservationCovariance, GrowsWithRangeShrinksWithPointsAlignsWithTheLineOfSight) {
    const CovarianceModel model;
    const Vec2 direction = Vec2(1.0, 1.0).normalized();
    const Mat2 near = observation_covariance(model, 5.0 * direction, 20);
    const Mat2 far = observation_covariance(model, 25.0 * direction, 20);
    const Mat2 many = observation_covariance(model, 25.0 * direction, 200);
    EXPECT_GT(far.determinant(), near.determinant());
    EXPECT_LT(many.determinant(), far.determinant());
    for (const Mat2& c : {near, far, many}) {
        EXPECT_NEAR(c(0, 1), c(1, 0), 1e-15);
        EXPECT_GT(c.determinant(), 0.0);
        EXPECT_GT(c.trace(), 0.0);
    }
    // Radial and tangential are the eigenvectors.
    const Vec2 radial = near * direction;
    EXPECT_NEAR(radial.normalized().dot(direction), 1.0, 1e-12);
    EXPECT_NEAR(direction.dot(near * direction), model.radial.at(5.0, 20), 1e-15);
}

TEST(MajorityTruthId, PicksTheMostFrequentWithTiesToTheSmaller) {
    std::vector<ObstaclePoint> points = {at(0, 0, 0), at(0, 0, 0), at(0, 0, 0), at(0, 0, 0)};
    points[0].truth_id = 3;
    points[1].truth_id = 3;
    std::vector<std::int32_t> scratch;
    const std::vector<std::uint32_t> all = {0, 1, 2, 3};
    EXPECT_EQ(majority_truth_id(points, all, scratch), -1);  // 3, 3, -1, -1: tie
    points[2].truth_id = 3;
    EXPECT_EQ(majority_truth_id(points, all, scratch), 3);
    EXPECT_EQ(majority_truth_id(points, {}, scratch), -1);
}

}  // namespace
}  // namespace apex::perception
