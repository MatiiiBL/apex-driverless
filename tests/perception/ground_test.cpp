#include "apex/perception/ground.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"

#include "scene.hpp"

namespace apex::perception {
namespace {

using testing::cast_lidar;
using testing::LidarSimOptions;

// A field of cones around the LiDAR, dense enough that many seed cells hold cone returns.
std::vector<Cone> cone_field() {
    std::vector<Cone> cones;
    for (int i = 0; i < 12; ++i) {
        for (int side = -1; side <= 1; side += 2) {
            Cone cone;
            cone.id = static_cast<std::int32_t>(cones.size());
            cone.position = {3.0 + 3.0 * i, side * (1.75 + 0.2 * i)};
            cone.color = i % 5 == 0 ? ConeColor::BigOrange : ConeColor::Blue;
            cones.push_back(cone);
        }
    }
    return cones;
}

TEST(GroundEstimator, FindsTheSensorHeightWithoutKnowingTheMount) {
    const LidarGeometry geometry;  // nominal mount height 0.20 m
    const std::vector<Cone> cones = cone_field();
    for (const double height : {0.12, 0.20, 0.35}) {
        Rng rng(7);
        LidarSimOptions options;
        options.sensor_height = height;
        options.range_sigma = 0.02;
        const LidarScan scan = cast_lidar(geometry, Pose2{}, cones, options, rng);
        GroundEstimator estimator(GroundConfig{}, geometry);
        const GroundFit fit = estimator.fit(scan.points);
        ASSERT_TRUE(fit.ok) << "height " << height;
        EXPECT_NEAR(fit.plane.sensor_height(), height, 0.005) << "height " << height;
        EXPECT_LT(fit.plane.tilt(), deg_to_rad(0.05));
        EXPECT_GT(fit.inliers, 100U);
    }
}

TEST(GroundEstimator, FollowsAPitchedSensor) {
    const LidarGeometry geometry;
    const std::vector<Cone> cones = cone_field();
    Rng rng(3);
    LidarSimOptions options;
    options.pitch = deg_to_rad(2.0);  // nose down: the ground rises ahead in the LiDAR frame
    options.range_sigma = 0.02;
    const LidarScan scan = cast_lidar(geometry, Pose2{}, cones, options, rng);
    GroundEstimator estimator(GroundConfig{}, geometry);
    const GroundFit fit = estimator.fit(scan.points);
    ASSERT_TRUE(fit.ok);
    EXPECT_NEAR(fit.plane.tilt(), deg_to_rad(2.0), deg_to_rad(0.05));
    EXPECT_GT(fit.plane.slope_x, 0.0);
    EXPECT_NEAR(fit.plane.sensor_height(), options.sensor_height, 0.005);

    // With the tilt fitted, the ground returns 30 to 50 m out (a channel or two graze the ground
    // there) are still within the threshold.
    std::size_t ground_far = 0;
    std::size_t misclassified = 0;
    for (const LidarPoint& p : scan.points) {
        if (p.truth_id >= 0 || p.x < 30.0) continue;
        ++ground_far;
        if (fit.plane.height_of(p.x, p.y, p.z) > GroundConfig{}.threshold) ++misclassified;
    }
    EXPECT_GT(ground_far, 100U);
    EXPECT_EQ(misclassified, 0U);
}

TEST(GroundEstimator, IgnoresReturnsAboveTheGround) {
    // Every ray hits a cone or a wall of clutter in many cells; the plane must not move.
    const LidarGeometry geometry;
    std::vector<LidarPoint> points;
    for (int ix = 1; ix <= 40; ++ix) {
        for (int iy = -40; iy <= 40; ++iy) {
            const auto x = static_cast<float>(ix);
            const auto y = static_cast<float>(iy);
            points.push_back({x, y, -0.25F, -1});
            // A third of the cells also hold an obstacle, some lower than the ground nearby.
            if ((ix + iy) % 3 == 0) points.push_back({x + 0.3F, y, 0.4F, 5});
        }
    }
    // Some cells hold only obstacle returns (no ground under them), as behind a wall.
    for (int iy = -40; iy <= 40; ++iy) points.push_back({45.0F, static_cast<float>(iy), 0.1F, 6});
    GroundEstimator estimator(GroundConfig{}, geometry);
    const GroundFit fit = estimator.fit(points);
    ASSERT_TRUE(fit.ok);
    EXPECT_NEAR(fit.plane.sensor_height(), 0.25, 1e-6);
    EXPECT_NEAR(fit.plane.tilt(), 0.0, 1e-6);
}

TEST(GroundEstimator, FallsBackToTheLastGoodPlane) {
    const LidarGeometry geometry;  // mount at 0.20 m
    GroundEstimator estimator(GroundConfig{}, geometry);

    // Before any good fit: the nominal plane from the mount height.
    const GroundFit empty = estimator.fit({});
    EXPECT_FALSE(empty.ok);
    EXPECT_EQ(empty.seeds, 0U);
    EXPECT_NEAR(empty.plane.sensor_height(), 0.20, 1e-12);

    Rng rng(1);
    LidarSimOptions options;
    options.sensor_height = 0.31;
    const LidarScan scan = cast_lidar(geometry, Pose2{}, cone_field(), options, rng);
    ASSERT_TRUE(estimator.fit(scan.points).ok);

    // Too few seeds: a handful of returns.
    const std::vector<LidarPoint> few(scan.points.begin(), scan.points.begin() + 5);
    const GroundFit fallback = estimator.fit(few);
    EXPECT_FALSE(fallback.ok);
    EXPECT_NEAR(fallback.plane.sensor_height(), 0.31, 0.005);

    // A steep "ground" (a wall facing the car) is rejected rather than trusted.
    std::vector<LidarPoint> wall;
    for (int i = 0; i < 40; ++i) {
        for (int j = 0; j < 40; ++j) {
            wall.push_back({2.0F * static_cast<float>(i), 2.0F * static_cast<float>(j - 20),
                            0.5F * static_cast<float>(i), -1});
        }
    }
    EXPECT_FALSE(estimator.fit(wall).ok);
}

TEST(GroundEstimator, SkipsNonFiniteReturns) {
    const LidarGeometry geometry;
    Rng rng(2);
    LidarScan scan = cast_lidar(geometry, Pose2{}, cone_field(), LidarSimOptions{}, rng);
    GroundEstimator clean(GroundConfig{}, geometry);
    const GroundFit reference = clean.fit(scan.points);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    scan.points.insert(scan.points.begin(), {nan, 1.0F, 0.0F, -1});
    scan.points.push_back({1.0F, inf, 0.0F, -1});
    scan.points.push_back({1.0F, 1.0F, -inf, -1});
    GroundEstimator noisy(GroundConfig{}, geometry);
    const GroundFit fit = noisy.fit(scan.points);
    ASSERT_TRUE(fit.ok);
    EXPECT_EQ(fit.plane.offset, reference.plane.offset);
    EXPECT_EQ(fit.plane.slope_x, reference.plane.slope_x);
}

TEST(GroundEstimator, RejectsBadConfig) {
    GroundConfig config;
    config.threshold = 0.0;
    EXPECT_THROW(GroundEstimator(config, LidarGeometry{}), std::invalid_argument);
    config = GroundConfig{};
    config.iterations = 0;
    EXPECT_THROW(GroundEstimator(config, LidarGeometry{}), std::invalid_argument);
    config = GroundConfig{};
    config.seed_cell_size = -1.0;
    EXPECT_THROW(GroundEstimator(config, LidarGeometry{}), std::invalid_argument);
}

TEST(ExtractObstacles, KeepsOnlyFiniteReturnsAboveTheThresholdAndInRange) {
    const GroundPlane plane{0.0, 0.0, -0.2};
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const std::vector<LidarPoint> points = {
        {5.0F, 0.0F, -0.19F, -1},  // 1 cm above the ground: ground
        {5.0F, 0.0F, -0.15F, 3},   // 5 cm: obstacle
        {5.0F, 0.0F, 0.10F, 3},    // 30 cm: obstacle
        {nan, 0.0F, 0.10F, 3},     // not finite
        {45.0F, 0.0F, 0.0F, 4},    // beyond max_range
        {5.0F, 0.0F, -0.5F, -1},   // below the ground (noise): not an obstacle
    };
    std::vector<ObstaclePoint> out;
    extract_obstacles(points, plane, 0.04, 40.0, out);
    ASSERT_EQ(out.size(), 2U);
    EXPECT_NEAR(out[0].height, 0.05, 1e-6);
    EXPECT_NEAR(out[1].height, 0.30, 1e-6);
    EXPECT_EQ(out[1].truth_id, 3);
}

}  // namespace
}  // namespace apex::perception
