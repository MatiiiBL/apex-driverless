#include "apex/runner/oracle.hpp"

#include <cmath>
#include <optional>
#include <vector>

#include <Eigen/LU>
#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/track.hpp"
#include "apex/perception/cone_model.hpp"

namespace apex::runner {
namespace {

VehicleState car_at(double x, double y, double yaw) {
    VehicleState state;
    state.pose = {x, y, yaw};
    state.vx = 8.0;
    state.yaw_rate = 0.2;
    return state;
}

// A covariance model of zero: the oracle's draws are then the true positions (to its 0.01 mm
// floor), which makes the geometry checkable.
PerceptionOracleConfig noiseless() {
    PerceptionOracleConfig config;
    config.covariance = perception::CovarianceModel{{}, {}};
    return config;
}

double expected_returns(const LidarGeometry& lidar, ConeColor color, double range,
                        const PerceptionOracleConfig& config) {
    return perception::expected_cone_returns(lidar, cone_shape(color), range, lidar.mount.z(),
                                             config.ground_threshold);
}

TEST(OracleObservations, EveryVisibleConeInRangeAndFieldOfView) {
    const LidarGeometry lidar;
    std::vector<Cone> cones;
    // Car at the origin facing +y; the LiDAR is 1.2 m ahead of the CoG, at (0, 1.2).
    cones.push_back({0, {0.0, 10.0}, ConeColor::Blue});        // ahead
    cones.push_back({1, {5.0, 1.3}, ConeColor::Yellow});       // just inside +-90 degrees
    cones.push_back({2, {5.0, 1.1}, ConeColor::Yellow});       // just behind the LiDAR's plane
    cones.push_back({3, {0.0, 1.2 + 45.0}, ConeColor::Blue});  // beyond max_range
    cones.push_back({4, {-3.0, 3.0}, ConeColor::BigOrange});
    Rng rng(1);
    const ConeObservations obs =
        oracle_observations(cones, car_at(0.0, 0.0, kPi / 2), lidar, noiseless(), 1.5, rng);
    EXPECT_DOUBLE_EQ(obs.stamp, 1.5);
    ASSERT_EQ(obs.cones.size(), 3U);
    const std::vector<std::int32_t> ids = {obs.cones[0].truth_id, obs.cones[1].truth_id,
                                           obs.cones[2].truth_id};
    EXPECT_EQ(ids, (std::vector<std::int32_t>{0, 1, 4}));
    // Body frame: x forward (+y in the world), y left (-x in the world).
    EXPECT_NEAR(obs.cones[0].position.x(), 10.0, 1e-4);
    EXPECT_NEAR(obs.cones[0].position.y(), 0.0, 1e-4);
    EXPECT_NEAR(obs.cones[2].position.x(), 3.0, 1e-4);
    EXPECT_NEAR(obs.cones[2].position.y(), 3.0, 1e-4);
    EXPECT_EQ(obs.cones[2].color, ConeColor::BigOrange);
    EXPECT_DOUBLE_EQ(obs.cones[2].color_confidence, 1.0);
    EXPECT_GT(obs.cones[0].covariance.determinant(), 0.0);
}

TEST(OracleObservations, SeesWhatPerceptionCouldSee) {
    const LidarGeometry lidar;
    const PerceptionOracleConfig config = noiseless();
    // A cone is reported exactly where an unoccluded one would give enough returns.
    for (double range = 2.0; range < 40.0; range += 0.5) {
        for (const ConeColor color : {ConeColor::Blue, ConeColor::BigOrange}) {
            const std::vector<Cone> cones = {{0, {1.2 + range, 0.0}, color}};
            Rng rng(3);
            const bool reported =
                !oracle_observations(cones, car_at(0.0, 0.0, 0.0), lidar, config, 0.0, rng)
                     .cones.empty();
            EXPECT_EQ(reported, expected_returns(lidar, color, range, config) >= 3.0)
                << to_string(color) << " at " << range << " m";
        }
    }
    // Big cones are taller, so they stay visible farther out than small ones.
    std::optional<double> gap;
    for (double range = 10.0; range < 40.0 && !gap; range += 0.5) {
        if (expected_returns(lidar, ConeColor::Blue, range, config) < 3.0 &&
            expected_returns(lidar, ConeColor::BigOrange, range, config) >= 3.0) {
            gap = range;
        }
    }
    ASSERT_TRUE(gap.has_value());
    const std::vector<Cone> pair = {{0, {1.2 + *gap, 0.5}, ConeColor::Blue},
                                    {1, {1.2 + *gap, -0.5}, ConeColor::BigOrange}};
    Rng rng(4);
    const ConeObservations obs =
        oracle_observations(pair, car_at(0.0, 0.0, 0.0), lidar, config, 0.0, rng);
    ASSERT_EQ(obs.cones.size(), 1U);
    EXPECT_EQ(obs.cones[0].truth_id, 1);
}

TEST(OracleObservations, NoiseIsPerceptionsCovariance) {
    const LidarGeometry lidar;
    const PerceptionOracleConfig config;
    const double range = 15.0;
    const std::vector<Cone> cones = {{0, {1.2 + range, 0.0}, ConeColor::Yellow}};
    const auto points = static_cast<std::size_t>(
        std::lround(expected_returns(lidar, ConeColor::Yellow, range, config)));
    const Mat2 expected =
        perception::observation_covariance(config.covariance, {range, 0.0}, points);
    Rng rng(7);
    Vec2 sum = Vec2::Zero();
    Vec2 sum_squares = Vec2::Zero();
    constexpr int kDraws = 4000;
    for (int i = 0; i < kDraws; ++i) {
        const ConeObservations obs =
            oracle_observations(cones, car_at(0.0, 0.0, 0.0), lidar, config, 0.0, rng);
        ASSERT_EQ(obs.cones.size(), 1U);
        EXPECT_TRUE(obs.cones[0].covariance.isApprox(expected, 1e-4));
        const Vec2 error = obs.cones[0].position - Vec2{1.2 + range, 0.0};
        sum += error;
        sum_squares += error.cwiseProduct(error);
    }
    // Straight ahead, x is radial and y tangential: the model's two variances.
    for (int axis = 0; axis < 2; ++axis) {
        const double sigma = std::sqrt(expected(axis, axis));
        EXPECT_NEAR(sum[axis] / kDraws, 0.0, 4.0 * sigma / std::sqrt(kDraws));
        EXPECT_NEAR(std::sqrt(sum_squares[axis] / kDraws), sigma, 0.05 * sigma);
    }
}

TEST(OracleObservations, SameSeedSameNoise) {
    const Track track = make_oval_track();
    Rng a(5);
    Rng b(5);
    const VehicleState car = car_at(track.start_pose.x, track.start_pose.y, track.start_pose.yaw);
    const ConeObservations first =
        oracle_observations(track.cones, car, LidarGeometry{}, PerceptionOracleConfig{}, 0.0, a);
    const ConeObservations second =
        oracle_observations(track.cones, car, LidarGeometry{}, PerceptionOracleConfig{}, 0.0, b);
    ASSERT_EQ(first.cones.size(), second.cones.size());
    ASSERT_FALSE(first.cones.empty());
    for (std::size_t i = 0; i < first.cones.size(); ++i) {
        EXPECT_EQ(first.cones[i].position, second.cones[i].position);
    }
}

TEST(OracleState, IsTheTruth) {
    const VehicleState truth = car_at(3.0, 4.0, 0.5);
    const StateEstimate s = oracle_state(truth, 2.0);
    EXPECT_TRUE(s.valid);
    EXPECT_DOUBLE_EQ(s.stamp, 2.0);
    EXPECT_DOUBLE_EQ(s.pose.x, 3.0);
    EXPECT_DOUBLE_EQ(s.pose.yaw, 0.5);
    EXPECT_DOUBLE_EQ(s.vx, 8.0);
    EXPECT_DOUBLE_EQ(s.yaw_rate, 0.2);
    EXPECT_TRUE(s.pose_covariance.isZero());
}

}  // namespace
}  // namespace apex::runner
