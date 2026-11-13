#include "apex/runner/oracle.hpp"

#include <cmath>
#include <vector>

#include <Eigen/LU>
#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/track.hpp"

namespace apex::runner {
namespace {

VehicleState car_at(double x, double y, double yaw) {
    VehicleState state;
    state.pose = {x, y, yaw};
    state.vx = 8.0;
    state.yaw_rate = 0.2;
    return state;
}

TEST(OracleObservations, EveryConeInRangeAndFieldOfView) {
    const LidarGeometry lidar;
    std::vector<Cone> cones;
    // Car at the origin facing +y; the LiDAR is 1.2 m ahead of the CoG, at (0, 1.2).
    cones.push_back({0, {0.0, 10.0}, ConeColor::Blue});        // ahead
    cones.push_back({1, {5.0, 1.3}, ConeColor::Yellow});       // just inside +-90 degrees
    cones.push_back({2, {5.0, 1.1}, ConeColor::Yellow});       // just behind the LiDAR's plane
    cones.push_back({3, {0.0, 1.2 + 26.0}, ConeColor::Blue});  // out of range
    cones.push_back({4, {-3.0, 3.0}, ConeColor::BigOrange});
    PerceptionOracleConfig config;
    config.sigma = 0.0;
    config.sigma_per_meter = 0.0;
    Rng rng(1);
    const ConeObservations obs =
        oracle_observations(cones, car_at(0.0, 0.0, kPi / 2), lidar, config, 1.5, rng);
    EXPECT_DOUBLE_EQ(obs.stamp, 1.5);
    ASSERT_EQ(obs.cones.size(), 3U);
    const std::vector<std::int32_t> ids = {obs.cones[0].truth_id, obs.cones[1].truth_id,
                                           obs.cones[2].truth_id};
    EXPECT_EQ(ids, (std::vector<std::int32_t>{0, 1, 4}));
    // Body frame: x forward (+y in the world), y left (-x in the world).
    EXPECT_NEAR(obs.cones[0].position.x(), 10.0, 1e-12);
    EXPECT_NEAR(obs.cones[0].position.y(), 0.0, 1e-12);
    EXPECT_NEAR(obs.cones[2].position.x(), 3.0, 1e-12);
    EXPECT_NEAR(obs.cones[2].position.y(), 3.0, 1e-12);
    EXPECT_EQ(obs.cones[2].color, ConeColor::BigOrange);
    EXPECT_DOUBLE_EQ(obs.cones[2].color_confidence, 1.0);
    EXPECT_GT(obs.cones[0].covariance.determinant(), 0.0);
}

TEST(OracleObservations, NoiseGrowsWithRangeAsConfigured) {
    const LidarGeometry lidar;
    const std::vector<Cone> cones = {{0, {1.2 + 20.0, 0.0}, ConeColor::Blue}};
    PerceptionOracleConfig config;
    config.sigma = 0.05;
    config.sigma_per_meter = 0.01;
    const double sigma = 0.05 + 0.01 * 20.0;
    Rng rng(7);
    double sum = 0.0;
    double sum_squares = 0.0;
    constexpr int kDraws = 4000;
    for (int i = 0; i < kDraws; ++i) {
        const ConeObservations obs =
            oracle_observations(cones, car_at(0.0, 0.0, 0.0), lidar, config, 0.0, rng);
        ASSERT_EQ(obs.cones.size(), 1U);
        EXPECT_NEAR(obs.cones[0].covariance(0, 0), sigma * sigma, 1e-12);
        const double error = obs.cones[0].position.x() - 21.2;
        sum += error;
        sum_squares += error * error;
    }
    EXPECT_NEAR(sum / kDraws, 0.0, 4.0 * sigma / std::sqrt(kDraws));
    EXPECT_NEAR(std::sqrt(sum_squares / kDraws), sigma, 0.05 * sigma);
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
