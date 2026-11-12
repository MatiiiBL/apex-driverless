#include "apex/stack/braking.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/track.hpp"

namespace apex::stack {
namespace {

Trajectory straight(double length, double speed) {
    std::vector<TrajectoryPoint> points;
    for (double x = 0.0; x <= length; x += 0.5) {
        TrajectoryPoint p;
        p.position = {x, 0.0};
        p.speed = speed;
        p.width_left = 1.5;
        p.width_right = 1.5;
        points.push_back(p);
    }
    return Trajectory(std::move(points), false);
}

TEST(BrakingReference, ConstantDecelerationToAStandstill) {
    constexpr double kSpeed = 12.0;
    constexpr double kDecel = 6.0;
    const Trajectory ref = braking_reference(straight(100.0, 20.0), 10.0, kSpeed, kDecel, 5.0);
    EXPECT_FALSE(ref.closed());
    // 12^2 / (2 * 6) = 12 m to stop, plus the 5 m margin.
    EXPECT_NEAR(ref.length(), 17.0, 0.5);
    EXPECT_NEAR(ref[0].position.x(), 10.0, 1e-9);
    EXPECT_NEAR(ref[0].speed, kSpeed, 1e-9);
    for (const TrajectoryPoint& p : ref.points()) {
        const double expected = std::sqrt(std::max(kSpeed * kSpeed - 2.0 * kDecel * p.s, 0.0));
        EXPECT_NEAR(p.speed, expected, 1e-9);
        EXPECT_GE(p.accel, -kDecel - 1e-6);
        EXPECT_DOUBLE_EQ(p.width_left, 1.5);
    }
    EXPECT_EQ(ref.points().back().speed, 0.0);
}

TEST(BrakingReference, NeverFasterThanThePath) {
    const Trajectory ref = braking_reference(straight(100.0, 5.0), 0.0, 12.0, 6.0);
    for (const TrajectoryPoint& p : ref.points()) EXPECT_LE(p.speed, 5.0 + 1e-12);
}

TEST(BrakingReference, WrapsAroundAClosedPath) {
    const Track track = make_oval_track();
    const double s_start = track.centerline.length() - 3.0;
    const Trajectory ref = braking_reference(track.centerline, s_start, 0.0, 5.0, 8.0);
    EXPECT_NEAR(ref.length(), 8.0, 0.1);
    EXPECT_LT((ref.points().back().position - track.centerline.at(5.0).position).norm(), 0.05);
    for (const TrajectoryPoint& p : ref.points()) EXPECT_EQ(p.speed, 0.0);
}

TEST(BrakingReference, ExtendsAShortOpenPathStraightOn) {
    const Trajectory ref = braking_reference(straight(4.0, 20.0), 2.0, 10.0, 5.0, 2.0);
    // 10 m to stop + 2 m margin from x = 2, on a path that ends at x = 4.
    EXPECT_NEAR(ref.points().back().position.x(), 14.0, 0.5);
    EXPECT_NEAR(ref.points().back().position.y(), 0.0, 1e-9);
    EXPECT_EQ(ref.points().back().speed, 0.0);
}

TEST(BrakingReference, RejectsInvalidArguments) {
    const Trajectory path = straight(10.0, 5.0);
    EXPECT_THROW((void)braking_reference(path, 0.0, 5.0, 0.0), std::invalid_argument);
    EXPECT_THROW((void)braking_reference(path, 0.0, -1.0, 5.0), std::invalid_argument);
    EXPECT_THROW((void)braking_reference(Trajectory{}, 0.0, 5.0, 5.0), std::invalid_argument);
}

}  // namespace
}  // namespace apex::stack
