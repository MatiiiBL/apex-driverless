#include "apex/core/trajectory.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

namespace apex {
namespace {

Trajectory circle_trajectory(double radius) {
    std::vector<Vec2> waypoints;
    for (int i = 0; i < 36; ++i) waypoints.push_back(radius * heading_vector(i * kTwoPi / 36.0));
    return Trajectory::from_waypoints(waypoints, true, 0.5);
}

Trajectory straight_line() {
    std::vector<TrajectoryPoint> points(11);
    for (std::size_t i = 0; i < points.size(); ++i) {
        points[i].position = {static_cast<double>(i), 0.0};
        points[i].speed = static_cast<double>(i);
    }
    return Trajectory(std::move(points), false);
}

TEST(Trajectory, LengthAndClosure) {
    const Trajectory circle = circle_trajectory(10.0);
    EXPECT_TRUE(circle.closed());
    EXPECT_NEAR(circle.length(), kTwoPi * 10.0, 0.01);
    EXPECT_NEAR(circle[0].curvature, 0.1, 1e-3);
    const Trajectory line = straight_line();
    EXPECT_DOUBLE_EQ(line.length(), 10.0);
    EXPECT_DOUBLE_EQ(line[3].s, 3.0);
}

TEST(Trajectory, WrapAndDelta) {
    const Trajectory circle = circle_trajectory(10.0);
    const double length = circle.length();
    EXPECT_NEAR(circle.wrap_s(length + 1.0), 1.0, 1e-9);
    EXPECT_NEAR(circle.wrap_s(-1.0), length - 1.0, 1e-9);
    EXPECT_NEAR(circle.delta_s(length - 1.0, 1.0), 2.0, 1e-9);
    EXPECT_NEAR(circle.delta_s(1.0, length - 1.0), -2.0, 1e-9);
    const Trajectory line = straight_line();
    EXPECT_DOUBLE_EQ(line.wrap_s(12.0), 10.0);
    EXPECT_DOUBLE_EQ(line.delta_s(7.0, 2.0), -5.0);
}

TEST(Trajectory, InterpolatesFields) {
    const Trajectory line = straight_line();
    const TrajectoryPoint p = line.at(2.5);
    EXPECT_DOUBLE_EQ(p.position.x(), 2.5);
    EXPECT_DOUBLE_EQ(p.speed, 2.5);
    EXPECT_DOUBLE_EQ(line.at(20.0).position.x(), 10.0);
    // On a closed path the closing segment interpolates back to the first point.
    const Trajectory circle = circle_trajectory(10.0);
    const TrajectoryPoint last = circle.at(circle.length() - 1e-6);
    EXPECT_NEAR((last.position - circle[0].position).norm(), 0.0, 1e-4);
}

TEST(Trajectory, ProjectionSignsLeftPositive) {
    const Trajectory line = straight_line();
    const Trajectory::Projection left = line.project({4.2, 1.5});
    EXPECT_NEAR(left.s, 4.2, 1e-12);
    EXPECT_NEAR(left.lateral, 1.5, 1e-12);
    EXPECT_NEAR(line.project({6.0, -2.0}).lateral, -2.0, 1e-12);
    // Counter-clockwise circle: the center is on the left.
    const Trajectory circle = circle_trajectory(10.0);
    EXPECT_GT(circle.project({0.0, 9.0}).lateral, 0.9);
    EXPECT_LT(circle.project({0.0, 11.0}).lateral, -0.9);
}

TEST(Trajectory, LocalProjectionAgreesNearTheHint) {
    const Trajectory circle = circle_trajectory(10.0);
    const Vec2 p{9.5, 1.0};
    const Trajectory::Projection global = circle.project(p);
    const Trajectory::Projection local = circle.project(p, global.s + 2.0, 5.0);
    EXPECT_NEAR(local.s, global.s, 1e-9);
    EXPECT_NEAR(local.lateral, global.lateral, 1e-9);
    // Across the seam: hint just before the end, point just after the start.
    const Trajectory::Projection seam = circle.project({10.2, 0.3}, circle.length() - 0.5, 3.0);
    EXPECT_LT(seam.s, 1.0);
    EXPECT_NEAR(seam.distance, circle.project({10.2, 0.3}).distance, 1e-12);
}

TEST(Trajectory, SetProfilesAndValidate) {
    Trajectory line = straight_line();
    const std::vector<double> speed(line.size(), 3.0);
    const std::vector<double> width(line.size(), 1.5);
    line.set_speed_profile(speed);
    line.set_widths(width, width);
    EXPECT_DOUBLE_EQ(line.at(4.5).speed, 3.0);
    EXPECT_DOUBLE_EQ(line.at(4.5).width_left, 1.5);
    EXPECT_THROW(line.set_speed_profile(std::vector<double>(3, 1.0)), std::invalid_argument);
    std::vector<TrajectoryPoint> repeated(3);
    EXPECT_THROW(Trajectory(repeated, false), std::invalid_argument);
}

}  // namespace
}  // namespace apex
