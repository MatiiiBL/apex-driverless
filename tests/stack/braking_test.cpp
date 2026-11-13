#include "apex/stack/braking.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/track.hpp"

namespace apex::stack {
namespace {

const VehicleParams kCar;
const planning::SpeedProfileConfig kEnvelope;

Trajectory straight(double length) {
    std::vector<TrajectoryPoint> points;
    for (double x = 0.0; x <= length; x += 0.5) {
        TrajectoryPoint p;
        p.position = {x, 0.0};
        p.yaw = 0.0;
        p.speed = 30.0;
        p.width_left = 1.5;
        p.width_right = 1.5;
        points.push_back(p);
    }
    return Trajectory(std::move(points), false);
}

// A circle of radius r, driven counter-clockwise.
Trajectory circle(double r) {
    std::vector<Vec2> waypoints;
    for (int i = 0; i < 72; ++i) {
        const double a = 2.0 * 3.141592653589793 * i / 72.0;
        waypoints.emplace_back(r * std::cos(a), r * std::sin(a));
    }
    return Trajectory::from_waypoints(waypoints, true, 0.5);
}

// Arc length at which the reference comes to rest.
double stop_distance(const Trajectory& ref) {
    for (const TrajectoryPoint& p : ref.points()) {
        if (p.speed == 0.0) return p.s;
    }
    return ref.length();
}

TEST(BrakingReference, FromTheCarsSpeedToRestThenHolds) {
    constexpr double kSpeed = 20.0;
    const Trajectory ref = braking_reference(straight(300.0), 10.0, kSpeed, kCar, kEnvelope, 5.0);
    EXPECT_FALSE(ref.closed());
    EXPECT_NEAR(ref[0].position.x(), 10.0, 1e-9);
    EXPECT_NEAR(ref[0].speed, kSpeed, 1e-3);
    for (std::size_t i = 1; i < ref.size(); ++i) {
        EXPECT_LE(ref[i].speed, ref[i - 1].speed + 1e-9);
        EXPECT_LE(ref[i - 1].accel, 1e-9);
    }
    EXPECT_NEAR(ref.length() - stop_distance(ref), 5.0, 0.5);
    EXPECT_EQ(ref.points().back().speed, 0.0);
    EXPECT_DOUBLE_EQ(ref[0].width_left, 1.5);
    // Braking at the envelope's limit on a straight: firmer than 0.5 g, within the tires' 1.6.
    const double average = kSpeed * kSpeed / (2.0 * stop_distance(ref));
    EXPECT_GT(average, 5.0);
    EXPECT_LT(average, 1.6 * 9.81 * 1.5);
}

TEST(BrakingReference, IsTheShortestStopTheEnvelopeAllows) {
    constexpr double kSpeed = 18.0;
    const Trajectory path = straight(300.0);
    const Trajectory ref = braking_reference(path, 0.0, kSpeed, kCar, kEnvelope, 0.0);
    const double distance = stop_distance(ref);
    // Over a stretch one meter shorter the envelope cannot start from the car's speed.
    std::vector<TrajectoryPoint> shorter;
    for (const TrajectoryPoint& p : ref.points()) {
        if (p.s <= distance - 1.0) shorter.push_back(p);
    }
    Trajectory tight(std::move(shorter), false);
    planning::apply_speed_profile(tight, kCar, kEnvelope, kSpeed, 0.0);
    EXPECT_LT(tight[0].speed, kSpeed - 0.01);
    // Nowhere faster than the envelope's profile towards a later stop.
    Trajectory loose = path;
    planning::apply_speed_profile(loose, kCar, kEnvelope, kSpeed, 0.0);
    for (const TrajectoryPoint& p : ref.points()) EXPECT_LE(p.speed, loose.at(p.s).speed + 1e-6);
}

TEST(BrakingReference, StopsLaterInACurve) {
    // Cornering near its limit uses most of the friction ellipse, so less is left for braking.
    constexpr double kSpeed = 12.0;
    const double on_straight =
        stop_distance(braking_reference(straight(300.0), 0.0, kSpeed, kCar, kEnvelope, 0.0));
    const double in_curve =
        stop_distance(braking_reference(circle(12.0), 0.0, kSpeed, kCar, kEnvelope, 0.0));
    EXPECT_GT(in_curve, 1.1 * on_straight);
}

TEST(BrakingReference, WrapsAroundAClosedPath) {
    const Track track = make_oval_track();
    const double s_start = track.centerline.length() - 3.0;
    const Trajectory ref = braking_reference(track.centerline, s_start, 0.0, kCar, kEnvelope, 8.0);
    EXPECT_NEAR(ref.length(), 9.0, 0.6);
    EXPECT_LT(
        (ref.points().back().position - track.centerline.at(ref.length() - 3.0).position).norm(),
        0.05);
    for (const TrajectoryPoint& p : ref.points()) EXPECT_EQ(p.speed, 0.0);
}

TEST(BrakingReference, ExtendsAShortOpenPathStraightOn) {
    const Trajectory ref = braking_reference(straight(4.0), 2.0, 10.0, kCar, kEnvelope, 2.0);
    EXPECT_GT(ref.points().back().position.x(), 4.0 + 2.0);
    EXPECT_NEAR(ref.points().back().position.y(), 0.0, 1e-9);
    EXPECT_EQ(ref.points().back().speed, 0.0);
    EXPECT_NEAR(ref[0].speed, 10.0, 1e-3);
}

TEST(BrakingReference, RejectsInvalidArguments) {
    const Trajectory path = straight(10.0);
    EXPECT_THROW((void)braking_reference(path, 0.0, -1.0, kCar, kEnvelope), std::invalid_argument);
    EXPECT_THROW((void)braking_reference(path, 0.0, 5.0, kCar, kEnvelope, -1.0),
                 std::invalid_argument);
    EXPECT_THROW((void)braking_reference(Trajectory{}, 0.0, 5.0, kCar, kEnvelope),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apex::stack
