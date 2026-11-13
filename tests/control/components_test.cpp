#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "apex/control/reference.hpp"
#include "apex/control/speed_controller.hpp"
#include "apex/control/vehicle_model.hpp"
#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

#include "references.hpp"

// The pieces under the controllers: the speed loop, the reference view and the nominal model.
namespace apex::control {
namespace {

TEST(SpeedController, FeedForwardHoldsASteadySpeed) {
    const NominalModel model{VehicleParams{}};
    SpeedController loop(SpeedLoopConfig{}, model);
    // On target with no reference acceleration, the command is exactly the resistance.
    EXPECT_NEAR(loop.update(15.0, 0.0, 15.0, 0.02), model.resistance(15.0), 1e-12);
    EXPECT_EQ(loop.integral(), 0.0);
    // The reference acceleration passes straight through.
    EXPECT_NEAR(loop.update(15.0, 2.0, 15.0, 0.02), 2.0 + model.resistance(15.0), 1e-12);
}

TEST(SpeedController, IntegralIsClampedAndDoesNotWindUpInSaturation) {
    const NominalModel model{VehicleParams{}};
    SpeedLoopConfig config;
    SpeedController loop(config, model);
    // Far below the target the drive limit saturates: the integral must not grow.
    for (int i = 0; i < 500; ++i) {
        EXPECT_LE(loop.update(30.0, 0.0, 5.0, 0.02), model.max_drive_accel(5.0) + 1e-12);
    }
    EXPECT_EQ(loop.integral(), 0.0);
    // A small persistent error that does not saturate integrates up to the clamp.
    for (int i = 0; i < 5000; ++i) (void)loop.update(10.2, 0.0, 10.0, 0.02);
    EXPECT_NEAR(loop.integral(), config.integral_limit, 1e-12);
    loop.reset();
    EXPECT_EQ(loop.integral(), 0.0);
}

TEST(SpeedController, HoldsTheBrakeAtAStandstill) {
    const NominalModel model{VehicleParams{}};
    SpeedLoopConfig config;
    SpeedController loop(config, model);
    EXPECT_EQ(loop.update(0.0, 0.0, 0.1, 0.02), -config.hold_brake);
    // A profile that starts from rest has zero speed but a positive acceleration: go.
    EXPECT_GT(loop.update(0.0, 3.0, 0.0, 0.02), 0.0);
}

TEST(Reference, ExtendsAnOpenPathBeyondItsEnds) {
    const Trajectory line = testing::straight_reference({1.0, 2.0}, 0.5, 20.0, 4.0);
    const TrajectoryPoint beyond = sample_reference(line, 25.0);
    const Vec2 expected = Vec2{1.0, 2.0} + 25.0 * heading_vector(0.5);
    EXPECT_LT((beyond.position - expected).norm(), 1e-9);
    EXPECT_EQ(beyond.speed, 4.0);
    EXPECT_EQ(beyond.curvature, 0.0);
    const TrajectoryPoint before = sample_reference(line, -3.0);
    EXPECT_LT((before.position - (Vec2{1.0, 2.0} - 3.0 * heading_vector(0.5))).norm(), 1e-9);
}

TEST(Reference, ProjectionInvertsSampling) {
    const Trajectory line = testing::straight_reference({0.0, 0.0}, 0.0, 20.0, 4.0);
    for (const double s : {-4.0, 3.0, 19.5, 24.0}) {
        for (const double lateral : {-1.0, 0.0, 0.7}) {
            const Vec2 p = sample_reference(line, s).position + lateral * Vec2{0.0, 1.0};
            const PathCoordinates c = extend_projection(line, p, line.project(p));
            EXPECT_NEAR(c.s, s, 1e-9);
            EXPECT_NEAR(c.lateral, lateral, 1e-9);
        }
    }
}

TEST(Reference, TrackerDoesNotJumpToANearbyLegOfTheTrack) {
    // A hairpin: out along y = 0, back along y = 5.
    std::vector<TrajectoryPoint> points;
    auto add = [&](const Vec2& p, double yaw) {
        TrajectoryPoint t;
        t.position = p;
        t.yaw = yaw;
        points.push_back(t);
    };
    for (double x = 0.0; x <= 30.0; x += 0.5) add({x, 0.0}, 0.0);
    for (int i = 1; i < 16; ++i) {
        const double a = -0.5 * kPi + kPi * i / 16.0;
        add(Vec2{30.0, 2.5} + 2.5 * heading_vector(a), a + 0.5 * kPi);
    }
    for (double x = 30.0; x >= 0.0; x -= 0.5) add({x, 5.0}, kPi);
    const Trajectory hairpin(std::move(points), false);

    ReferenceTracker tracker;
    PathCoordinates c = tracker.locate(hairpin, {5.0, 0.2}, 1.0);
    EXPECT_TRUE(tracker.reference_changed());
    EXPECT_NEAR(c.s, 5.0, 1e-9);
    // Drifting towards the return leg: 2.6 m from the first, 2.4 m from the second.
    c = tracker.locate(hairpin, {6.0, 2.6}, 1.0);
    EXPECT_FALSE(tracker.reference_changed());
    EXPECT_NEAR(c.s, 6.0, 1e-9);
    EXPECT_NEAR(c.lateral, 2.6, 1e-9);
    // A global projection would have picked the return leg.
    EXPECT_GT(hairpin.project({6.0, 2.6}).s, 30.0);
}

// Steady cornering with the slip angles the model says it needs: the lateral and yaw
// accelerations of the dynamic model must vanish.
TEST(NominalModel, SteadyStateSlipBalancesTheTurn) {
    const VehicleParams params;
    const NominalModel model(params);
    for (const double speed : {8.0, 15.0}) {
        for (const double curvature : {0.02, -0.05}) {
            const SlipAngles slip = model.steady_state_slip(curvature, speed);
            const double r = speed * curvature;
            const double vy = speed * std::tan(slip.rear) + params.lr * r;
            const double delta = std::atan2(vy + params.lf * r, speed) - slip.front;
            const BodyRates<double> rates = model.body_rates(speed, vy, r, delta, 0.0, 0.0);
            const double lateral = speed * speed * std::abs(curvature);
            EXPECT_LT(std::abs(rates.vy_dot), 0.01 * lateral) << speed << " " << curvature;
            EXPECT_LT(std::abs(rates.yaw_accel), 0.05) << speed << " " << curvature;
            // Turning left takes negative slip angles.
            EXPECT_LT(slip.front * curvature, 0.0);
        }
    }
}

TEST(NominalModel, IsKinematicAtLowSpeed) {
    const VehicleParams params;
    const NominalModel model(params);
    // Below the blend, the yaw acceleration is d/dt (v tan(delta) / L).
    const double v = 2.0;
    const double delta = 0.2;
    const double rate = 0.5;
    const double accel = 1.0;
    const BodyRates<double> rates = model.body_rates(v, 0.0, 0.0, delta, rate, accel);
    const double v_dot = accel - model.resistance(v);
    const double expected =
        (v_dot * std::tan(delta) + v * rate / std::pow(std::cos(delta), 2)) / params.wheelbase();
    EXPECT_NEAR(rates.yaw_accel, expected, 1e-12);
    EXPECT_NEAR(rates.vx_dot, v_dot, 1e-12);
}

TEST(NominalModel, TractionLimitsShrinkWithLateralDemand) {
    const VehicleParams params;
    const NominalModel model(params);
    const AccelRange straight = model.traction_limits(10.0, 0.0, 1.0);
    // On a straight the front axle, with 40% of the drive and unloaded by the transfer, limits
    // the drive below the motors' 12.5 m/s^2; the brakes are traction-limited too.
    EXPECT_LT(straight.max, model.max_drive_accel(10.0));
    EXPECT_GT(straight.max, 8.0);
    EXPECT_GT(straight.min, -model.max_brake_decel());
    // The front axle's drive force fits its grip exactly at the limit.
    const double transfer = params.mass * params.cg_height / params.wheelbase();
    const double front_load = params.mass * kGravity * params.lr / params.wheelbase() +
                              params.aero_balance_front * params.downforce(10.0);
    EXPECT_NEAR(params.drive_split_front * params.mass * straight.max,
                params.front_tire.mu * (front_load - transfer * straight.max), 1e-6);
    // Cornering at 12 m/s^2 leaves less, but never nothing.
    const AccelRange corner = model.traction_limits(10.0, 12.0, 1.0);
    EXPECT_LT(corner.max, 0.6 * straight.max);
    EXPECT_GT(corner.min, 0.6 * straight.min);
    EXPECT_GT(corner.max, 0.0);
    EXPECT_LT(corner.min, 0.0);
    // A smaller grip fraction is more conservative.
    EXPECT_LT(model.traction_limits(10.0, 0.0, 0.8).max, straight.max);
}

TEST(NominalModel, ResistanceAndLimits) {
    const VehicleParams params;
    const NominalModel model(params);
    EXPECT_NEAR(model.resistance(0.0), 0.0, 1e-12);
    const double v = 20.0;
    const double expected =
        (params.drag(v) + params.rolling_resistance *
                              (params.mass * kGravity + params.downforce(v)) * std::tanh(v / 0.5)) /
        params.mass;
    EXPECT_NEAR(model.resistance(v), expected, 1e-12);
    EXPECT_NEAR(model.max_drive_accel(5.0), params.max_drive_force / params.mass, 1e-12);
    EXPECT_NEAR(model.max_drive_accel(30.0), params.max_power / 30.0 / params.mass, 1e-12);
    EXPECT_NEAR(model.max_brake_decel(), params.max_brake_force / params.mass, 1e-12);
}

}  // namespace
}  // namespace apex::control
