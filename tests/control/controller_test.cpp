#include "apex/control/controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "apex/control/vehicle_model.hpp"
#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/track.hpp"

#include "closed_loop.hpp"
#include "plant.hpp"
#include "references.hpp"

// Behaviour every controller must show, whatever its law: it converges onto a line and a
// circle, respects the actuator limits, survives replanned references and the end of an open
// one, never returns a non-finite command, and is deterministic.
namespace apex::control {
namespace {

using testing::LoopSample;
using testing::Plant;

class ControllerBehaviour : public ::testing::TestWithParam<ControllerKind> {
protected:
    [[nodiscard]] std::unique_ptr<Controller> make() const {
        ControllerConfig config;
        config.kind = GetParam();
        return make_controller(config, params_);
    }

    VehicleParams params_;
};

std::string kind_name(const ::testing::TestParamInfo<ControllerKind>& info) {
    return std::string(to_string(info.param));
}

TEST_P(ControllerBehaviour, ConvergesOntoAStraightLine) {
    const Trajectory line = testing::straight_reference({0.0, 0.0}, 0.0, 300.0, 10.0);
    Plant plant(params_, {5.0, 1.0, 0.0}, 10.0);
    const auto controller = make();
    const std::vector<LoopSample> samples = testing::run_loop(*controller, plant, line, 6.0);
    EXPECT_LT(testing::max_lateral_after(samples, 4.0), 0.05);
    EXPECT_LT(std::abs(samples.back().truth.pose.yaw), 0.01);
    EXPECT_NEAR(samples.back().truth.vx, 10.0, 0.2);
}

TEST_P(ControllerBehaviour, ConvergesOntoACircle) {
    // 20 m radius at 9 m/s: 4 m/s^2 of lateral acceleration, a quarter of the grip.
    const Trajectory circle = testing::circle_reference({0.0, 20.0}, 20.0, 9.0);
    Plant plant(params_, {0.0, -0.6, 0.0}, 9.0);
    const auto controller = make();
    const std::vector<LoopSample> samples = testing::run_loop(*controller, plant, circle, 8.0);
    EXPECT_LT(testing::max_lateral_after(samples, 5.0), 0.15);
    EXPECT_NEAR(samples.back().truth.vx, 9.0, 0.3);
}

TEST_P(ControllerBehaviour, RespectsActuatorLimitsNearTheGripLimit) {
    Trajectory centerline = make_oval_track(40.0, 12.0).centerline;
    testing::SpeedLimits limits;
    limits.lateral = 13.0;
    limits.accel = 7.0;
    limits.brake = 12.0;
    testing::apply_speed_limits(centerline, limits);
    const Pose2 start{centerline[0].position.x(), centerline[0].position.y(), centerline[0].yaw};
    Plant plant(params_, start, 0.0);
    const auto controller = make();
    const NominalModel model(params_);
    const std::vector<LoopSample> samples = testing::run_loop(*controller, plant, centerline, 12.0);
    double previous = 0.0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const ControlCommand& c = samples[i].command;
        EXPECT_LE(std::abs(c.steer), params_.max_steer + 1e-12);
        if (i > 0) {
            EXPECT_LE(std::abs(c.steer - previous),
                      params_.max_steer_rate * testing::kControlPeriod + 1e-12);
        }
        EXPECT_GE(c.accel, -model.max_brake_decel() - 1e-12);
        EXPECT_LE(c.accel, model.max_drive_accel(samples[i].truth.vx) + 1e-12);
        previous = c.steer;
    }
    // It laps the 155 m oval and stays on the track, the CoG well inside the cone lines.
    double driven = 0.0;
    for (std::size_t i = 1; i < samples.size(); ++i) {
        driven += (samples[i].truth.pose.position() - samples[i - 1].truth.pose.position()).norm();
    }
    EXPECT_GT(driven, 155.0);
    EXPECT_LT(testing::max_lateral_after(samples, 0.0), testing::kHalfWidth - 0.25);
}

// During lap 1 the exploration planner replans every 100 ms: a new open path that starts
// behind the car, with its own arc-length origin and length. Tracking must not notice.
TEST_P(ControllerBehaviour, FollowsAReferenceReplannedEvery100Ms) {
    const Trajectory circle = testing::circle_reference({0.0, 20.0}, 20.0, 8.0);
    Plant plant(params_, {0.0, 0.0, 0.0}, 8.0);
    const auto controller = make();
    Trajectory window;
    Time planned = -1.0;
    int replans = 0;
    auto reference_at = [&](Time now) -> const Trajectory& {
        if (now >= planned + 0.1 - 1e-9) {
            // A window from a little behind the car to 25-35 m ahead of it.
            const double s = circle.project(plant.truth().pose.position()).s;
            const double behind = 2.0 + static_cast<double>(replans % 3);
            const double ahead = 25.0 + 5.0 * static_cast<double>(replans % 3);
            std::vector<TrajectoryPoint> points;
            for (double d = -behind; d <= ahead; d += 0.5) points.push_back(circle.at(s + d));
            window = Trajectory(std::move(points), false);
            planned = now;
            ++replans;
        }
        return window;
    };
    const std::vector<LoopSample> samples =
        testing::run_loop(*controller, plant, reference_at, 6.0);
    double worst = 0.0;
    for (const LoopSample& s : samples) {
        worst = std::max(worst, std::abs(circle.project(s.truth.pose.position()).lateral));
    }
    EXPECT_LT(worst, 0.2);
    EXPECT_NEAR(samples.back().truth.vx, 8.0, 0.4);
}

// Lap 1 as the stack drives it: every 100 ms the exploration planner hands over a new open
// path from the car to 25-30 m ahead, whose speed profile ends at rest there (the car must be
// able to stop within what it has seen). The end moves forward with every replan, so the
// previous plan's braking for it is always out of date; through the oval's curves the MPC used
// to stall its QP solver on exactly that, fall back, and then linearize around its own
// fallback until the commands oscillated between the limits.
TEST_P(ControllerBehaviour, FollowsAnExplorationPathThatEndsAtRest) {
    const Trajectory centerline = make_oval_track(40.0, 12.0).centerline;
    const Pose2 start{centerline[0].position.x(), centerline[0].position.y(), centerline[0].yaw};
    Plant plant(params_, start, 10.0);
    const auto controller = make();
    Trajectory window;
    Time planned = -1.0;
    int replans = 0;
    auto reference_at = [&](Time now) -> const Trajectory& {
        if (now >= planned + 0.1 - 1e-9) {
            const double s = centerline.project(plant.truth().pose.position()).s;
            const double ahead = 25.0 + 2.5 * static_cast<double>(replans % 3);
            std::vector<TrajectoryPoint> points;
            for (double d = 0.0; d <= ahead; d += 0.5) points.push_back(centerline.at(s + d));
            window = Trajectory(std::move(points), false);
            // As the planner's profile: from the car's speed, 12 m/s at most, braking to rest at
            // the end as hard as the tires allow.
            testing::SpeedLimits limits;
            limits.top = 12.0;
            limits.accel = 11.0;
            limits.brake = 12.0;
            limits.lateral = 12.0;
            limits.start = std::max(plant.truth().vx, 1.0);
            limits.end = 0.0;
            testing::apply_speed_limits(window, limits);
            planned = now;
            ++replans;
        }
        return window;
    };
    // 12 s: a straight, a curve, the other straight and into the second curve.
    const std::vector<LoopSample> samples =
        testing::run_loop(*controller, plant, reference_at, 12.0);
    double worst = 0.0;
    double slowest = 1e9;
    double accel_jumps = 0.0;  // squared change of the acceleration command between calls
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const LoopSample& sample = samples[i];
        worst = std::max(worst, std::abs(centerline.project(sample.truth.pose.position()).lateral));
        if (sample.time > 1.0) slowest = std::min(slowest, sample.truth.vx);
        if (i > 0) accel_jumps += std::pow(sample.command.accel - samples[i - 1].command.accel, 2);
    }
    EXPECT_EQ(controller->stats().solver_failures, 0U);
    EXPECT_LT(worst, 0.2);
    // It keeps going at the profile's pace (12 m/s on the straights, 11 m/s in the 12 m curves
    // at 12 m/s^2) instead of braking for an end it never reaches.
    EXPECT_GT(slowest, 9.0);
    EXPECT_LT(std::sqrt(accel_jumps / static_cast<double>(samples.size())), 1.0);
}

// The exploration path ends where the car must have stopped: a profile down to 0 m/s.
TEST_P(ControllerBehaviour, StopsAtTheEndOfAnOpenReference) {
    Trajectory line = testing::straight_reference({0.0, 0.0}, 0.0, 60.0, 0.0);
    testing::SpeedLimits limits;
    limits.accel = 4.0;
    limits.brake = 4.0;
    limits.start = 8.0;
    limits.end = 0.0;
    testing::apply_speed_limits(line, limits);
    Plant plant(params_, {0.0, 0.3, 0.0}, 8.0);
    const auto controller = make();
    const std::vector<LoopSample> samples = testing::run_loop(*controller, plant, line, 16.0);
    const VehicleState& end = samples.back().truth;
    EXPECT_LT(end.vx, 0.05);
    EXPECT_GT(end.pose.x, 55.0);
    EXPECT_LT(end.pose.x, 61.0);
    EXPECT_LT(std::abs(end.pose.y), 0.1);
    EXPECT_LE(samples.back().command.accel, 0.0);
}

TEST_P(ControllerBehaviour, NeverReturnsNonFiniteCommands) {
    constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    Trajectory oval = make_oval_track(40.0, 12.0).centerline;
    testing::apply_speed_limits(oval, {});
    const Trajectory line = testing::straight_reference({0.0, 0.0}, 0.3, 30.0, 5.0);
    const Trajectory empty;
    const NominalModel model(params_);
    const auto controller = make();
    Rng rng(42);
    Time now = 0.0;
    for (int i = 0; i < 400; ++i) {
        StateEstimate state;
        state.valid = rng.bernoulli(0.9);
        state.pose = {rng.uniform(-60.0, 60.0), rng.uniform(-60.0, 60.0), rng.uniform(-4.0, 4.0)};
        state.vx = rng.uniform(-3.0, 35.0);
        state.vy = rng.uniform(-4.0, 4.0);
        state.yaw_rate = rng.uniform(-3.0, 3.0);
        if (rng.bernoulli(0.05)) state.vx = rng.bernoulli(0.5) ? kNan : kInf;
        if (rng.bernoulli(0.05)) state.pose.yaw = kNan;
        now += rng.bernoulli(0.1) ? rng.uniform(-0.5, 1.0) : 0.02;
        state.stamp = now - rng.uniform(0.0, 0.05);
        const Trajectory& reference = rng.bernoulli(0.05)  ? empty
                                      : rng.bernoulli(0.5) ? oval
                                                           : line;
        const ControlCommand c = controller->compute(state, reference, now);
        ASSERT_TRUE(std::isfinite(c.steer) && std::isfinite(c.accel)) << "call " << i;
        EXPECT_LE(std::abs(c.steer), params_.max_steer);
        EXPECT_GE(c.accel, -model.max_brake_decel());
        EXPECT_LE(c.accel, model.max_drive_accel(0.0));
    }
    EXPECT_EQ(controller->stats().calls, 400U);
    EXPECT_GT(controller->stats().invalid_inputs, 0U);
}

TEST_P(ControllerBehaviour, IsDeterministicAndResetsCleanly) {
    const Trajectory circle = testing::circle_reference({0.0, 20.0}, 20.0, 9.0);
    auto run = [&](Controller& controller) {
        Plant plant(params_, {0.0, -0.4, 0.05}, 7.0);
        return testing::run_loop(controller, plant, circle, 2.0);
    };
    const auto first = make();
    const auto second = make();
    const std::vector<LoopSample> a = run(*first);
    const std::vector<LoopSample> b = run(*second);
    first->reset();
    const std::vector<LoopSample> c = run(*first);
    ASSERT_EQ(a.size(), b.size());
    ASSERT_EQ(a.size(), c.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].command.steer, b[i].command.steer);
        EXPECT_EQ(a[i].command.accel, b[i].command.accel);
        EXPECT_EQ(a[i].command.steer, c[i].command.steer);
        EXPECT_EQ(a[i].command.accel, c[i].command.accel);
    }
    EXPECT_EQ(first->stats().calls, 2 * a.size());
    EXPECT_EQ(first->stats().compute_time.count(), 2 * a.size());
}

INSTANTIATE_TEST_SUITE_P(Controllers, ControllerBehaviour,
                         ::testing::Values(ControllerKind::PurePursuit, ControllerKind::Stanley,
                                           ControllerKind::Mpc),
                         kind_name);

}  // namespace
}  // namespace apex::control
