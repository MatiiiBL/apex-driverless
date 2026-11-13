#include "apex/sim/vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"

namespace apex::sim {
namespace {

constexpr double kControlPeriod = 0.02;  // s, the stack's control rate
constexpr int kStepsPerControl = 20;

double resistance(const VehicleParams& p, double v) {
    return p.drag(v) + p.rolling_resistance * (p.mass * kGravity + p.downforce(v));
}

VehicleState moving(double vx, double steer = 0.0) {
    VehicleState state;
    state.vx = vx;
    state.steer = steer;
    return state;
}

// Holds a speed with a PI loop on top of a drag and rolling-resistance feed-forward, as a
// controller would, while steering at a fixed angle. Runs `seconds` of simulated time.
class SpeedHold {
public:
    SpeedHold(VehicleModel& model, double speed) : model_(model), speed_(speed) {}
    template <class SteerAt>
    void run(double seconds, SteerAt steer_at) {
        const int periods = static_cast<int>(std::lround(seconds / kControlPeriod));
        for (int k = 0; k < periods; ++k) {
            const VehicleParams& p = model_.params();
            const double error = speed_ - model_.state().vx;
            integral_ += error * kControlPeriod;
            const double accel =
                resistance(p, model_.state().vx) / p.mass + 3.0 * error + 2.0 * integral_;
            model_.command({model_.time(), steer_at(model_.time()), accel});
            for (int i = 0; i < kStepsPerControl; ++i) {
                model_.step();
                on_step(model_);
            }
        }
    }
    void run(double seconds, double steer) {
        run(seconds, [steer](double) { return steer; });
    }
    std::function<void(const VehicleModel&)> on_step = [](const VehicleModel&) {};

private:
    VehicleModel& model_;
    double speed_;
    double integral_ = 0.0;
};

TEST(VehicleModel, RejectsInvalidParameters) {
    auto invalid = [](auto mutate) {
        VehicleParams params;
        mutate(params);
        return params;
    };
    EXPECT_THROW(VehicleModel(invalid([](VehicleParams& p) { p.mass = 0.0; })),
                 std::invalid_argument);
    EXPECT_THROW(VehicleModel(invalid([](VehicleParams& p) { p.lr = -0.1; })),
                 std::invalid_argument);
    EXPECT_THROW(VehicleModel(invalid([](VehicleParams& p) { p.front_tire.mu = 0.0; })),
                 std::invalid_argument);
    EXPECT_THROW(VehicleModel(invalid([](VehicleParams& p) {
                     p.aero_balance_front = std::numeric_limits<double>::quiet_NaN();
                 })),
                 std::invalid_argument);
    EXPECT_THROW(VehicleModel(invalid([](VehicleParams& p) { p.drive_split_front = 1.5; })),
                 std::invalid_argument);
    EXPECT_THROW(VehicleModel(invalid([](VehicleParams& p) { p.steer_time_constant = 0.0; })),
                 std::invalid_argument);
    EXPECT_THROW(VehicleModel(invalid([](VehicleParams& p) { p.max_steer = 2.0; })),
                 std::invalid_argument);
    EXPECT_THROW(VehicleModel(VehicleParams{}, 0.0), std::invalid_argument);
    EXPECT_NO_THROW(VehicleModel(VehicleParams{}, 0.5));
}

TEST(VehicleModel, StaysAtRestWithoutCommands) {
    VehicleModel model{VehicleParams{}};
    model.reset(VehicleState{{3.0, -2.0, 1.0}});
    for (int i = 0; i < 1000; ++i) model.step();
    const VehicleState& s = model.state();
    EXPECT_EQ(s.pose.x, 3.0);
    EXPECT_EQ(s.pose.y, -2.0);
    EXPECT_EQ(s.pose.yaw, 1.0);
    EXPECT_EQ(s.vx, 0.0);
    EXPECT_EQ(s.ax, 0.0);
    EXPECT_EQ(s.ay, 0.0);
    EXPECT_NEAR(model.time(), 1.0, 1e-9);
    // Static load split: lr / L of the weight on the front axle.
    const VehicleParams p;
    EXPECT_NEAR(model.tire_forces().fz_front, p.mass * kGravity * p.lr / p.wheelbase(), 1e-9);
}

// Dead time, then a first-order lag; a large step is rate limited instead.
TEST(VehicleModel, ActuatorsHaveDeadTimeLagAndLimits) {
    const VehicleParams p;
    VehicleModel model{p};
    model.reset(moving(10.0));
    model.command({0.0, 0.02, 2.0});
    // Steps start at multiples of 1 ms; a command due at 30 ms acts from the step starting there.
    auto run_until = [&](double t) {
        while (model.time() < t - 1e-9) model.step();
    };
    run_until(p.drive_dead_time);  // 20 ms
    EXPECT_NEAR(model.state().force, 0.0, 1e-12);
    run_until(p.steer_dead_time);  // 30 ms
    EXPECT_NEAR(model.state().steer, 0.0, 1e-12);
    run_until(p.steer_dead_time + p.steer_time_constant);  // 60 ms
    EXPECT_NEAR(model.state().steer, 0.02 * (1.0 - std::exp(-1.0)), 1e-8);
    run_until(p.drive_dead_time + p.drive_time_constant);  // 70 ms
    EXPECT_NEAR(model.state().force, 2.0 * p.mass * (1.0 - std::exp(-1.0)), 1e-6);

    // A full-lock step moves at the rate limit at first, and stops at max_steer.
    model.command({0.0, 5.0, 0.0});
    const double start = model.time();
    run_until(start + p.steer_dead_time + 0.1);
    const double expected = 0.02 + p.max_steer_rate * 0.1;
    EXPECT_NEAR(model.state().steer, expected, 0.01);
    run_until(start + 2.0);
    EXPECT_NEAR(model.state().steer, p.max_steer, 1e-9);
}

TEST(VehicleModel, IgnoresNonFiniteCommandFields) {
    VehicleModel model{VehicleParams{}};
    model.reset(moving(5.0));
    model.command({0.0, 0.1, 1.0});
    model.command(
        {0.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()});
    for (int i = 0; i < 2000; ++i) model.step();
    EXPECT_NEAR(model.state().steer, 0.1, 1e-6);
    EXPECT_NEAR(model.state().force, 1.0 * model.params().mass, 1e-3);
}

// At walking pace the car follows the kinematic bicycle: the CoG's path curvature is
// tan(d) cos(beta) / L with sideslip beta = atan(lr tan(d) / L), whatever the speed.
TEST(VehicleModel, LowSpeedPathCurvatureIsKinematic) {
    const VehicleParams p;
    for (const double speed : {1.0, 2.0}) {
        for (const double steer : {0.1, 0.3}) {
            SCOPED_TRACE(testing::Message() << speed << " m/s, steer " << steer);
            VehicleModel model{p};
            model.reset(moving(speed, steer));
            SpeedHold(model, speed).run(5.0, steer);
            const VehicleState& s = model.state();
            const double beta = std::atan(p.lr * std::tan(steer) / p.wheelbase());
            const double expected = std::tan(steer) * std::cos(beta) / p.wheelbase();
            const double curvature = s.yaw_rate / std::hypot(s.vx, s.vy);
            EXPECT_NEAR(curvature / expected, 1.0, 0.005);
            EXPECT_NEAR(std::atan2(s.vy, s.vx), beta, 0.005);
        }
    }
}

// Steady-state cornering at small slip against the linear single-track model:
//   r = v d / (L + K v^2),  K = (m / L) (lr / Cf - lf / Cr),
// with axle cornering stiffness C = B C mu Fz (the magic formula's slope at zero slip) and Fz
// from weight and downforce. Drag and rolling resistance are switched off so no drive force
// shifts the loads. Checked on the default (near neutral) car and on strongly understeering and
// oversteering variants, at lateral accelerations near 2 m/s^2.
TEST(VehicleModel, SteadyStateCorneringMatchesLinearModel) {
    VehicleParams neutral;
    neutral.drag_area = 0.0;
    neutral.rolling_resistance = 0.0;
    VehicleParams understeer = neutral;
    understeer.front_tire.B = 6.0;
    VehicleParams oversteer = neutral;
    oversteer.rear_tire.B = 8.0;
    for (const VehicleParams& p : {neutral, understeer, oversteer}) {
        for (const double speed : {8.0, 12.0, 16.0}) {
            const double L = p.wheelbase();
            const double downforce = p.downforce(speed);
            const double fz_front = p.mass * kGravity * p.lr / L + p.aero_balance_front * downforce;
            const double fz_rear =
                p.mass * kGravity * p.lf / L + (1.0 - p.aero_balance_front) * downforce;
            const TireParams& tf = p.front_tire;
            const TireParams& tr = p.rear_tire;
            const double cf = tf.B * tf.C * tf.mu * fz_front;
            const double cr = tr.B * tr.C * tr.mu * fz_rear;
            const double gradient = p.mass / L * (p.lr / cf - p.lf / cr);
            const double steer = (L + gradient * speed * speed) * 2.0 / (speed * speed);
            const double expected = speed * steer / (L + gradient * speed * speed);
            SCOPED_TRACE(testing::Message()
                         << "B " << tf.B << "/" << tr.B << ", " << speed << " m/s, K " << gradient);

            VehicleModel model{p};
            model.reset(moving(speed, steer));
            SpeedHold(model, speed).run(4.0, steer);
            EXPECT_NEAR(model.state().vx, speed, 0.01);
            EXPECT_NEAR(model.state().yaw_rate / expected, 1.0, 0.02);
            EXPECT_NEAR(model.state().ay, speed * model.state().yaw_rate, 0.01);
        }
    }
}

// Coasting in a straight line: m v' = -(a + b v^2) with a = c_rr m g (rolling resistance on
// the weight) and b = rho (CdA + c_rr ClA) / 2 (drag, and rolling resistance on the
// downforce). With c = sqrt(a b) / m and theta(t) = atan(v0 sqrt(b / a)) - c t:
//   v(t) = sqrt(a / b) tan(theta(t)),   x(t) = (m / b) ln(cos(theta(t)) / cos(theta(0))).
TEST(VehicleModel, CoastDownMatchesAnalyticSolution) {
    const VehicleParams p;
    const double a = p.rolling_resistance * p.mass * kGravity;
    const double b = 0.5 * p.air_density * (p.drag_area + p.rolling_resistance * p.downforce_area);
    const double v0 = 25.0;
    const double theta0 = std::atan(v0 * std::sqrt(b / a));
    const double c = std::sqrt(a * b) / p.mass;
    VehicleModel model{p};
    model.reset(moving(v0));
    for (int second = 1; second <= 20; ++second) {
        for (int i = 0; i < 1000; ++i) model.step();
        const double theta = theta0 - c * second;
        const double v = std::sqrt(a / b) * std::tan(theta);
        const double x = p.mass / b * std::log(std::cos(theta) / std::cos(theta0));
        EXPECT_NEAR(model.state().vx, v, 1e-9 * v0) << second << " s";
        EXPECT_NEAR(model.state().pose.x, x, 1e-8 * x) << second << " s";
        EXPECT_NEAR(model.state().ax, -(a + b * v * v) / p.mass, 1e-8);
    }
    EXPECT_EQ(model.state().pose.y, 0.0);
}

// Flat out in a straight line, the motors deliver max_power / v once that is below
// max_drive_force: the acceleration is (P / v - drag - rolling) / m. A 40 kW car makes the
// power-limited range wide (above 13.3 m/s).
TEST(VehicleModel, AccelerationIsPowerLimitedAtSpeed) {
    VehicleParams p;
    p.max_power = 40e3;
    VehicleModel model{p};
    model.reset(moving(14.0));
    int checked = 0;
    while (model.state().vx < 26.0) {
        model.command({model.time(), 0.0, 20.0});
        for (int i = 0; i < kStepsPerControl; ++i) {
            model.step();
            const double v = model.state().vx;
            const TireForces& f = model.tire_forces();
            EXPECT_LE((f.fx_front + f.fx_rear) * v, p.max_power * (1.0 + 1e-12));
            if (model.time() < 0.3) continue;  // the drive actuator is still rising
            EXPECT_NEAR(model.state().ax, (p.max_power / v - resistance(p, v)) / p.mass, 1e-6);
            ++checked;
        }
    }
    EXPECT_GT(checked, 1000);
}

TEST(VehicleModel, TopSpeedIsCapped) {
    const VehicleParams p;
    VehicleModel model{p};
    model.reset(moving(25.0));
    double top = 0.0;
    for (int k = 0; k < 1000; ++k) {
        model.command({model.time(), 0.0, 20.0});
        for (int i = 0; i < kStepsPerControl; ++i) {
            model.step();
            top = std::max(top, model.state().vx);
        }
    }
    EXPECT_LT(top, p.max_speed);
    EXPECT_GT(top, p.max_speed - 1.0);
}

// Full braking stops the car and holds it: no creeping backwards, and an IMU at rest reads zero.
// Under braking the front axle carries more than its static load.
TEST(VehicleModel, BrakesToAStandstillWithoutReversing) {
    const VehicleParams p;
    VehicleModel model{p};
    model.reset(moving(15.0));
    bool loaded_front = false;
    for (int k = 0; k < 250; ++k) {
        model.command({model.time(), 0.0, -20.0});
        for (int i = 0; i < kStepsPerControl; ++i) {
            model.step();
            ASSERT_GE(model.state().vx, 0.0);
            if (model.state().vx > 5.0 &&
                model.tire_forces().fz_front > p.mass * kGravity * p.lr / p.wheelbase() + 300.0) {
                loaded_front = true;
            }
        }
    }
    EXPECT_TRUE(loaded_front);
    EXPECT_EQ(model.state().vx, 0.0);
    EXPECT_NEAR(model.state().vy, 0.0, 1e-9);
    EXPECT_NEAR(model.state().yaw_rate, 0.0, 1e-9);
    EXPECT_EQ(model.state().ax, 0.0);
    // About v^2 / (2 mu g), plus the distance covered during the dead time and lag.
    const double friction_distance = 15.0 * 15.0 / (2.0 * p.front_tire.mu * kGravity);
    EXPECT_GT(model.state().pose.x, friction_distance);
    EXPECT_LT(model.state().pose.x, friction_distance + 1.5);
}

// The largest lateral acceleration in a turn tightened slowly at constant speed: no more than
// friction allows, mu (g + downforce / m), and close to it, growing with speed as the downforce
// does and scaling with the friction coefficient.
TEST(VehicleModel, MaximumLateralAccelerationFollowsFrictionAndDownforce) {
    const VehicleParams p;
    auto max_lateral = [&](double speed, double friction_scale) {
        VehicleModel model{p, friction_scale};
        model.reset(moving(speed));
        double best = 0.0;
        SpeedHold hold(model, speed);
        hold.on_step = [&](const VehicleModel& m) {
            const double limit =
                p.front_tire.mu * friction_scale * (kGravity + p.downforce(m.state().vx) / p.mass);
            EXPECT_LE(m.state().ay, limit * (1.0 + 1e-9));
            best = std::max(best, m.state().ay / limit);
        };
        hold.run(8.0, [](double t) { return 0.05 * t; });
        EXPECT_EQ(model.recoveries(), 0U);
        return best;
    };
    for (const double speed : {10.0, 20.0}) {
        for (const double friction : {1.0, 0.5}) {
            SCOPED_TRACE(testing::Message() << speed << " m/s, friction x" << friction);
            // The front axle saturates first (the car understeers at the limit), and the drive
            // force that holds the speed against drag and cornering drag shifts load off it:
            // 5-12% short of the whole car's friction limit (results/sim/vehicle_validation.json).
            EXPECT_GT(max_lateral(speed, friction), 0.85);
        }
    }
}

// RK4 is fourth order: halving the step shrinks the change in a smooth 5 s manoeuvre
// (sinusoidal steering, varying acceleration, no actuator limits reached) about 16-fold.
TEST(VehicleModel, HalvingTheStepConvergesAtFourthOrder) {
    auto run = [](double dt) {
        VehicleModel model{VehicleParams{}};
        model.reset(moving(10.0));
        const int steps = static_cast<int>(std::lround(kControlPeriod / dt));
        for (int k = 0; k < 250; ++k) {
            const double t = k * kControlPeriod;
            model.command({t, 0.05 * std::sin(kPi * t), 1.0 + 0.5 * std::cos(2.0 * t)});
            for (int i = 0; i < steps; ++i) model.step(dt);
        }
        const VehicleState& s = model.state();
        return Eigen::Matrix<double, 6, 1>(s.pose.x, s.pose.y, s.pose.yaw, s.vx, s.vy, s.yaw_rate);
    };
    const auto coarse = run(1e-3);
    const auto medium = run(5e-4);
    const auto fine = run(2.5e-4);
    const double change_coarse = (coarse - medium).norm();
    const double change_medium = (medium - fine).norm();
    EXPECT_GT(change_coarse, 0.0);
    const double order = std::log2(change_coarse / change_medium);
    EXPECT_NEAR(order, 4.0, 0.3) << change_coarse << " " << change_medium;
}

// Random commands within the actuator limits, held for random durations: hard launches, full
// braking, full lock at speed, spins. The state stays finite and within its limits.
TEST(VehicleModel, RandomCommandsKeepTheStateFinite) {
    const VehicleParams p;
    for (std::uint64_t seed = 1; seed <= 8; ++seed) {
        SCOPED_TRACE(seed);
        Rng rng(seed);
        VehicleModel model{p, rng.uniform(0.5, 1.2)};
        model.reset(moving(rng.uniform(0.0, 25.0)));
        while (model.time() < 20.0) {
            const ControlCommand command{
                model.time(), rng.uniform(-p.max_steer, p.max_steer),
                rng.uniform(-p.max_brake_force, p.max_drive_force) / p.mass};
            const int periods = 1 + static_cast<int>(rng.index(50));
            for (int k = 0; k < periods; ++k) {
                model.command(command);
                for (int i = 0; i < kStepsPerControl; ++i) model.step();
                const VehicleState& s = model.state();
                const bool finite = std::isfinite(s.pose.x) && std::isfinite(s.pose.y) &&
                                    std::isfinite(s.pose.yaw) && std::isfinite(s.vx) &&
                                    std::isfinite(s.vy) && std::isfinite(s.yaw_rate) &&
                                    std::isfinite(s.ax) && std::isfinite(s.ay) &&
                                    std::isfinite(s.force);
                ASSERT_TRUE(finite) << "t = " << model.time();
                ASSERT_GE(s.vx, 0.0);
                ASSERT_LE(std::abs(s.steer), p.max_steer);
                ASSERT_LE(s.vx, p.max_speed);
            }
        }
        EXPECT_EQ(model.recoveries(), 0U);
    }
}

}  // namespace
}  // namespace apex::sim
