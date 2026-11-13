#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "apex/control/dual.hpp"
#include "apex/control/matrix_exponential.hpp"
#include "apex/control/mpc_controller.hpp"
#include "apex/control/mpc_model.hpp"
#include "apex/control/mpc_plan.hpp"
#include "apex/control/qp/ocp_qp.hpp"
#include "apex/core/angles.hpp"
#include "apex/core/track.hpp"

#include "closed_loop.hpp"
#include "osqp_reference.hpp"
#include "plant.hpp"
#include "references.hpp"

namespace apex::control {
namespace {

using mpc::Input;
using mpc::State;

TEST(Dual, GradientsMatchFiniteDifferences) {
    auto f = [](const auto& x, const auto& y) {
        using std::atan;
        using std::atan2;
        using std::cos;
        using std::sin;
        using std::sqrt;
        using std::tan;
        using std::tanh;
        return sin(x) * y + atan2(y, x) + sqrt(x) * tanh(y) / (1.0 + x * x) - cos(y) / x +
               tan(0.3 * x) * atan(y) - 2.0 * x + 3.0 - y / 4.0;
    };
    const double x = 0.7;
    const double y = -1.3;
    const Dual<2> value = f(Dual<2>::variable(x, 0), Dual<2>::variable(y, 1));
    constexpr double h = 1e-6;
    EXPECT_NEAR(value.value, f(x, y), 1e-14);
    EXPECT_NEAR(value.grad(0), (f(x + h, y) - f(x - h, y)) / (2 * h), 1e-8);
    EXPECT_NEAR(value.grad(1), (f(x, y + h) - f(x, y - h)) / (2 * h), 1e-8);
}

// The degree-8 Taylor polynomial of a matrix scaled to norm 1/2 is accurate to ~5e-9 relative
// (matrix_exponential.hpp); squaring keeps that.
TEST(MatrixExponential, MatchesClosedFormsAndIdentities) {
    // A rotation generator: exp([0 -w; w 0] t) is the rotation by w t.
    Eigen::Matrix2d generator;
    generator << 0.0, -3.0, 3.0, 0.0;
    const Eigen::Matrix2d rotation = matrix_exponential<2>(generator);
    EXPECT_NEAR(rotation(0, 0), std::cos(3.0), 1e-8);
    EXPECT_NEAR(rotation(1, 0), std::sin(3.0), 1e-8);
    // A stiff diagonal, as the lateral dynamics are at low speed.
    const Eigen::Vector3d diagonal{-40.0, 0.5, 2.0};
    const Eigen::Matrix3d e = matrix_exponential<3>(Eigen::Matrix3d(diagonal.asDiagonal()));
    for (int i = 0; i < 3; ++i) {
        EXPECT_NEAR(e(i, i), std::exp(diagonal(i)), 1e-8 * std::exp(diagonal(i)) + 1e-18);
    }
    // exp(M) exp(-M) = I for a dense matrix with a large norm.
    Eigen::Matrix<double, 5, 5> m;
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) m(i, j) = std::sin(1.0 + 3.0 * i + 7.0 * j) * 2.0;
    }
    const Eigen::Matrix<double, 5, 5> product =
        matrix_exponential<5>(m) * matrix_exponential<5>(Eigen::Matrix<double, 5, 5>(-m));
    EXPECT_LT((product - Eigen::Matrix<double, 5, 5>::Identity()).lpNorm<Eigen::Infinity>(), 1e-8);
}

// The discrete model is the exact zero-order hold of the model linearized at (x, u). Its
// matrices are the continuous Jacobians to first order in the step, its step from (x, u)
// differs from the nonlinear model's only at second order in the step (the curvature of the
// tire forces), and it integrates the steering exactly.
TEST(MpcModel, DiscretizationIsTheLinearizedZeroOrderHold) {
    const NominalModel model{VehicleParams{}};
    State x;
    x << 0.1, 0.05, 12.0, 0.2, 0.8, 0.1;
    Input u;
    u << 0.5, 1.0, 0.0;
    constexpr double kCurvature = 1.0 / 15.0;
    auto f = [&](const State& state, const Input& input) {
        const mpc::StateArray<double> d = mpc::error_dynamics(
            model, mpc::to_array(state), input(mpc::kSteerRate), input(mpc::kAccel), kCurvature);
        return State(Eigen::Map<const State>(d.data()));
    };
    constexpr double kTiny = 1e-5;
    const mpc::DiscreteModel tiny = mpc::discretize(model, x, u, kCurvature, kTiny);
    for (int i = 0; i < mpc::kStates; ++i) {
        State h = State::Zero();
        h(i) = 1e-6;
        const State column = (f(x + h, u) - f(x - h, u)) / 2e-6;
        const State discrete = (tiny.A.col(i) - State::Unit(i)) / kTiny;
        EXPECT_LT((discrete - column).lpNorm<Eigen::Infinity>(),
                  1e-3 * (1.0 + column.lpNorm<Eigen::Infinity>()))
            << "column " << i;
    }
    auto step_error = [&](double dt) {
        const mpc::DiscreteModel d = mpc::discretize(model, x, u, kCurvature, dt);
        const State linear = d.A * x + d.B * u + d.c;
        return (linear - mpc::simulate(model, x, u, kCurvature, dt, 200)).lpNorm<Eigen::Infinity>();
    };
    // At least second order: halving the step divides the error by four or more.
    EXPECT_GT(step_error(0.02) / step_error(0.01), 3.5);
    const mpc::DiscreteModel d = mpc::discretize(model, x, u, kCurvature, 0.05);
    EXPECT_NEAR((d.A * x + d.B * u + d.c)(mpc::kSteer), x(mpc::kSteer) + 0.05 * u(mpc::kSteerRate),
                1e-12);
    // The slack does not enter the dynamics.
    EXPECT_EQ(d.B.col(mpc::kSlack).norm(), 0.0);
}

// The warm start is kept in the world frame, so it means the same thing on a reference that
// was replanned with a different arc-length origin: the plan's inputs at the same times, rolled
// out from the same state, give the same trajectory.
TEST(MpcPlan, SurvivesAReplannedReference) {
    const NominalModel model{VehicleParams{}};
    const Trajectory circle = testing::circle_reference({0.0, 20.0}, 20.0, 8.0);
    constexpr std::size_t kStages = 21;
    constexpr double kStep = 0.05;
    std::vector<State> states(kStages);
    std::vector<Input> inputs(kStages);
    for (std::size_t k = 0; k < kStages; ++k) {
        const double t = static_cast<double>(k) * kStep;
        states[k] << 0.2 - 0.1 * t, 0.03 - 0.02 * t, 8.0 + t, 0.1, 0.4, 0.1;
        inputs[k] << 0.1, 1.0, 0.0;
    }
    mpc::Plan plan;
    mpc::plan_from_solution(circle, 10.0, states, inputs, 1.0, kStep, plan);
    ASSERT_EQ(plan.stages().size(), kStages);

    auto guess_on = [&](const Trajectory& reference, double s0) {
        std::vector<mpc::StagePoint> guess(kStages);
        mpc::guess_from_plan(model, plan, reference, states[0], s0, 1.0, kStep, guess);
        for (std::size_t k = 0; k + 1 < kStages; ++k) {
            EXPECT_NEAR(guess[k].u(mpc::kAccel), inputs[k](mpc::kAccel), 1e-9) << k;
            EXPECT_NEAR(guess[k].u(mpc::kSteerRate), inputs[k](mpc::kSteerRate), 1e-9) << k;
        }
        return guess;
    };
    const std::vector<mpc::StagePoint> same = guess_on(circle, 10.0);
    // A window of the circle that starts 6 m behind the car: every station is 4 m less.
    std::vector<TrajectoryPoint> points;
    for (double s = 4.0; s <= 40.0; s += 0.5) points.push_back(circle.at(s));
    const Trajectory window(std::move(points), false);
    const std::vector<mpc::StagePoint> moved = guess_on(window, 6.0);
    for (std::size_t k = 0; k < kStages; ++k) {
        EXPECT_NEAR(moved[k].s, same[k].s - 4.0, 5e-3) << k;
        EXPECT_LT((moved[k].x - same[k].x).lpNorm<Eigen::Infinity>(), 5e-3) << k;
    }
}

// The rollout is what the QP is linearized around, so it must be a trajectory the car can
// drive whatever inputs the previous plan left: steering within its angle and rate limits,
// acceleration within the drive and brake limits, and no reversing.
TEST(MpcPlan, RolloutStaysWithinTheCarsLimits) {
    const VehicleParams params;
    const NominalModel model(params);
    const Trajectory line = testing::straight_reference({0.0, 0.0}, 0.0, 60.0, 5.0);
    std::vector<mpc::StagePoint> guess(31);
    for (mpc::StagePoint& point : guess) point.u << 5.0, -40.0, -1.0;  // all out of range
    State x0;
    x0 << 0.0, 0.0, 3.0, 0.0, 0.0, 0.35;
    mpc::rollout(model, line, x0, 0.0, 0.05, guess);
    for (std::size_t k = 0; k < guess.size(); ++k) {
        const mpc::StagePoint& point = guess[k];
        EXPECT_LE(std::abs(point.x(mpc::kSteer)), params.max_steer + 1e-12) << k;
        EXPECT_GE(point.x(mpc::kVx), 0.0) << k;
        if (k + 1 < guess.size()) {
            EXPECT_LE(std::abs(point.u(mpc::kSteerRate)), params.max_steer_rate) << k;
            EXPECT_GE(point.u(mpc::kAccel), -model.max_brake_decel()) << k;
            EXPECT_GE(point.u(mpc::kSlack), 0.0) << k;
        }
    }
    // Full braking from 3 m/s stops the car within the horizon, and it stays stopped.
    EXPECT_EQ(guess.back().x(mpc::kVx), 0.0);
    EXPECT_NEAR(guess.back().s, guess[guess.size() - 2].s, 1e-12);
}

// The controller's own QPs, from a lap of the oval near the grip limit, solved by OSQP.
TEST(Mpc, ItsQpsMatchOsqp) {
    const VehicleParams params;
    Trajectory oval = make_oval_track(40.0, 12.0).centerline;
    testing::apply_speed_limits(oval, {});
    MpcController mpc(MpcConfig{}, params);
    testing::Plant plant(params, {oval[0].position.x(), oval[0].position.y(), oval[0].yaw}, 0.0);
    int compared = 0;
    for (int i = 0; i < 400; ++i) {
        const ControlCommand c = mpc.compute(plant.estimate(), oval, plant.time());
        plant.command(c);
        plant.advance(testing::kControlPeriod);
        if (i % 40 != 39) continue;
        const mpc::Qp& problem = mpc.last_problem();
        const mpc::Solution& ours = mpc.last_solution();
        ASSERT_TRUE(qp::usable(ours.status));
        const qp::QpResiduals residuals = qp::kkt_residuals(problem, ours);
        EXPECT_LT(residuals.stationarity, 1e-5);
        EXPECT_LT(residuals.dynamics, 1e-8);
        EXPECT_LT(residuals.inequality, 1e-6);

        testing::OsqpOptions options;
        options.eps_abs = 1e-8;
        options.polishing = true;
        const testing::OsqpResult osqp =
            testing::solve_with_osqp(testing::to_sparse(problem), options);
        ASSERT_TRUE(osqp.solved) << osqp.status;
        const mpc::Solution reference = testing::from_sparse(problem, osqp);
        double difference = 0.0;
        for (std::size_t k = 0; k < ours.u.size(); ++k) {
            difference =
                std::max(difference, (ours.u[k] - reference.u[k]).lpNorm<Eigen::Infinity>());
            difference =
                std::max(difference, (ours.x[k] - reference.x[k]).lpNorm<Eigen::Infinity>());
        }
        EXPECT_LT(difference, 1e-3) << "call " << i;
        EXPECT_NEAR(ours.objective, reference.objective,
                    1e-6 * (1.0 + std::abs(reference.objective)));
        ++compared;
    }
    EXPECT_EQ(compared, 10);
    EXPECT_EQ(mpc.stats().solver_failures, 0U);
}

// Lap 1's exploration paths carry widths from a partial map: near the car a quarter of them
// say the path is closer to a cone line than half the car, which a drivable path cannot be.
// Bounding the car by them pushed it off a path it could have followed, into the cones on the
// other side; open references are therefore not bounded unless asked.
TEST(Mpc, DoesNotBoundOpenReferencesByTheirWidths) {
    const VehicleParams params;
    const Trajectory circle = testing::circle_reference({0.0, 20.0}, 20.0, 8.0);
    auto worst_lateral = [&](bool bound_open) {
        MpcConfig config;
        config.bound_open_references = bound_open;
        MpcController mpc(config, params);
        testing::Plant plant(params, {0.0, 0.0, 0.0}, 8.0);
        Trajectory window;
        Time planned = -1.0;
        auto reference_at = [&](Time now) -> const Trajectory& {
            if (now >= planned + 0.1 - 1e-9) {
                const double s = circle.project(plant.truth().pose.position()).s;
                std::vector<TrajectoryPoint> points;
                for (double d = 0.0; d <= 25.0; d += 0.5) {
                    TrajectoryPoint point = circle.at(s + d);
                    if (d < 4.0) point.width_right = 0.2;  // a chord between sparse cones
                    points.push_back(point);
                }
                window = Trajectory(std::move(points), false);
                planned = now;
            }
            return window;
        };
        const std::vector<testing::LoopSample> samples =
            testing::run_loop(mpc, plant, reference_at, 4.0);
        double worst = 0.0;
        for (const testing::LoopSample& sample : samples) {
            worst = std::max(worst, std::abs(circle.project(sample.truth.pose.position()).lateral));
        }
        EXPECT_EQ(mpc.stats().solver_failures, 0U);
        return worst;
    };
    EXPECT_LT(worst_lateral(false), 0.1);
    // Bounded, the MPC keeps the car's body 0.15 m off the reported cone line: half a metre
    // off its path.
    EXPECT_GT(worst_lateral(true), 0.4);
}

TEST(Mpc, FallsBackSafelyWhenTheSolverFails) {
    const VehicleParams params;
    MpcConfig config;
    config.max_qp_iterations = 1;  // no QP can converge
    MpcController mpc(config, params);
    const Trajectory circle = testing::circle_reference({0.0, 20.0}, 20.0, 9.0);
    testing::Plant plant(params, {0.0, -0.3, 0.0}, 9.0);
    const std::vector<testing::LoopSample> samples = testing::run_loop(mpc, plant, circle, 3.0);
    EXPECT_EQ(mpc.stats().solver_failures, mpc.stats().calls);
    EXPECT_TRUE(mpc.plan().empty());
    for (const testing::LoopSample& s : samples) {
        ASSERT_TRUE(std::isfinite(s.command.steer) && std::isfinite(s.command.accel));
    }
    // Stanley keeps it on the circle while the car brakes.
    EXPECT_LT(testing::max_lateral_after(samples, 0.0), 0.5);
    EXPECT_LT(samples.back().truth.vx, 9.0 - 0.8 * config.fallback_decel * 2.0);
}

// Near the grip limit the 60 ms of steering delay matter: predicting through them must help.
TEST(Mpc, DeadTimeCompensationReducesTrackingError) {
    VehicleParams params;
    params.steer_dead_time = 0.06;
    Trajectory oval = make_oval_track(40.0, 12.0).centerline;
    testing::apply_speed_limits(oval, {});
    auto rms = [&](bool compensate) {
        MpcConfig config;
        config.compensate_dead_time = compensate;
        MpcController mpc(config, params);
        testing::Plant plant(params, {oval[0].position.x(), oval[0].position.y(), oval[0].yaw},
                             0.0);
        const std::vector<testing::LoopSample> samples = testing::run_loop(mpc, plant, oval, 10.0);
        double sum = 0.0;
        for (const testing::LoopSample& s : samples) sum += s.lateral * s.lateral;
        return std::sqrt(sum / static_cast<double>(samples.size()));
    };
    const double with = rms(true);
    const double without = rms(false);
    EXPECT_LT(with, 0.7 * without) << with << " vs " << without;
    EXPECT_LT(with, 0.05);
}

}  // namespace
}  // namespace apex::control
