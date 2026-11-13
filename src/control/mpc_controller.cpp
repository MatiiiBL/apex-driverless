#include "apex/control/mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>

#include "apex/control/qp/interior_point.hpp"
#include "apex/control/reference.hpp"
#include "apex/control/vehicle_model.hpp"
#include "apex/core/angles.hpp"
#include "apex/core/log.hpp"
#include "apex/core/timing.hpp"

#include "common.hpp"
#include "delay_prediction.hpp"
#include "mpc_qp.hpp"

namespace apex::control {

namespace {

using mpc::kAccel;
using mpc::kHeading;
using mpc::kSteer;
using mpc::kSteerRate;

// At rest with a reference that asks to stay there, the car is held by the brake, as the speed
// loop of the geometric controllers does.
constexpr double kStandstillSpeed = 0.3;  // m/s
constexpr double kHoldBrake = 1.5;        // m/s^2
// The estimate is predicted forward from its stamp by at most this (s): an older one is stale.
constexpr double kMaxPrediction = 0.25;
// Fallback steering: Stanley's law with the defaults of StanleyConfig.
constexpr double kFallbackGain = 2.0;
constexpr double kFallbackSoftening = 1.0;

const MpcConfig& validated(const MpcConfig& c) {
    const bool ok = c.horizon >= 2 && c.horizon <= 200 && c.step > 0.0 && c.step <= 0.5 &&
                    c.weight_lateral >= 0.0 && c.weight_course >= 0.0 && c.weight_speed >= 0.0 &&
                    c.weight_steer_rate > 0.0 && c.weight_accel > 0.0 && c.terminal_scale >= 0.0 &&
                    c.lateral_margin >= 0.0 && c.slack_linear >= 0.0 && c.slack_quadratic > 0.0 &&
                    c.grip_fraction > 0.0 && c.grip_fraction <= 1.0 && c.sqp_iterations >= 1 &&
                    c.initial_sqp_iterations >= 1 && c.max_qp_iterations >= 1 &&
                    c.qp_tolerance > 0.0 && c.fallback_plan_age >= 0.0 && c.fallback_decel > 0.0;
    if (!ok) {
        throw std::invalid_argument(
            "MpcConfig: need 2 <= horizon <= 200, 0 < step <= 0.5 s, non-negative weights with "
            "positive steer-rate, accel and slack_quadratic weights, at least one iteration, "
            "and positive tolerance and fallback deceleration");
    }
    return c;
}

}  // namespace

struct MpcController::Impl {
    Impl(const MpcConfig& c, const VehicleParams& nominal)
        : config(validated(c)),
          model(nominal, c.kinematic_below, c.dynamic_above),
          solver(solver_settings(c)) {
        const auto stages = static_cast<std::size_t>(config.horizon) + 1;
        guess.resize(stages);
        warm.x.resize(stages);
        warm.u.resize(stages);
        delays.steer = config.compensate_dead_time
                           ? nominal.steer_dead_time +
                                 (config.compensate_lag ? nominal.steer_time_constant : 0.0)
                           : 0.0;
        delays.drive = config.compensate_dead_time
                           ? nominal.drive_dead_time +
                                 (config.compensate_lag ? nominal.drive_time_constant : 0.0)
                           : 0.0;
    }

    static qp::InteriorPointSettings solver_settings(const MpcConfig& c) {
        qp::InteriorPointSettings s;
        s.max_iterations = c.max_qp_iterations;
        s.tolerance = c.qp_tolerance;
        s.complementarity_tolerance = c.qp_tolerance;
        return s;
    }

    ControlCommand compute(const StateEstimate& state, const Trajectory& reference, Time now);
    // The steering the actuator is assumed to hold before the first command: the kinematic
    // angle for the estimate's yaw rate.
    [[nodiscard]] double initial_steer(const StateEstimate& state) const;
    [[nodiscard]] ControlCommand safe_command(double dt, double speed, Time now) const;
    // Solves the QPs; false if the last one is not usable.
    bool solve(const Trajectory& reference, const mpc::State& x0, double s0, Time t0);
    void reset();

    MpcConfig config;
    NominalModel model;
    qp::InteriorPointSolver<mpc::kStates, mpc::kInputs> solver;
    ActuatorDelays delays;

    ControllerStats stats;
    CallClock clock;
    ReferenceTracker tracker;
    CommandHistory history;
    std::optional<ControlCommand> last;
    std::optional<double> assumed_steer;  // before the first command
    mpc::Plan plan;
    Time plan_time = 0.0;  // the call that made the plan
    bool failing = false;  // inside a streak of solver failures (for logging)

    std::vector<mpc::StagePoint> guess;
    mpc::Qp problem;
    mpc::Solution solution;
    mpc::Solution warm;
};

double MpcController::Impl::initial_steer(const StateEstimate& state) const {
    const double max_steer = model.params().max_steer;
    if (state.vx < 1.0) return 0.0;
    const double kinematic = std::atan(model.params().wheelbase() * state.yaw_rate / state.vx);
    return std::clamp(kinematic, -max_steer, max_steer);
}

ControlCommand MpcController::Impl::safe_command(double dt, double speed, Time now) const {
    return limit_command(last ? last->steer : 0.0, -kSafeDecel, last, dt, speed, model, now);
}

bool MpcController::Impl::solve(const Trajectory& reference, const mpc::State& x0, double s0,
                                Time t0) {
    const double dt = config.step;
    const bool fresh = plan.empty() || t0 - plan_time > config.fallback_plan_age + dt;
    if (fresh) {
        mpc::guess_from_reference(model, reference, x0, s0, dt, guess);
    } else {
        mpc::guess_from_plan(model, plan, reference, x0, s0, t0, dt, guess);
    }
    const int iterations = fresh ? config.initial_sqp_iterations : config.sqp_iterations;
    for (int i = 0; i < iterations; ++i) {
        if (i > 0) mpc::guess_from_solution(model, reference, solution, dt, guess);
        mpc::build_qp(config, model, guess, dt, reference.closed() || config.bound_open_references,
                      problem);
        for (std::size_t k = 0; k < guess.size(); ++k) {
            warm.x[k] = guess[k].x;
            warm.u[k] = guess[k].u;
        }
        const qp::QpStatus status = solver.solve(problem, solution, &warm);
        stats.qp_iterations += static_cast<std::size_t>(solution.iterations);
        if (!qp::usable(status)) {
            if (!failing) {
                log(LogLevel::Warn, "mpc: QP {} after {} iterations, using the fallback",
                    qp::to_string(status), solution.iterations);
            }
            failing = true;
            return false;
        }
    }
    if (failing) log(LogLevel::Info, "mpc: QP solved again");
    failing = false;
    return true;
}

ControlCommand MpcController::Impl::compute(const StateEstimate& state, const Trajectory& reference,
                                            Time now) {
    const double dt = clock.step(now);
    const double speed = std::isfinite(state.vx) ? std::max(state.vx, 0.0) : 0.0;
    if (!usable_inputs(state, reference) || !std::isfinite(now)) {
        ++stats.invalid_inputs;
        return safe_command(dt, speed, now);
    }
    const VehicleParams& p = model.params();
    if (!assumed_steer) assumed_steer = initial_steer(state);
    const ControlCommand before{now, *assumed_steer, 0.0};
    const double previous_steer = last ? last->steer : *assumed_steer;

    // 1. The state when a command sent now starts to act.
    const Time t0 = now + delays.steer;
    StateEstimate from = state;
    from.stamp = std::clamp(state.stamp, t0 - kMaxPrediction, t0);
    const WorldState predicted = predict_through_delays(model, from, t0, history, delays, before);

    // 2. Its error state on the reference.
    const double travel = speed * (dt + (t0 - from.stamp)) + 1.0;
    const PathCoordinates at = tracker.locate(reference, predicted.pose.position(), travel);
    const TrajectoryPoint here = sample_reference(reference, at.s);
    mpc::State x0;
    x0 << at.lateral, angle_diff(predicted.pose.yaw, here.yaw), predicted.vx, predicted.vy,
        predicted.yaw_rate, previous_steer;
    if (!x0.allFinite()) {
        ++stats.invalid_inputs;
        return safe_command(dt, speed, now);
    }

    // A profile that starts from rest has a zero speed but a positive acceleration: no hold.
    if (here.speed < kStandstillSpeed && here.accel <= 0.0 && predicted.vx < kStandstillSpeed) {
        plan.clear();
        return limit_command(previous_steer, -kHoldBrake, last, dt, speed, model, now);
    }

    // 3-5. Linearize, solve, command.
    double steer = 0.0;
    double accel = 0.0;
    if (solve(reference, x0, at.s, t0)) {
        mpc::plan_from_solution(reference, at.s, solution.x, solution.u, t0, config.step, plan);
        plan_time = t0;
        steer = solution.x[0](kSteer) + solution.u[0](kSteerRate) * dt;
        accel = solution.u[0](kAccel);
    } else {
        ++stats.solver_failures;
        if (!plan.empty() && t0 - plan_time <= config.fallback_plan_age) {
            const mpc::PlanStage stage = plan.at(t0 + dt);
            steer = stage.steer;
            accel = stage.accel;
        } else {
            plan.clear();
            const double front_lateral = at.lateral + p.lf * std::sin(x0(kHeading));
            steer = std::atan(p.wheelbase() * here.curvature) +
                    stanley_law(-x0(kHeading), front_lateral, predicted.vx, kFallbackGain,
                                kFallbackSoftening);
            accel = -config.fallback_decel;
        }
    }
    return limit_command(steer, accel, last, dt, speed, model, now);
}

void MpcController::Impl::reset() {
    clock.reset();
    tracker.reset();
    history.clear();
    last.reset();
    assumed_steer.reset();
    plan.clear();
    plan_time = 0.0;
    failing = false;
}

MpcController::MpcController(const MpcConfig& config, const VehicleParams& nominal)
    : impl_(std::make_unique<Impl>(config, nominal)) {}

MpcController::~MpcController() = default;

ControlCommand MpcController::compute(const StateEstimate& state, const Trajectory& reference,
                                      Time now) {
    const Stopwatch watch;
    ++impl_->stats.calls;
    const ControlCommand command = impl_->compute(state, reference, now);
    impl_->history.add(command);
    impl_->last = command;
    impl_->stats.compute_time.add(watch.elapsed_seconds());
    return command;
}

void MpcController::reset() {
    impl_->reset();
}

const ControllerStats& MpcController::stats() const {
    return impl_->stats;
}

const mpc::Qp& MpcController::last_problem() const {
    return impl_->problem;
}

const mpc::Solution& MpcController::last_solution() const {
    return impl_->solution;
}

const mpc::Plan& MpcController::plan() const {
    return impl_->plan;
}

}  // namespace apex::control
