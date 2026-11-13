#include "apex/control/mpc_plan.hpp"

#include <algorithm>
#include <cmath>

#include "apex/control/reference.hpp"
#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

namespace apex::control::mpc {

namespace {

PlanStage blend(const PlanStage& a, const PlanStage& b, double w) {
    auto lerp = [w](double x, double y) { return x + w * (y - x); };
    PlanStage out;
    out.time = lerp(a.time, b.time);
    out.pose = {lerp(a.pose.x, b.pose.x), lerp(a.pose.y, b.pose.y),
                wrap_angle(a.pose.yaw + w * angle_diff(b.pose.yaw, a.pose.yaw))};
    out.vx = lerp(a.vx, b.vx);
    out.vy = lerp(a.vy, b.vy);
    out.yaw_rate = lerp(a.yaw_rate, b.yaw_rate);
    out.steer = lerp(a.steer, b.steer);
    out.steer_rate = lerp(a.steer_rate, b.steer_rate);
    out.accel = lerp(a.accel, b.accel);
    out.slack = lerp(a.slack, b.slack);
    return out;
}

// The station of p on the reference near `hint`, continuing `previous` without wrapping.
double locate_near(const Trajectory& reference, const Vec2& p, double previous, double hint,
                   double window, double& lateral) {
    const Trajectory::Projection projection = reference.project(p, reference.wrap_s(hint), window);
    const PathCoordinates coords = extend_projection(reference, p, projection);
    lateral = coords.lateral;
    if (!reference.closed()) return coords.s;
    return previous + reference.delta_s(reference.wrap_s(previous), coords.s);
}

}  // namespace

PlanStage Plan::at(Time t) const {
    if (stages_.size() == 1 || t <= stages_.front().time) return stages_.front();
    const PlanStage& last = stages_.back();
    if (t >= last.time) {
        const PlanStage& before = stages_[stages_.size() - 2];
        const double span = last.time - before.time;
        const double w = span > 0.0 ? 1.0 + (t - last.time) / span : 1.0;
        PlanStage out = blend(before, last, w);  // linear continuation of the last segment
        out.vx = last.vx;
        out.vy = last.vy;
        out.yaw_rate = last.yaw_rate;
        out.steer = last.steer;
        out.steer_rate = 0.0;
        out.accel = last.accel;
        out.slack = last.slack;
        return out;
    }
    const auto next =
        std::upper_bound(stages_.begin(), stages_.end(), t,
                         [](Time value, const PlanStage& s) { return value < s.time; });
    const PlanStage& b = *next;
    const PlanStage& a = *(next - 1);
    return blend(a, b, (t - a.time) / (b.time - a.time));
}

void guess_from_plan(const Plan& plan, const Trajectory& reference, const State& x0, double s0,
                     Time t0, double dt, std::vector<StagePoint>& guess) {
    double s = s0;
    for (std::size_t k = 0; k < guess.size(); ++k) {
        StagePoint& point = guess[k];
        const PlanStage stage = plan.at(t0 + static_cast<double>(k) * dt);
        point.u << stage.steer_rate, stage.accel, stage.slack;
        if (k == 0) {
            point.s = s0;
            point.x = x0;
        } else {
            const double hint = s + std::max(stage.vx, 0.0) * dt;
            double lateral = 0.0;
            s = locate_near(reference, stage.pose.position(), s, hint, 2.0 + stage.vx * dt,
                            lateral);
            point.s = s;
            point.x << lateral, 0.0, stage.vx, stage.vy, stage.yaw_rate, stage.steer;
        }
        point.reference = sample_reference(reference, point.s);
        if (k > 0) point.x(kHeading) = angle_diff(stage.pose.yaw, point.reference.yaw);
    }
}

void guess_from_reference(const NominalModel& model, const Trajectory& reference, const State& x0,
                          double s0, double dt, std::vector<StagePoint>& guess) {
    constexpr double kErrorTimeConstant = 0.5;  // s, decay of the initial errors
    constexpr double kSpeedGain = 2.0;          // 1/s, approach to the profile
    constexpr double kComfortAccel = 6.0;       // m/s^2
    const double decay = std::exp(-dt / kErrorTimeConstant);
    const double wheelbase = model.params().wheelbase();
    double s = s0;
    double v = std::max(x0(kVx), 0.0);
    double lateral = x0(kLateral);
    double heading = x0(kHeading);
    for (std::size_t k = 0; k < guess.size(); ++k) {
        StagePoint& point = guess[k];
        point.s = s;
        point.reference = sample_reference(reference, s);
        const TrajectoryPoint& ref = point.reference;
        if (k == 0) {
            point.x = x0;
        } else {
            point.x << lateral, heading, v, 0.0, v * ref.curvature,
                std::atan(wheelbase * ref.curvature);
        }
        point.u << 0.0, ref.accel + model.resistance(v), 0.0;
        const double change =
            std::clamp(ref.accel + kSpeedGain * (ref.speed - v), -kComfortAccel, kComfortAccel);
        s += v * dt;
        v = std::max(v + change * dt, 0.0);
        lateral *= decay;
        heading *= decay;
    }
}

void plan_from_solution(const Trajectory& reference, double s0, const std::vector<State>& states,
                        const std::vector<Input>& inputs, Time t0, double dt, Plan& plan) {
    std::vector<PlanStage>& stages = plan.stages();
    stages.resize(states.size());
    double s = s0;
    for (std::size_t k = 0; k < states.size(); ++k) {
        const State& x = states[k];
        const TrajectoryPoint ref = sample_reference(reference, s);
        const Vec2 position = ref.position + x(kLateral) * left_normal(heading_vector(ref.yaw));
        PlanStage& stage = stages[k];
        stage.time = t0 + static_cast<double>(k) * dt;
        stage.pose = {position.x(), position.y(), wrap_angle(ref.yaw + x(kHeading))};
        stage.vx = x(kVx);
        stage.vy = x(kVy);
        stage.yaw_rate = x(kYawRate);
        stage.steer = x(kSteer);
        // The last stage has no dynamics after it: it holds the inputs before it.
        const Input& u = k + 1 < states.size() ? inputs[k] : inputs[k > 0 ? k - 1 : 0];
        stage.steer_rate = k + 1 < states.size() ? u(kSteerRate) : 0.0;
        stage.accel = u(kAccel);
        stage.slack = inputs[k](kSlack);
        if (k + 1 < states.size()) {
            const double speed_here = path_speed(to_array(x), ref.curvature);
            const double speed_next = path_speed(to_array(states[k + 1]), ref.curvature);
            s += 0.5 * dt * (speed_here + speed_next);
        }
    }
}

}  // namespace apex::control::mpc
