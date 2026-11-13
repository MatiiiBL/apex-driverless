#include "common.hpp"

#include <algorithm>
#include <cmath>

#include "apex/core/angles.hpp"

namespace apex::control {

bool usable_inputs(const StateEstimate& state, const Trajectory& reference) {
    return state.valid && !reference.empty() && std::isfinite(state.stamp) &&
           std::isfinite(state.pose.x) && std::isfinite(state.pose.y) &&
           std::isfinite(state.pose.yaw) && std::isfinite(state.vx) && std::isfinite(state.vy) &&
           std::isfinite(state.yaw_rate);
}

double CallClock::step(Time now) {
    constexpr double kMaxGap = 0.2;  // s
    double dt = kNominalPeriod;
    if (last_.has_value()) {
        const double elapsed = now - *last_;
        if (elapsed > 0.0 && elapsed <= kMaxGap) dt = elapsed;
    }
    if (std::isfinite(now)) last_ = now;
    return dt;
}

Pose2 predict_pose(const StateEstimate& state, double horizon) {
    const double turn = state.yaw_rate * horizon;
    // Displacement in the start frame: the body velocity rotated through half the turn (exact
    // chord direction of the arc) and scaled by the chord-to-arc ratio.
    const double half = 0.5 * turn;
    const double ratio = std::abs(half) > 1e-9 ? std::sin(half) / half : 1.0;
    const Vec2 body{state.vx * horizon * ratio, state.vy * horizon * ratio};
    const Vec2 rotated{std::cos(half) * body.x() - std::sin(half) * body.y(),
                       std::sin(half) * body.x() + std::cos(half) * body.y()};
    const Vec2 position = state.pose.to_parent(rotated);
    return {position.x(), position.y(), wrap_angle(state.pose.yaw + turn)};
}

double stanley_law(double heading_error, double lateral, double speed, double gain,
                   double softening_speed) {
    return heading_error + std::atan2(-gain * lateral, softening_speed + std::max(speed, 0.0));
}

ControlCommand limit_command(double steer, double accel,
                             const std::optional<ControlCommand>& previous, double dt, double speed,
                             const NominalModel& model, Time now) {
    const VehicleParams& p = model.params();
    const double held = previous.has_value() ? previous->steer : 0.0;
    if (!std::isfinite(steer)) steer = held;
    if (!std::isfinite(accel)) accel = -kSafeDecel;
    if (previous.has_value()) {
        const double step = p.max_steer_rate * dt;
        steer = std::clamp(steer, held - step, held + step);
    }
    ControlCommand command;
    command.stamp = now;
    command.steer = std::clamp(steer, -p.max_steer, p.max_steer);
    const double top = std::isfinite(speed) ? model.max_drive_accel(speed) : 0.0;
    command.accel = std::clamp(accel, -model.max_brake_decel(), top);
    return command;
}

}  // namespace apex::control
