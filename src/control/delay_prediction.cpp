#include "delay_prediction.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include "apex/core/angles.hpp"

namespace apex::control {

void CommandHistory::add(const ControlCommand& command) {
    constexpr double kKeep = 0.5;  // s, far beyond any actuator delay
    if (!std::isfinite(command.stamp)) return;
    // A command older than the newest means time went backwards: what came after is void.
    while (!commands_.empty() && commands_.back().stamp > command.stamp) commands_.pop_back();
    commands_.push_back(command);
    // Keep the newest command before the cut too: it is still in force just after it.
    const Time cut = command.stamp - kKeep;
    while (commands_.size() > 1 && commands_[1].stamp <= cut) commands_.pop_front();
}

ControlCommand CommandHistory::at(Time t, const ControlCommand& fallback) const {
    for (auto it = commands_.rbegin(); it != commands_.rend(); ++it) {
        if (it->stamp <= t) return *it;
    }
    return fallback;
}

WorldState predict_through_delays(const NominalModel& model, const StateEstimate& state, Time until,
                                  const CommandHistory& history, const ActuatorDelays& delays,
                                  const ControlCommand& before) {
    // (x, y, yaw, vx, vy, yaw rate)
    using Vector = Eigen::Matrix<double, 6, 1>;
    Vector z;
    z << state.pose.x, state.pose.y, state.pose.yaw, state.vx, state.vy, state.yaw_rate;
    const double span = until - state.stamp;
    if (span > 0.0) {
        constexpr double kMaxStep = 0.005;  // s
        const double steps = std::ceil(span / kMaxStep);
        const double h = span / steps;
        for (double i = 0.0; i < steps; i += 1.0) {
            const Time t = state.stamp + i * h;
            const double steer = history.at(t - delays.steer, before).steer;
            const double steer_rate = (history.at(t + h - delays.steer, before).steer - steer) / h;
            const double accel = history.at(t - delays.drive, before).accel;
            auto derivative = [&](const Vector& v) {
                const BodyRates<double> rates =
                    model.body_rates(v(3), v(4), v(5), steer, steer_rate, accel);
                const double c = std::cos(v(2));
                const double s = std::sin(v(2));
                Vector d;
                d << v(3) * c - v(4) * s, v(3) * s + v(4) * c, v(5), rates.vx_dot, rates.vy_dot,
                    rates.yaw_accel;
                return d;
            };
            const Vector k1 = derivative(z);
            const Vector k2 = derivative(z + 0.5 * h * k1);
            const Vector k3 = derivative(z + 0.5 * h * k2);
            const Vector k4 = derivative(z + h * k3);
            z += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            if (z(3) <= 0.0) z.tail<3>().setZero();
        }
    }
    return {{z(0), z(1), wrap_angle(z(2))}, z(3), z(4), z(5)};
}

}  // namespace apex::control
