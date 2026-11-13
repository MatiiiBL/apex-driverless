#pragma once

#include <algorithm>
#include <cmath>
#include <deque>

#include "apex/control/vehicle_model.hpp"
#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/messages.hpp"
#include "apex/core/vehicle.hpp"

// The car the control tests drive until the simulator's vehicle model is available: the
// dynamic single-track model of vehicle_model.hpp with the *true* parameters, behind steering
// and drive actuators with dead time, first-order lag and limits, integrated with RK4 at 1 ms.
// Tests give the controller a different, nominal copy of the parameters when they want model
// mismatch. The estimate it reports is the truth.
namespace apex::control::testing {

class TestPlant {
public:
    static constexpr double kTick = 1e-3;  // s

    TestPlant(const VehicleParams& params, const Pose2& start, double speed = 0.0)
        : params_(params), model_(params) {
        state_.pose = start;
        state_.vx = speed;
    }

    // Queues a command; it reaches the actuators after their dead times.
    void command(const ControlCommand& command) { pending_.push_back({time_, command}); }

    // Advances by whole ticks.
    void advance(double seconds) {
        const auto ticks = static_cast<long>(std::lround(seconds / kTick));
        for (long i = 0; i < ticks; ++i) tick();
    }

    [[nodiscard]] Time time() const { return time_; }
    [[nodiscard]] const VehicleState& truth() const { return state_; }

    [[nodiscard]] StateEstimate estimate() const {
        StateEstimate e;
        e.stamp = time_;
        e.pose = state_.pose;
        e.vx = state_.vx;
        e.vy = state_.vy;
        e.yaw_rate = state_.yaw_rate;
        e.valid = true;
        return e;
    }

private:
    struct Queued {
        Time sent = 0.0;
        ControlCommand command;
    };

    void tick() {
        // The newest command older than each actuator's dead time is its target.
        while (pending_.size() > 1 && pending_[1].sent + params_.steer_dead_time <= time_ &&
               pending_[1].sent + params_.drive_dead_time <= time_) {
            pending_.pop_front();
        }
        double steer_target = 0.0;
        double accel_target = 0.0;
        for (const Queued& q : pending_) {
            if (q.sent + params_.steer_dead_time <= time_) steer_target = q.command.steer;
            if (q.sent + params_.drive_dead_time <= time_) accel_target = q.command.accel;
        }
        steer_target = std::clamp(steer_target, -params_.max_steer, params_.max_steer);
        const double steer_rate =
            std::clamp((steer_target - state_.steer) / params_.steer_time_constant,
                       -params_.max_steer_rate, params_.max_steer_rate);
        const double speed = std::max(state_.vx, 0.0);
        const double drive_limit =
            std::min(params_.max_drive_force, params_.max_power / std::max(speed, 1.0));
        const double force_target =
            std::clamp(params_.mass * accel_target, -params_.max_brake_force, drive_limit);
        state_.force += (force_target - state_.force) * (kTick / params_.drive_time_constant);

        const double accel = state_.force / params_.mass;
        auto derivative = [&](const Body& b) {
            const BodyRates<double> rates =
                model_.body_rates(b.vx, b.vy, b.r, state_.steer, steer_rate, accel);
            const double c = std::cos(b.yaw);
            const double s = std::sin(b.yaw);
            return Body{b.vx * c - b.vy * s, b.vx * s + b.vy * c, b.r,
                        rates.vx_dot,        rates.vy_dot,        rates.yaw_accel};
        };
        const Body b0{state_.pose.x, state_.pose.y, state_.pose.yaw,
                      state_.vx,     state_.vy,     state_.yaw_rate};
        const Body k1 = derivative(b0);
        const Body k2 = derivative(b0 + k1 * (0.5 * kTick));
        const Body k3 = derivative(b0 + k2 * (0.5 * kTick));
        const Body k4 = derivative(b0 + k3 * kTick);
        const Body b1 = b0 + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (kTick / 6.0);
        state_.ax = k1.vx - b0.vy * b0.r;
        state_.ay = k1.vy + b0.vx * b0.r;
        state_.pose = {b1.x, b1.y, wrap_angle(b1.yaw)};
        state_.vx = b1.vx;
        state_.vy = b1.vy;
        state_.yaw_rate = b1.r;
        // The car does not reverse: braking at a standstill holds it.
        if (state_.vx <= 0.0) {
            state_.vx = 0.0;
            state_.vy = 0.0;
            state_.yaw_rate = 0.0;
        }
        state_.steer =
            std::clamp(state_.steer + steer_rate * kTick, -params_.max_steer, params_.max_steer);
        time_ += kTick;
    }

    struct Body {
        double x, y, yaw, vx, vy, r;
        Body operator+(const Body& o) const {
            return {x + o.x, y + o.y, yaw + o.yaw, vx + o.vx, vy + o.vy, r + o.r};
        }
        Body operator*(double k) const { return {x * k, y * k, yaw * k, vx * k, vy * k, r * k}; }
    };

    VehicleParams params_;
    NominalModel model_;
    VehicleState state_;
    Time time_ = 0.0;
    std::deque<Queued> pending_;
};

}  // namespace apex::control::testing
