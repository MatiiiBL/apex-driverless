#include "apex/control/speed_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace apex::control {

SpeedController::SpeedController(const SpeedLoopConfig& config, const NominalModel& model)
    : config_(config), model_(model) {
    if (!(config.kp >= 0.0) || !(config.ki >= 0.0) || !(config.integral_limit >= 0.0) ||
        !(config.preview_time >= 0.0) || !(config.standstill_speed >= 0.0) ||
        !(config.hold_brake >= 0.0)) {
        throw std::invalid_argument("SpeedLoopConfig: gains, limits and times must be >= 0");
    }
}

double SpeedController::update(double target_speed, double target_accel, double speed, double dt) {
    // At rest with nothing asking to move: hold the car with the brake. A profile that starts
    // from rest has a zero speed but a positive acceleration there, and must not be held.
    if (target_speed < 0.05 && target_accel <= 0.0 && speed < config_.standstill_speed) {
        integral_ = 0.0;
        return -config_.hold_brake;
    }
    const double lo = -model_.max_brake_decel();
    const double hi = model_.max_drive_accel(speed);
    const double error = target_speed - speed;
    const double feed_forward = target_accel + model_.resistance(std::max(speed, 0.0));
    const double proportional = config_.kp * error;
    const double integral = std::clamp(integral_ + config_.ki * error * dt, -config_.integral_limit,
                                       config_.integral_limit);
    const double trial = feed_forward + proportional + integral;
    const bool winding_up = (trial > hi && error > 0.0) || (trial < lo && error < 0.0);
    if (!winding_up) integral_ = integral;
    return std::clamp(feed_forward + proportional + integral_, lo, hi);
}

}  // namespace apex::control
