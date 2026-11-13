#include "apex/estimation/dead_reckoning.hpp"

namespace apex::estimation {

DeadReckoning::DeadReckoning(const VelocityFilterConfig& velocity, const SlamConfig& slam,
                             const VehicleParams& nominal)
    : slam_(slam), velocity_(velocity, nominal) {}

Mat3 DeadReckoning::twist_noise_density() const {
    Mat3 q = 2.0 * slam_.velocity_correlation_time * velocity_.velocity_covariance();
    const double p = slam_.position_noise_density;
    const double y = slam_.yaw_noise_density;
    q.diagonal() += Vec3(p * p, p * p, y * y);
    return q;
}

void DeadReckoning::start(Time t) {
    if (started_) return;
    odometry_.reset(t);
    started_ = true;
}

void DeadReckoning::push(Time end, double vx, double vy, double yaw_rate) {
    OdometryIncrement increment;
    increment.end = end;
    increment.vx = vx;
    increment.vy = vy;
    increment.yaw_rate = yaw_rate;
    increment.noise_density = twist_noise_density();
    odometry_.push(increment);
}

bool DeadReckoning::on_imu(const ImuSample& sample) {
    const Vec3 before(velocity_.vx(), velocity_.vy(), velocity_.yaw_rate());
    if (!velocity_.predict(sample)) return false;
    if (!started_) {
        start(sample.stamp);
        return true;
    }
    // Trapezoidal twist over the interval: the velocity before and after the sample.
    const Vec3 after(velocity_.vx(), velocity_.vy(), velocity_.yaw_rate());
    const Vec3 twist = 0.5 * (before + after);
    push(sample.stamp, twist.x(), twist.y(), twist.z());
    return true;
}

bool DeadReckoning::on_wheels(const WheelSpeeds& sample) {
    return velocity_.update_wheels(sample);
}

bool DeadReckoning::on_steering(const SteeringSample& sample) {
    return velocity_.update_steering(sample);
}

void DeadReckoning::extrapolate_to(Time t) {
    if (t > odometry_.end()) push(t, velocity_.vx(), velocity_.vy(), velocity_.yaw_rate());
}

}  // namespace apex::estimation
