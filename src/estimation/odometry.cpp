#include "apex/estimation/odometry.hpp"

#include <algorithm>
#include <cmath>

#include "apex/core/angles.hpp"

namespace apex::estimation {

RelativeMotion compose(const RelativeMotion& first, const RelativeMotion& second) {
    const double c = std::cos(first.delta.yaw);
    const double s = std::sin(first.delta.yaw);
    const double x2 = second.delta.x;
    const double y2 = second.delta.y;
    Mat3 a = Mat3::Identity();
    a(0, 2) = -s * x2 - c * y2;
    a(1, 2) = c * x2 - s * y2;
    Mat3 b = Mat3::Identity();
    b(0, 0) = c;
    b(0, 1) = -s;
    b(1, 0) = s;
    b(1, 1) = c;
    RelativeMotion result;
    result.delta = first.delta.compose(second.delta);
    result.covariance =
        a * first.covariance * a.transpose() + b * second.covariance * b.transpose();
    result.duration = first.duration + second.duration;
    return result;
}

Pose2 integrate_twist(double vx, double vy, double yaw_rate, double dt) {
    const double theta = yaw_rate * dt;
    double sin_ratio = 1.0;  // sin(theta) / theta
    double cos_ratio = 0.0;  // (1 - cos(theta)) / theta
    if (std::abs(theta) > 1e-6) {
        sin_ratio = std::sin(theta) / theta;
        cos_ratio = (1.0 - std::cos(theta)) / theta;
    } else {
        const double theta2 = theta * theta;
        sin_ratio = 1.0 - theta2 / 6.0;
        cos_ratio = theta / 2.0 - theta * theta2 / 24.0;
    }
    return {(sin_ratio * vx - cos_ratio * vy) * dt, (cos_ratio * vx + sin_ratio * vy) * dt,
            wrap_angle(theta)};
}

RelativeMotion OdometryIncrement::motion(Time until) const {
    const double dt = std::clamp(until, start, end) - start;
    return {integrate_twist(vx, vy, yaw_rate, dt), noise_density * dt, dt};
}

void OdometryBuffer::reset(Time time) {
    increments_.clear();
    pending_ = {};
    start_ = time;
    end_ = time;
}

void OdometryBuffer::push(OdometryIncrement increment) {
    increment.start = end_;
    if (!(increment.end > increment.start)) return;
    end_ = increment.end;
    pending_ = compose(pending_, increment.motion(increment.end));
    increments_.push_back(increment);
}

RelativeMotion OdometryBuffer::take_until(Time t) {
    RelativeMotion taken;
    while (!increments_.empty() && increments_.front().end <= t) {
        taken = compose(taken, increments_.front().motion(increments_.front().end));
        increments_.pop_front();
    }
    if (!increments_.empty() && increments_.front().start < t) {
        taken = compose(taken, increments_.front().motion(t));
        increments_.front().start = t;
    }
    start_ = increments_.empty() ? end_ : increments_.front().start;
    pending_ = {};
    for (const OdometryIncrement& increment : increments_) {
        pending_ = compose(pending_, increment.motion(increment.end));
    }
    return taken;
}

}  // namespace apex::estimation
