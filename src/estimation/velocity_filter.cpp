#include "apex/estimation/velocity_filter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "apex/core/angles.hpp"
#include "apex/core/json.hpp"
#include "apex/estimation/lateral_model.hpp"

namespace apex::estimation {

// clang-format off
#define APEX_VELOCITY_FIELDS(X) \
    X(accel_noise_density) \
    X(yaw_accel_density) \
    X(gyro_bias_walk) \
    X(gyro_noise) \
    X(wheel_noise) \
    X(wheel_slip_stiffness) \
    X(wheel_slip_uncertainty) \
    X(wheel_robust_threshold) \
    X(steering_noise) \
    X(lateral_noise) \
    X(lateral_noise_per_speed) \
    X(lateral_slip_uncertainty) \
    X(max_tire_utilization) \
    X(slip_correlation_time) \
    X(yaw_accel_time_constant) \
    X(initial_speed_std) \
    X(initial_yaw_rate_std) \
    X(initial_gyro_bias_std)
// clang-format on

void to_json(nlohmann::json& j, const VelocityFilterConfig& config) {
    j = nlohmann::json::object();
#define APEX_WRITE(name) j[#name] = config.name;
    APEX_VELOCITY_FIELDS(APEX_WRITE)
#undef APEX_WRITE
}

void from_json(const nlohmann::json& j, VelocityFilterConfig& config) {
#define APEX_NAME(name) #name,
    reject_unknown_keys(j, {APEX_VELOCITY_FIELDS(APEX_NAME)}, "velocity filter");
#undef APEX_NAME
#define APEX_READ(name) read_optional(j, #name, config.name);
    APEX_VELOCITY_FIELDS(APEX_READ)
#undef APEX_READ
}

namespace {

void require_positive(double value, const char* name) {
    if (!(value > 0.0) || !std::isfinite(value)) {
        throw std::invalid_argument(std::string("velocity filter: ") + name + " must be positive");
    }
}

}  // namespace

VelocityFilter::VelocityFilter(const VelocityFilterConfig& config, const VehicleParams& nominal)
    : config_(config), nominal_(nominal) {
#define APEX_CHECK(name) require_positive(config.name, #name);
    APEX_VELOCITY_FIELDS(APEX_CHECK)
#undef APEX_CHECK
    if (!(nominal.wheelbase() > 0.0) || !(nominal.mass > 0.0)) {
        throw std::invalid_argument("velocity filter: invalid nominal vehicle");
    }
    P_.diagonal() << config.initial_speed_std * config.initial_speed_std,
        config.initial_speed_std * config.initial_speed_std,
        config.initial_yaw_rate_std * config.initial_yaw_rate_std,
        config.initial_gyro_bias_std * config.initial_gyro_bias_std;
}

#undef APEX_VELOCITY_FIELDS

double VelocityFilter::update(const Row4& h, double residual, double variance) {
    const Vec4 ph = P_ * h.transpose();
    const double s = h.dot(ph) + variance;
    const Vec4 k = ph / s;
    x_ += k * residual;
    const Mat4 a = Mat4::Identity() - k * h;
    P_ = a * P_ * a.transpose() + variance * k * k.transpose();
    return residual / std::sqrt(s);
}

void VelocityFilter::robust_update(const Row4& h, double residual, double variance) {
    const double s = h * P_ * h.transpose() + variance;
    const double normalized = std::abs(residual) / std::sqrt(s);
    const double c = config_.wheel_robust_threshold;
    if (normalized > c) {
        // Huber's weight c / |e| on the residual is an R inflated so that S grows by |e| / c.
        ++wheel_outliers_;
        variance = s * normalized / c - (s - variance);
    }
    update(h, residual, variance);
}

bool VelocityFilter::predict(const ImuSample& sample) {
    if (!std::isfinite(sample.stamp) || !std::isfinite(sample.ax) || !std::isfinite(sample.ay) ||
        !std::isfinite(sample.yaw_rate)) {
        return false;
    }
    if (initialized_) {
        const double dt = sample.stamp - stamp_;
        if (!(dt > 0.0)) return false;
        const double vx = x_[0];
        const double vy = x_[1];
        const double r = x_[2];
        // Euler step of the rotating-frame kinematics; at 200 Hz its error is far below the
        // accelerometer noise.
        x_[0] = vx + dt * (sample.ax + r * vy);
        x_[1] = vy + dt * (sample.ay - r * vx);
        Mat4 f = Mat4::Identity();
        f(0, 1) = dt * r;
        f(0, 2) = dt * vy;
        f(1, 0) = -dt * r;
        f(1, 2) = -dt * vx;
        const double qa = config_.accel_noise_density * config_.accel_noise_density * dt;
        const double qr = config_.yaw_accel_density * config_.yaw_accel_density * dt;
        const double qb = config_.gyro_bias_walk * config_.gyro_bias_walk * dt;
        P_ = f * P_ * f.transpose();
        P_.diagonal() += Vec4(qa, qa, qr, qb);
        const double raw_accel = (sample.yaw_rate - last_gyro_) / dt;
        yaw_accel_ += dt / (config_.yaw_accel_time_constant + dt) * (raw_accel - yaw_accel_);
    }
    stamp_ = sample.stamp;
    initialized_ = true;
    ax_ = sample.ax;
    ay_ = sample.ay;
    last_gyro_ = sample.yaw_rate;
    update(Row4(0.0, 0.0, 1.0, 1.0), sample.yaw_rate - (x_[2] + x_[3]),
           config_.gyro_noise * config_.gyro_noise);
    return true;
}

// Slip ratio (wheel speed / ground speed - 1) an axle runs at for the force it transmits: the
// total tire force is the measured specific force plus the drag and rolling resistance it
// overcomes, split between the axles as the drivetrain or the brakes split it.
double VelocityFilter::expected_slip_ratio(bool front) const {
    const double speed = std::max(x_[0], 0.0);
    const double resistance =
        nominal_.drag(speed) + nominal_.rolling_resistance * nominal_.mass * kGravity;
    const double force = nominal_.mass * ax_ + (speed > 0.5 ? resistance : 0.0);
    const double split = force >= 0.0 ? nominal_.drive_split_front : nominal_.brake_split_front;
    const AxleLoads loads = axle_normal_loads(nominal_, speed, ax_);
    const double axle_force = force * (front ? split : 1.0 - split);
    const double coefficient = axle_force / (front ? loads.front : loads.rear);
    return coefficient / config_.wheel_slip_stiffness;
}

double VelocityFilter::model_error_scale(Time stamp, Time& last) const {
    const double correlation_time = config_.slip_correlation_time;
    // The first sample, or one out of order, counts as one correlation time after the last.
    const double interval =
        stamp > last ? std::min(stamp - last, correlation_time) : correlation_time;
    last = stamp;
    return correlation_time / interval;
}

bool VelocityFilter::update_wheels(const WheelSpeeds& sample) {
    if (!std::isfinite(sample.stamp) || !std::isfinite(sample.front) ||
        !std::isfinite(sample.rear)) {
        return false;
    }
    const double noise = config_.wheel_noise * config_.wheel_noise;
    const double correlation = model_error_scale(sample.stamp, last_wheel_stamp_);
    {
        // Rear axle: the mean of the two wheels' longitudinal speeds is vx.
        const double slip = expected_slip_ratio(false);
        const double sigma_slip = config_.wheel_slip_uncertainty * slip * x_[0];
        const double predicted = x_[0] * (1.0 + slip);
        robust_update(Row4(1.0 + slip, 0.0, 0.0, 0.0), sample.rear - predicted,
                      noise + correlation * sigma_slip * sigma_slip);
    }
    {
        // Front axle: the velocity of the axle center along the steered wheels.
        const double c = std::cos(steer_);
        const double s = std::sin(steer_);
        const double slip = expected_slip_ratio(true);
        const double lateral = x_[1] + nominal_.lf * x_[2];
        const double along = x_[0] * c + lateral * s;
        const double sigma_slip = config_.wheel_slip_uncertainty * slip * along;
        const double g = 1.0 + slip;
        robust_update(Row4(g * c, g * s, g * nominal_.lf * s, 0.0), sample.front - g * along,
                      noise + correlation * sigma_slip * sigma_slip);
    }
    return true;
}

bool VelocityFilter::update_steering(const SteeringSample& sample) {
    if (!std::isfinite(sample.stamp) || !std::isfinite(sample.angle)) return false;
    steer_ = sample.angle;
    update_lateral(model_error_scale(sample.stamp, last_steering_stamp_));
    return true;
}

// Two pseudo-measurements, h = 0, from the single-track model: at each axle, the lateral
// velocity in the wheel frame equals the longitudinal one times tan(slip angle), where the slip
// angle follows from the force the axle carries (lateral_model.hpp). Their noise grows with
// speed and with the predicted slip, whose prediction is only as good as the nominal tires and
// the steady-state force split; that part is correlated in time and scaled by `model_scale`.
void VelocityFilter::update_lateral(double model_scale) {
    const double vx = x_[0];
    const double speed = std::abs(vx);
    const AxleSlip slip = predict_axle_slip(nominal_, speed, ax_, ay_, yaw_accel_);
    const double base =
        config_.lateral_noise + config_.lateral_noise_per_speed * speed;  // m/s, std
    const double lf = nominal_.lf;
    const double lr = nominal_.lr;
    if (slip.front.utilization <= config_.max_tire_utilization) {
        const double c = std::cos(steer_);
        const double s = std::sin(steer_);
        const double t = std::tan(slip.front.alpha);
        const double lateral = x_[1] + lf * x_[2];
        const double v_lat = -vx * s + lateral * c;
        const double v_long = vx * c + lateral * s;
        const double h = v_lat - v_long * t;
        const Row4 jac(-s - c * t, c - s * t, lf * (c - s * t), 0.0);
        const double d_steer = -v_long - t * v_lat;  // dh / d(steer)
        const double model = config_.lateral_slip_uncertainty * speed * std::abs(t);
        const double steer = d_steer * config_.steering_noise;
        update(jac, -h, base * base + model_scale * model * model + steer * steer);
    } else {
        ++lateral_skips_;
    }
    if (slip.rear.utilization <= config_.max_tire_utilization) {
        const double t = std::tan(slip.rear.alpha);
        const double h = x_[1] - lr * x_[2] - vx * t;
        const double model = config_.lateral_slip_uncertainty * speed * std::abs(t);
        update(Row4(-t, 1.0, -lr, 0.0), -h, base * base + model_scale * model * model);
    } else {
        ++lateral_skips_;
    }
}

}  // namespace apex::estimation
