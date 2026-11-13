#pragma once

#include <cstddef>
#include <limits>

#include <Eigen/Core>
#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

namespace apex::estimation {

// Noise model of the velocity filter. Process noises are continuous-time densities, so they do
// not depend on the IMU rate; measurement noises are per sample. docs/estimation.md derives the
// defaults.
struct VelocityFilterConfig {
    // Accelerometer noise plus its unmodelled bias and the CoG/IMU offset: (m/s^2)/sqrt(Hz).
    double accel_noise_density = 0.15;
    // Yaw acceleration driving the yaw-rate random walk: (rad/s^2)/sqrt(Hz).
    double yaw_accel_density = 3.0;
    double gyro_bias_walk = 2e-4;  // gyro bias random walk, (rad/s)/sqrt(s)
    double gyro_noise = 0.003;     // rad/s per sample

    double wheel_noise = 0.05;  // m/s per sample, either axle
    // Predicted slip ratio = axle force coefficient Fx / Fz divided by this stiffness.
    double wheel_slip_stiffness = 20.0;
    double wheel_slip_uncertainty = 0.5;  // std of the slip prediction, relative to it
    // Normalized wheel innovation beyond which a sample is down-weighted (Huber).
    double wheel_robust_threshold = 2.5;

    double steering_noise = 0.003;  // rad per sample
    // The single-track lateral pseudo-measurements: noise floor, growth with speed, and the
    // relative uncertainty of the predicted slip angle.
    double lateral_noise = 0.02;            // m/s
    double lateral_noise_per_speed = 0.01;  // (m/s) per (m/s)
    double lateral_slip_uncertainty = 0.3;
    // Axles using more of their friction than this skip the lateral update: near the peak of
    // the tire curve the slip angle no longer follows from the force.
    double max_tire_utilization = 0.9;
    // The errors of the slip predictions (wheel slip ratio, tire slip angles) persist through a
    // manoeuvre: every sample within this time sees the same error, so it must not be averaged
    // away as if it were white noise.
    double slip_correlation_time = 0.3;     // s
    double yaw_accel_time_constant = 0.05;  // s, low-pass of the differentiated gyro

    // The car starts at rest, so the initial velocity is known well; the bias is not.
    double initial_speed_std = 0.05;      // m/s
    double initial_yaw_rate_std = 0.01;   // rad/s
    double initial_gyro_bias_std = 0.01;  // rad/s
};

void to_json(nlohmann::json& j, const VelocityFilterConfig& config);
void from_json(const nlohmann::json& j, VelocityFilterConfig& config);

// An extended Kalman filter over x = (vx, vy, r, b): body velocities at the CoG, yaw rate and
// gyro bias.
//
// Prediction takes the accelerometer as input, from the planar rigid-body kinematics of a
// rotating frame:
//   vx' = ax + r vy,   vy' = ay - r vx,   r' = w_r,   b' = w_b
// with r and b random walks. The gyro then measures r + b. Wheel speeds measure each axle's
// longitudinal velocity with a slip ratio predicted from the axle's force coefficient, and
// samples that disagree beyond `wheel_robust_threshold` sigma are down-weighted with Huber's
// weight (Huber, "Robust estimation of a location parameter", Ann. Math. Stat. 35, 1964),
// which keeps the filter consistent through wheel spin and lock-up. Each steering sample adds
// two single-track pseudo-measurements, one per axle: the velocity of the axle in the wheel's
// frame points along the predicted slip angle (lateral_model.hpp). Together with the gyro they
// make vy and the bias observable while driving; at rest they pin r to zero, which calibrates
// the bias before the start.
//
// The slip predictions are only as good as the nominal tires, and their error is not white: it
// lasts as long as the manoeuvre. Fusing 100 samples a second as if each had an independent
// error would average it away and make the filter overconfident exactly where the model is
// worst, so the model part of each variance is scaled by the samples per slip_correlation_time
// (the effective sample count of a correlated error: a cheaper, conservative alternative to
// augmenting the state with it, as Bar-Shalom, Li and Kirubarajan, "Estimation with
// Applications to Tracking and Navigation", Wiley, 2001, chapter 8, do for autocorrelated
// measurement noise). Every update uses the Joseph form.
class VelocityFilter {
public:
    using Vec4 = Eigen::Vector4d;
    using Mat4 = Eigen::Matrix4d;

    // Throws std::invalid_argument for a non-positive noise or time constant.
    VelocityFilter(const VelocityFilterConfig& config, const VehicleParams& nominal);

    // Each returns false, changing nothing, for a non-finite sample or (IMU) one not newer than
    // the last.
    bool predict(const ImuSample& sample);
    bool update_wheels(const WheelSpeeds& sample);
    bool update_steering(const SteeringSample& sample);

    [[nodiscard]] bool initialized() const { return initialized_; }
    [[nodiscard]] Time stamp() const { return stamp_; }  // of the latest IMU sample
    [[nodiscard]] const Vec4& state() const { return x_; }
    [[nodiscard]] const Mat4& covariance() const { return P_; }
    [[nodiscard]] double vx() const { return x_[0]; }
    [[nodiscard]] double vy() const { return x_[1]; }
    [[nodiscard]] double yaw_rate() const { return x_[2]; }
    [[nodiscard]] double gyro_bias() const { return x_[3]; }
    [[nodiscard]] Mat3 velocity_covariance() const { return P_.topLeftCorner<3, 3>(); }

    // Wheel samples down-weighted as slipping, and lateral updates skipped near the tire limit.
    [[nodiscard]] std::size_t wheel_outliers() const { return wheel_outliers_; }
    [[nodiscard]] std::size_t lateral_skips() const { return lateral_skips_; }

private:
    using Row4 = Eigen::RowVector4d;

    // Scalar EKF update with innovation `residual`, Joseph form. Returns the normalized
    // innovation residual / sqrt(S) before the update.
    double update(const Row4& h, double residual, double variance);
    // Huber-weighted update: variance inflated where the normalized innovation exceeds c.
    void robust_update(const Row4& h, double residual, double variance);
    void update_lateral(double model_scale);
    [[nodiscard]] double expected_slip_ratio(bool front) const;
    // How much to inflate a slip-model variance for a sample at `stamp`: the number of samples
    // in one slip_correlation_time at the interval since `last` (which it then advances), so
    // that the samples sharing one model error together count as one.
    [[nodiscard]] double model_error_scale(Time stamp, Time& last) const;

    VelocityFilterConfig config_;
    VehicleParams nominal_;
    Vec4 x_ = Vec4::Zero();
    Mat4 P_ = Mat4::Zero();
    Time stamp_ = 0.0;
    bool initialized_ = false;
    double ax_ = 0.0;  // latest IMU specific force, for the slip models
    double ay_ = 0.0;
    double last_gyro_ = 0.0;
    double yaw_accel_ = 0.0;  // low-passed derivative of the gyro
    double steer_ = 0.0;      // latest measured steering angle
    // Previous wheel and steering stamps, NaN before the first.
    Time last_wheel_stamp_ = std::numeric_limits<double>::quiet_NaN();
    Time last_steering_stamp_ = std::numeric_limits<double>::quiet_NaN();
    std::size_t wheel_outliers_ = 0;
    std::size_t lateral_skips_ = 0;
};

}  // namespace apex::estimation
