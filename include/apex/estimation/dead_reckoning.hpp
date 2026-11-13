#pragma once

#include "apex/core/messages.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/estimation/odometry.hpp"
#include "apex/estimation/slam_config.hpp"
#include "apex/estimation/velocity_filter.hpp"

namespace apex::estimation {

// The velocity filter and the odometry it integrates since the SLAM state's time: the part of
// the estimator that runs on every IMU, wheel and steering message. Also used on its own to
// record a run's odometry for the batch reference, so both see the same motion and noise.
class DeadReckoning {
public:
    DeadReckoning(const VelocityFilterConfig& velocity, const SlamConfig& slam,
                  const VehicleParams& nominal);

    // Each returns false for a sample the velocity filter rejected.
    bool on_imu(const ImuSample& sample);
    bool on_wheels(const WheelSpeeds& sample);
    bool on_steering(const SteeringSample& sample);

    // Sets the time base if no IMU sample has: the motion before it is zero (at rest).
    void start(Time t);
    [[nodiscard]] bool started() const { return started_; }

    // Extends the odometry to t at the current velocity when t is beyond its newest sample.
    void extrapolate_to(Time t);

    [[nodiscard]] const VelocityFilter& velocity() const { return velocity_; }
    [[nodiscard]] OdometryBuffer& odometry() { return odometry_; }
    [[nodiscard]] const OdometryBuffer& odometry() const { return odometry_; }

    // Spectral density of the pose error the current velocity estimate causes: its error as a
    // first-order Gauss-Markov process of correlation time tau, whose integral grows like white
    // noise of density 2 tau Sigma_v (Maybeck, "Stochastic Models, Estimation, and Control",
    // vol. 1, Academic Press, 1979, section 4.11), plus the configured floors.
    [[nodiscard]] Mat3 twist_noise_density() const;

private:
    void push(Time end, double vx, double vy, double yaw_rate);

    SlamConfig slam_;
    VelocityFilter velocity_;
    OdometryBuffer odometry_;
    bool started_ = false;
};

}  // namespace apex::estimation
