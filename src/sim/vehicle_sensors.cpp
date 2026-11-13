#include <algorithm>
#include <cmath>

#include "apex/sim/sensors.hpp"

// The proprioceptive sensors: IMU, wheel speeds and steering angle.
namespace apex::sim {

ImuBias draw_imu_bias(const ImuNoise& noise, Rng& rng) {
    ImuBias bias;
    bias.ax = rng.normal(0.0, noise.accel_bias);
    bias.ay = rng.normal(0.0, noise.accel_bias);
    bias.yaw_rate = rng.normal(0.0, noise.gyro_bias);
    return bias;
}

ImuSample simulate_imu(const ImuNoise& noise, const VehicleState& state, Time stamp, double period,
                       ImuBias& bias, Rng& rng) {
    const double root_period = std::sqrt(std::max(period, 0.0));
    bias.ax += rng.normal(0.0, noise.accel_bias_walk * root_period);
    bias.ay += rng.normal(0.0, noise.accel_bias_walk * root_period);
    bias.yaw_rate += rng.normal(0.0, noise.gyro_bias_walk * root_period);
    ImuSample sample;
    sample.stamp = stamp;
    sample.ax = state.ax + bias.ax + rng.normal(0.0, noise.accel_stddev);
    sample.ay = state.ay + bias.ay + rng.normal(0.0, noise.accel_stddev);
    sample.yaw_rate = state.yaw_rate + bias.yaw_rate + rng.normal(0.0, noise.gyro_stddev);
    return sample;
}

WheelSpeeds simulate_wheel_speeds(const VehicleParams& vehicle, const WheelNoise& noise,
                                  const VehicleState& state, const TireForces& tires, Time stamp,
                                  Rng& rng) {
    auto slip = [&noise](double fx, double fz) {
        return fz > 0.0 ? fx / (noise.slip_stiffness * fz) : 0.0;
    };
    const double front_ground = state.vx * std::cos(state.steer) +
                                (state.vy + vehicle.lf * state.yaw_rate) * std::sin(state.steer);
    const double front = front_ground * (1.0 + slip(tires.fx_front, tires.fz_front));
    const double rear = state.vx * (1.0 + slip(tires.fx_rear, tires.fz_rear));
    WheelSpeeds speeds;
    speeds.stamp = stamp;
    speeds.front = std::max(front + rng.normal(0.0, noise.stddev), 0.0);
    speeds.rear = std::max(rear + rng.normal(0.0, noise.stddev), 0.0);
    return speeds;
}

SteeringSample simulate_steering(const SteeringNoise& noise, const VehicleState& state, Time stamp,
                                 Rng& rng) {
    return {stamp, state.steer + rng.normal(0.0, noise.stddev)};
}

}  // namespace apex::sim
