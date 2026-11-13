#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <string>

#include "apex/sim/sensors.hpp"

namespace apex::sim {

namespace {

void check(double value, const char* name) {
    if (!(std::isfinite(value) && value >= 0.0)) {
        throw std::invalid_argument(std::string("SensorNoise: ") + name +
                                    " must be finite and non-negative");
    }
}

}  // namespace

// The structured bindings name every field of each struct, so adding a field breaks the build
// here until it is either scaled or deliberately left alone.
SensorNoise scale_noise(const SensorNoise& noise, double factor) {
    if (!(std::isfinite(factor) && factor >= 0.0)) {
        throw std::invalid_argument("scale_noise: factor must be finite and non-negative");
    }
    SensorNoise scaled = noise;
    auto& [accel, gyro, accel_bias, gyro_bias, accel_walk, gyro_walk] = scaled.imu;
    auto& [wheel_stddev, slip_stiffness] = scaled.wheels;
    auto& [steering_stddev] = scaled.steering;
    (void)slip_stiffness;  // a tire property, not noise
    for (double* value : {&accel, &gyro, &accel_bias, &gyro_bias, &accel_walk, &gyro_walk,
                          &wheel_stddev, &steering_stddev}) {
        *value *= factor;
    }
    return scaled;
}

void validate_noise(const SensorNoise& noise) {
    const ImuNoise& imu = noise.imu;
    check(imu.accel_stddev, "imu.accel_stddev");
    check(imu.gyro_stddev, "imu.gyro_stddev");
    check(imu.accel_bias, "imu.accel_bias");
    check(imu.gyro_bias, "imu.gyro_bias");
    check(imu.accel_bias_walk, "imu.accel_bias_walk");
    check(imu.gyro_bias_walk, "imu.gyro_bias_walk");
    check(noise.wheels.stddev, "wheels.stddev");
    if (!(std::isfinite(noise.wheels.slip_stiffness) && noise.wheels.slip_stiffness > 0.0)) {
        throw std::invalid_argument("SensorNoise: wheels.slip_stiffness must be positive");
    }
    check(noise.steering.stddev, "steering.stddev");
}

}  // namespace apex::sim
