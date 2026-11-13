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

void check_probability(double value, const char* name) {
    if (!(value >= 0.0 && value <= 1.0)) {
        throw std::invalid_argument(std::string("SensorNoise: ") + name + " must be in [0, 1]");
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
    auto& [lidar_range, dropout] = scaled.lidar;
    auto& [min_range, detection, half_range, falloff, bearing, camera_range, camera_range_r2,
           confusion_near, confusion_far, confidence, false_positives] = scaled.camera;
    auto& [accel, gyro, accel_bias, gyro_bias, accel_walk, gyro_walk] = scaled.imu;
    auto& [wheel_stddev, slip_stiffness] = scaled.wheels;
    auto& [steering_stddev] = scaled.steering;
    // Probabilities, rates and physical properties stay as they are.
    (void)dropout;
    (void)min_range;
    (void)detection;
    (void)half_range;
    (void)falloff;
    (void)confusion_near;
    (void)confusion_far;
    (void)false_positives;
    (void)slip_stiffness;
    for (double* value :
         {&lidar_range, &bearing, &camera_range, &camera_range_r2, &confidence, &accel, &gyro,
          &accel_bias, &gyro_bias, &accel_walk, &gyro_walk, &wheel_stddev, &steering_stddev}) {
        *value *= factor;
    }
    return scaled;
}

void validate_noise(const SensorNoise& noise) {
    check(noise.lidar.range_stddev, "lidar.range_stddev");
    check_probability(noise.lidar.dropout, "lidar.dropout");
    const CameraNoise& camera = noise.camera;
    check(camera.min_range, "camera.min_range");
    check_probability(camera.detection_probability, "camera.detection_probability");
    check(camera.detection_half_range, "camera.detection_half_range");
    if (!(std::isfinite(camera.detection_falloff) && camera.detection_falloff > 0.0)) {
        throw std::invalid_argument("SensorNoise: camera.detection_falloff must be positive");
    }
    check(camera.bearing_stddev, "camera.bearing_stddev");
    check(camera.range_stddev, "camera.range_stddev");
    check(camera.range_stddev_quadratic, "camera.range_stddev_quadratic");
    check_probability(camera.confusion_near, "camera.confusion_near");
    check_probability(camera.confusion_far, "camera.confusion_far");
    check(camera.confidence_stddev, "camera.confidence_stddev");
    check(camera.false_positives, "camera.false_positives");
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
