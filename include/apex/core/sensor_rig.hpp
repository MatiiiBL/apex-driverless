#pragma once

#include <vector>

#include "apex/core/types.hpp"

namespace apex {

// Sensor geometry and rates: what the simulator simulates and what the car's software assumes.
// Noise models are the simulator's business (sim/sensors.hpp); the software has its own
// nominal noise figures in its configs.
//
// Every rate must divide the 1 kHz physics rate, so each sensor fires on exact ticks.

// A multi-channel spinning LiDAR, mounted low on the front wing as FS teams do, so its
// near-horizontal channels see cones far ahead. Sensor axes are parallel to the body axes.
struct LidarGeometry {
    Vec3 mount{1.20, 0.0, 0.20};  // in the body frame (m)
    double rate_hz = 10.0;
    // Channel elevations (rad, positive up). Default: 64 channels evenly spaced over
    // [-12, +4] degrees, 0.25 degree apart.
    std::vector<double> elevations = default_elevations();
    double azimuth_min = -1.5707963267948966;  // rad, from the sensor's +x axis
    double azimuth_max = 1.5707963267948966;
    double azimuth_step = 0.0034906585039886592;  // rad (0.2 degrees)
    double min_range = 0.3;                       // m
    double max_range = 50.0;                      // m

    [[nodiscard]] static std::vector<double> default_elevations();
};

// A camera with a cone detector. The simulator produces detections, not pixels: bearing,
// ground-plane range, color and confidence of each cone the detector would report.
struct CameraGeometry {
    Vec3 mount{0.0, 0.0, 1.10};  // in the body frame (m), on the main hoop
    double rate_hz = 20.0;
    double horizontal_fov = 1.7453292519943295;  // rad (100 degrees), centered on +x
    double max_range = 30.0;                     // m
};

struct SensorRig {
    LidarGeometry lidar;
    CameraGeometry camera;
    double imu_rate_hz = 200.0;       // IMU at the CoG, axes aligned with the body
    double wheel_rate_hz = 100.0;     // wheel speed sensors
    double steering_rate_hz = 100.0;  // steering angle sensor
};

}  // namespace apex
