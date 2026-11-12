#pragma once

#include <memory>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/timing.hpp"

namespace apex::perception {

// Tunables of the cone detector and fusion. The perception module owns the fields and their
// defaults (docs/perception.md); every field has a default that works with the default rig.
struct PerceptionConfig {};

struct PerceptionStats {
    LatencyStats lidar_time;  // wall-clock time of on_lidar
    std::size_t scans = 0;
    std::size_t cones_reported = 0;
    std::size_t colored_by_camera = 0;  // reported cones that got a color from a camera frame
};

// LiDAR cone detection fused with the camera's colors.
//
// on_lidar finds cones in the point cloud (ground removal, clustering, shape checks), estimates
// each center and its covariance, then colors each cone from the buffered camera frame closest
// in time, compensating the car's rotation between the two stamps with yaw_rate. Cones the camera
// did not see come out Unknown. Positions are in the body frame at the scan's stamp.
class ConePerception {
public:
    ConePerception(const PerceptionConfig& config, const SensorRig& rig);
    ~ConePerception();
    ConePerception(ConePerception&&) noexcept;
    ConePerception& operator=(ConePerception&&) noexcept;

    void on_camera(const CameraFrame& frame);
    [[nodiscard]] ConeObservations on_lidar(const LidarScan& scan, double yaw_rate);
    [[nodiscard]] const PerceptionStats& stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex::perception
