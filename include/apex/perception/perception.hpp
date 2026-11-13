#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/timing.hpp"
#include "apex/perception/camera_fusion.hpp"
#include "apex/perception/clustering.hpp"
#include "apex/perception/cone_model.hpp"
#include "apex/perception/ground.hpp"

namespace apex::perception {

// Tunables of the cone detector and fusion. The perception module owns the fields and their
// defaults (docs/perception.md); every field has a default that works with the default rig.
struct PerceptionConfig {
    GroundConfig ground;
    ClusterConfig clustering;
    ConeCheckConfig cone_check;
    CenterFitConfig center_fit;
    CovarianceModel covariance;
    CameraFusionConfig camera;
};

// JSON keys are the field names; missing keys keep their defaults, unknown keys throw.
void to_json(nlohmann::json& j, const PerceptionConfig& config);
void from_json(const nlohmann::json& j, PerceptionConfig& config);

struct PerceptionStats {
    LatencyStats lidar_time;  // wall-clock time of on_lidar
    std::size_t scans = 0;
    std::size_t cones_reported = 0;
    std::size_t colored_by_camera = 0;  // reported cones that got a color from a camera frame

    std::size_t points = 0;            // returns received
    std::size_t invalid_points = 0;    // non-finite returns, ignored
    std::size_t obstacle_points = 0;   // returns above the ground, within clustering range
    std::size_t ground_fallbacks = 0;  // scans whose plane fit failed (previous plane used)
    std::size_t clusters = 0;
    // Clusters by the outcome of the cone checks, indexed by ConeCheck.
    std::array<std::size_t, kConeCheckCount> cone_checks{};
    std::size_t non_finite_estimates = 0;    // accepted clusters dropped for a non-finite estimate
    std::size_t camera_frames = 0;           // frames received
    std::size_t camera_frames_dropped = 0;   // older than the whole (full) buffer, or bad stamp
    std::size_t camera_frames_replaced = 0;  // a second frame with an already-buffered stamp
    std::size_t scans_without_camera = 0;    // no buffered frame within max_time_gap of the scan
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
