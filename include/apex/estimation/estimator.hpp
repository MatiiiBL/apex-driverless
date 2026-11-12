#pragma once

#include <memory>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/timing.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

namespace apex::estimation {

// Tunables of the velocity filter and the SLAM back end. The estimation module owns the fields
// and their defaults (docs/estimation.md).
struct EstimatorConfig {};

struct EstimatorStats {
    LatencyStats predict_time;  // wall-clock time per IMU-driven prediction
    LatencyStats update_time;   // wall-clock time per on_cones
    std::size_t observations = 0;
    std::size_t associated = 0;  // matched to an existing landmark
    std::size_t new_landmarks = 0;
    std::size_t rejected = 0;  // gated out or pending confirmation
};

// Velocity estimation and cone SLAM behind one interface.
//
// A velocity filter fuses the IMU, wheel speeds and steering angle into (vx, vy, yaw rate). An
// EKF-SLAM over the car's pose and the cone landmarks predicts with that velocity and corrects
// with cone observations, associating each to a landmark (or a new one) with gating that
// tolerates the dense, look-alike cone field of a track. Cone observations arrive late (their
// stamp is the LiDAR scan's, older than the latest IMU sample); the estimator compensates.
//
// The map frame is the world frame: the car is told its start pose, as an FS car defines its
// map frame at the staging position.
class StateEstimator {
public:
    StateEstimator(const EstimatorConfig& config, const VehicleParams& nominal,
                   const SensorRig& rig, const Pose2& start);
    ~StateEstimator();
    StateEstimator(StateEstimator&&) noexcept;
    StateEstimator& operator=(StateEstimator&&) noexcept;

    void on_imu(const ImuSample& sample);
    void on_wheels(const WheelSpeeds& sample);
    void on_steering(const SteeringSample& sample);
    void on_cones(const ConeObservations& observations);

    // The state at the stamp of the latest message processed.
    [[nodiscard]] StateEstimate estimate() const;
    [[nodiscard]] const ConeMap& map() const;

    // On (the default): unmatched, confirmed cones become new landmarks. Off: the map is final
    // and the estimator localizes against it.
    void set_mapping(bool enabled);
    [[nodiscard]] bool mapping() const;

    [[nodiscard]] const EstimatorStats& stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex::estimation
