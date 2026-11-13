#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/timing.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/estimation/association.hpp"
#include "apex/estimation/slam_config.hpp"
#include "apex/estimation/velocity_filter.hpp"

namespace apex::estimation {

// Tunables of the velocity filter and the SLAM back end. The estimation module owns the fields
// and their defaults (docs/estimation.md).
struct EstimatorConfig {
    VelocityFilterConfig velocity;
    SlamConfig slam;
    AssociationConfig association;
};

// {"velocity": {...}, "slam": {...}, "association": {...}}; missing keys keep their defaults.
void to_json(nlohmann::json& j, const EstimatorConfig& config);
void from_json(const nlohmann::json& j, EstimatorConfig& config);

struct EstimatorStats {
    LatencyStats predict_time;  // wall-clock time per IMU-driven prediction
    LatencyStats update_time;   // wall-clock time per on_cones
    std::size_t observations = 0;
    std::size_t associated = 0;  // matched to an existing landmark
    std::size_t new_landmarks = 0;
    std::size_t rejected = 0;  // gated out or pending confirmation

    std::size_t tentative_matches = 0;  // of the rejected: matched to an unconfirmed landmark
    std::size_t tentatives_started = 0;
    std::size_t tentatives_dropped = 0;  // unconfirmed landmarks that timed out
    std::size_t late_dropped = 0;        // observations older than the odometry history
    std::size_t invalid_inputs = 0;      // non-finite or out-of-order messages ignored
    std::size_t ambiguous_scans = 0;     // scans where observations competed for landmarks
    std::size_t search_exhausted = 0;    // scans where the JCBB node budget ran out
    std::size_t wheel_outliers = 0;      // wheel samples down-weighted as slipping
    std::size_t failed_updates = 0;      // updates refused for a singular innovation covariance
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

    // For the most recent on_cones call, element i is the id of the confirmed landmark that
    // observation i was associated to, or -1 if it was rejected, gated out, initialized a new
    // landmark, or fed a tentative one. Evaluation only: it reflects decisions, never drives them.
    [[nodiscard]] const std::vector<std::int32_t>& last_associations() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex::estimation
