#pragma once

#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/planning/planning.hpp"
#include "apex/stack/driver.hpp"
#include "apex/stack/lap_counter.hpp"
#include "apex/stack/stack_config.hpp"

namespace apex::stack {

// The car's software assembled (docs/design.md, section 5.6): a ConePerception, a
// StateEstimator, the exploration planner, the track model and raceline, and a Controller,
// behind the mission state machine.
//
// Message routing: IMU, wheel speeds and steering go to the estimator; camera frames to
// perception's buffer; each LiDAR scan through perception into the estimator's cone update,
// after which the exploration planner replans on the new map (while the exploration path is the
// reference). control() reads the estimate, counts laps on the timing line mapped from the big
// orange cones, steps the mission and asks the controller for a command on the current
// reference: the exploration path, then the raceline, then a braking reference after the last
// lap.
//
// Under the oracles (Driver::on_oracle_*) the stack bypasses perception (observations come from
// the runner) or estimation (the state comes from the runner, and the map from a
// KnownPoseMapper fed with the true poses).
class Stack final : public Driver {
public:
    // `start` is the staging pose, which defines the map frame (docs/design.md, section 3).
    // Throws std::invalid_argument for an invalid config.
    Stack(const StackConfig& config, const VehicleParams& nominal, const SensorRig& rig,
          const Pose2& start, const Oracles& oracles = {});
    ~Stack() override;
    Stack(const Stack&) = delete;
    Stack& operator=(const Stack&) = delete;

    void on_sensor(const SensorMessage& message) override;
    void on_oracle_observations(const ConeObservations& observations) override;
    void on_oracle_state(const StateEstimate& truth) override;
    [[nodiscard]] ControlCommand control(Time now) override;

    [[nodiscard]] MissionState mission_state() const override;
    [[nodiscard]] const std::vector<MissionTransition>& transitions() const override;
    [[nodiscard]] StateEstimate estimate() const override;
    [[nodiscard]] const ConeMap* map() const override;
    [[nodiscard]] const Trajectory* reference() const override;
    [[nodiscard]] AssociationCounts association_counts() const override;
    [[nodiscard]] nlohmann::json compute_summary() const override;

    // ---- Telemetry ----
    [[nodiscard]] int laps_completed() const;
    [[nodiscard]] const TimingLine& timing_line() const;
    // Set once lap 1 has closed and the map was turned into a track model and raceline.
    [[nodiscard]] const std::optional<planning::TrackModel>& track_model() const;
    [[nodiscard]] const std::optional<planning::Raceline>& raceline() const;
    // The poses recorded on lap 1 (from the first timing-line crossing on).
    [[nodiscard]] std::span<const Pose2> driven_path() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex::stack
