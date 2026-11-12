#pragma once

#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/types.hpp"
#include "apex/stack/driver.hpp"

namespace apex::stack {

struct MissionConfig {
    int laps = 10;  // laps to drive, the first one exploring; at least 1
    // The car waits at rest this long before driving off: the go signal, and time for the
    // velocity filter to settle its IMU bias estimate at standstill.
    double start_delay = 0.5;  // s
    // Simulated time charged for computing the raceline once lap 1 closes. The optimization
    // itself runs at once (its result is deterministic); charging a fixed delay rather than its
    // wall-clock time keeps runs reproducible. The car follows the exploration planner meanwhile.
    double raceline_delay = 1.0;    // s
    double standstill_speed = 0.1;  // m/s: below it, the car has stopped
    double max_no_path_time = 3.0;  // s without a fresh exploration path before failing
    double max_lost_time = 0.5;     // s with an unusable estimate before failing
    double max_position_std = 2.0;  // m: a wider estimate is unusable (localization lost)
};

void to_json(nlohmann::json& j, const MissionConfig& config);
void from_json(const nlohmann::json& j, MissionConfig& config);

// What the mission logic needs to know at each control step.
struct MissionInputs {
    Time now = 0.0;
    bool estimate_finite = true;
    bool estimate_usable = false;  // valid and within max_position_std
    double speed = 0.0;            // estimated (m/s)
    int laps_completed = 0;        // by the stack's own lap counter
    // Stamp of the latest successful exploration plan; negative before the first one.
    Time last_path_time = -1.0;
};

// The mission state machine of docs/design.md, sections 1 and 5.6, without the modules: the
// stack feeds it its own view of the car and acts on the state it returns. Keeping the
// decisions here makes every transition testable in isolation.
//
// Staged       -> Exploring     at start_delay, once the estimate is usable
// Exploring    -> MapFinalized  when the stack reports a track model and raceline (lap 1 closed)
// MapFinalized -> Racing        raceline_delay later
// any driving  -> Stopping      once laps_completed reaches laps
// Stopping     -> Finished      at standstill
// any          -> Failed        non-finite estimate; estimate unusable for max_lost_time;
//                               no exploration path for max_no_path_time while following one
class Mission {
public:
    // Throws std::invalid_argument for laps < 1 or negative times.
    explicit Mission(const MissionConfig& config);

    // Advances the state machine and returns the current state.
    MissionState update(const MissionInputs& inputs);
    // Exploring -> MapFinalized; ignored in any other state.
    void map_finalized(Time now);
    // Any non-terminal state -> Failed.
    void fail(Time now, std::string reason);

    [[nodiscard]] MissionState state() const { return state_; }
    [[nodiscard]] Time state_since() const { return since_; }
    [[nodiscard]] const std::vector<MissionTransition>& transitions() const { return transitions_; }
    [[nodiscard]] const MissionConfig& config() const { return config_; }

private:
    void transition(MissionState to, Time now, std::string reason);
    // The exploration path is the reference in these states.
    [[nodiscard]] bool following_exploration() const;

    MissionConfig config_;
    MissionState state_ = MissionState::Staged;
    Time since_ = 0.0;
    Time lost_since_ = -1.0;  // negative while the estimate is usable
    std::vector<MissionTransition> transitions_;
};

}  // namespace apex::stack
