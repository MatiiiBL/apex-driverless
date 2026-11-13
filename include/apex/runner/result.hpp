#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/runner/metrics.hpp"
#include "apex/stack/driver.hpp"

namespace apex::runner {

// Localization and tracking while the mission was in one state.
struct PhaseMetrics {
    double duration = 0.0;  // s of simulated time spent in the state
    ErrorStats position;    // |estimated - true| CoG position (m)
    ErrorStats yaw;         // |estimated - true| heading (rad)
    // Lateral distance of the estimated position from the reference the controller tracks (m).
    ErrorStats lateral;
};

// The outcome of one closed-loop run (docs/design.md, sections 5.6 and 8). Lap times and
// penalties come from the race officials (RaceMonitor), the error metrics from the truth.
struct RunResult {
    std::string name;
    std::string track_name;
    std::uint64_t track_seed = 0;
    double track_length = 0.0;
    std::uint64_t seed = 0;
    int laps_target = 0;

    bool completed = false;  // every lap timed with no DNF
    bool stopped = false;    // came to rest after the last lap
    std::string dnf_reason;  // empty unless the run did not finish
    std::string error;       // set when the run could not be set up (bad config, ...)
    stack::MissionState final_state = stack::MissionState::Staged;
    std::vector<stack::MissionTransition> transitions;

    double sim_time = 0.0;   // s simulated
    double wall_time = 0.0;  // s of wall-clock time for the whole run

    std::vector<double> lap_times;  // s, from the race officials
    std::size_t cones_hit = 0;
    std::size_t off_course = 0;
    double penalty_seconds = 0.0;

    MapError map;
    std::array<PhaseMetrics, stack::kMissionStateCount> phases{};
    stack::AssociationCounts association;
    double max_lateral_true = 0.0;  // largest true distance from the centerline (m)
    nlohmann::json compute = nlohmann::json::object();  // per-module compute time

    [[nodiscard]] double total_time() const;  // sum of lap times
    [[nodiscard]] double score() const;       // total time plus penalties
    [[nodiscard]] std::optional<double> lap1_time() const;
    // Over laps 2..N; nothing if there are none.
    [[nodiscard]] std::optional<double> best_racing_lap() const;
    [[nodiscard]] std::optional<double> mean_racing_lap() const;
};

void to_json(nlohmann::json& j, const RunResult& result);

// The result without anything that depends on wall-clock time (compute times, wall_time): two
// runs of the same config must agree on it exactly.
[[nodiscard]] nlohmann::json deterministic_json(const RunResult& result);

}  // namespace apex::runner
