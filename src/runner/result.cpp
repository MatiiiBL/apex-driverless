#include "apex/runner/result.hpp"

#include <algorithm>
#include <numeric>
#include <string>

namespace apex::runner {

using nlohmann::json;

namespace {

json optional_json(const std::optional<double>& value) {
    return value ? json(*value) : json(nullptr);
}

}  // namespace

double RunResult::total_time() const {
    return std::accumulate(lap_times.begin(), lap_times.end(), 0.0);
}

double RunResult::score() const {
    return total_time() + penalty_seconds;
}

std::optional<double> RunResult::lap1_time() const {
    if (lap_times.empty()) return std::nullopt;
    return lap_times.front();
}

std::optional<double> RunResult::best_racing_lap() const {
    if (lap_times.size() < 2) return std::nullopt;
    return *std::min_element(lap_times.begin() + 1, lap_times.end());
}

std::optional<double> RunResult::mean_racing_lap() const {
    if (lap_times.size() < 2) return std::nullopt;
    return std::accumulate(lap_times.begin() + 1, lap_times.end(), 0.0) /
           static_cast<double>(lap_times.size() - 1);
}

void to_json(json& j, const RunResult& r) {
    json transitions = json::array();
    for (const stack::MissionTransition& t : r.transitions) {
        transitions.push_back({{"time", t.time},
                               {"from", stack::to_string(t.from)},
                               {"to", stack::to_string(t.to)},
                               {"reason", t.reason}});
    }
    json phases = json::object();
    for (std::size_t i = 0; i < r.phases.size(); ++i) {
        const PhaseMetrics& p = r.phases[i];
        if (p.duration <= 0.0) continue;
        phases[std::string(stack::to_string(static_cast<stack::MissionState>(i)))] = {
            {"duration", p.duration},
            {"position", p.position},
            {"yaw", p.yaw},
            {"lateral", p.lateral}};
    }
    j = {{"name", r.name},
         {"track", {{"name", r.track_name}, {"seed", r.track_seed}, {"length", r.track_length}}},
         {"seed", r.seed},
         {"laps_target", r.laps_target},
         {"completed", r.completed},
         {"stopped", r.stopped},
         {"dnf_reason", r.dnf_reason},
         {"error", r.error},
         {"final_state", stack::to_string(r.final_state)},
         {"transitions", std::move(transitions)},
         {"sim_time", r.sim_time},
         {"wall_time", r.wall_time},
         {"laps", r.lap_times},
         {"lap1", optional_json(r.lap1_time())},
         {"racing_best", optional_json(r.best_racing_lap())},
         {"racing_mean", optional_json(r.mean_racing_lap())},
         {"total_time", r.total_time()},
         {"penalties",
          {{"cones_hit", r.cones_hit},
           {"off_course", r.off_course},
           {"seconds", r.penalty_seconds}}},
         {"score", r.score()},
         {"map", r.map},
         {"phases", std::move(phases)},
         {"association",
          {{"observations", r.association.observations},
           {"matched", r.association.matched},
           {"correct", r.association.correct},
           {"matchable", r.association.matchable},
           {"precision", r.association.precision()},
           {"recall", r.association.recall()}}},
         {"max_lateral_true", r.max_lateral_true},
         {"compute", r.compute}};
}

json deterministic_json(const RunResult& result) {
    json j = result;
    j.erase("wall_time");
    j.erase("compute");
    return j;
}

}  // namespace apex::runner
