#include "apex/runner/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <vector>

#include <nlohmann/json.hpp>

namespace apex::runner {

void to_json(nlohmann::json& j, const ErrorStats& s) {
    j = {{"samples", s.samples}, {"rms", s.rms}, {"max", s.max}};
}

void ErrorAccumulator::add(double error) {
    ++samples_;
    sum_squares_ += error * error;
    max_ = std::max(max_, std::abs(error));
}

ErrorStats ErrorAccumulator::stats() const {
    ErrorStats s;
    s.samples = samples_;
    s.max = max_;
    s.rms = samples_ == 0 ? 0.0 : std::sqrt(sum_squares_ / static_cast<double>(samples_));
    return s;
}

void to_json(nlohmann::json& j, const MapError& e) {
    j = {{"time", e.time},
         {"finalized", e.finalized},
         {"cones", e.cones},
         {"landmarks", e.landmarks},
         {"matched", e.matched},
         {"false_landmarks", e.false_landmarks},
         {"duplicates", e.duplicates},
         {"coverage", e.coverage},
         {"color_accuracy", e.color_accuracy},
         {"position", e.position}};
}

MapError evaluate_map(const ConeMap& map, const Track& track, Time time, bool finalized) {
    MapError e;
    e.time = time;
    e.finalized = finalized;
    e.cones = track.cones.size();
    e.landmarks = map.cones.size();
    std::vector<std::size_t> per_cone(track.cones.size(), 0);
    ErrorAccumulator position;
    std::size_t right_color = 0;
    for (const MapCone& landmark : map.cones) {
        if (landmark.truth_id < 0 ||
            static_cast<std::size_t>(landmark.truth_id) >= track.cones.size()) {
            ++e.false_landmarks;
            continue;
        }
        const Cone& truth = track.cones[static_cast<std::size_t>(landmark.truth_id)];
        ++e.matched;
        if (per_cone[static_cast<std::size_t>(truth.id)]++ > 0) ++e.duplicates;
        position.add((landmark.position - truth.position).norm());
        if (landmark.color == truth.color) ++right_color;
    }
    const auto covered = std::count_if(per_cone.begin(), per_cone.end(),
                                       [](std::size_t count) { return count > 0; });
    e.coverage = e.cones == 0 ? 0.0 : static_cast<double>(covered) / static_cast<double>(e.cones);
    e.color_accuracy =
        e.matched == 0 ? 0.0 : static_cast<double>(right_color) / static_cast<double>(e.matched);
    e.position = position.stats();
    return e;
}

double off_track_distance(const Track& track, double s, double lateral) {
    const TrajectoryPoint p = track.centerline.at(s);
    return std::max({lateral - p.width_left, -lateral - p.width_right, 0.0});
}

std::optional<std::string> DnfMonitor::update(Time now, double speed, double off_track) {
    if (off_track > rules_.max_off_track) {
        return std::format("off the track by {:.1f} m", off_track);
    }
    if (now > rules_.time_limit) return std::format("time limit of {:.0f} s", rules_.time_limit);
    if (speed < rules_.stall_speed) {
        if (!slow_) {
            slow_ = true;
            slow_since_ = now;
        }
        if (now - slow_since_ >= rules_.stall_time) {
            return std::format("stalled for {:.0f} s", rules_.stall_time);
        }
    } else {
        slow_ = false;
    }
    return std::nullopt;
}

}  // namespace apex::runner
