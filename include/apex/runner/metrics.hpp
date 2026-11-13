#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/track.hpp"
#include "apex/runner/config.hpp"

// The evaluation's measures (docs/design.md, section 8), computed from the ground truth the
// runner holds. None of this ever reaches the car's software.
namespace apex::runner {

// RMS and maximum of a non-negative error.
struct ErrorStats {
    std::size_t samples = 0;
    double rms = 0.0;
    double max = 0.0;
};

void to_json(nlohmann::json& j, const ErrorStats& stats);

class ErrorAccumulator {
public:
    void add(double error);
    [[nodiscard]] ErrorStats stats() const;

private:
    std::size_t samples_ = 0;
    double sum_squares_ = 0.0;
    double max_ = 0.0;
};

// The car's map against the true cones, by truth id.
struct MapError {
    Time time = 0.0;         // when the map was evaluated
    bool finalized = false;  // taken when lap 1 closed (else at the end of the run)
    std::size_t cones = 0;   // true cones on the track
    std::size_t landmarks = 0;
    std::size_t matched = 0;          // landmarks with a true cone behind them
    std::size_t false_landmarks = 0;  // landmarks of clutter or false positives
    std::size_t duplicates = 0;       // landmarks beyond the first for the same true cone
    double coverage = 0.0;            // share of the true cones with a landmark
    double color_accuracy = 0.0;      // share of matched landmarks with the true color
    ErrorStats position;              // landmark to its true cone, over matched landmarks
};

void to_json(nlohmann::json& j, const MapError& error);

[[nodiscard]] MapError evaluate_map(const ConeMap& map, const Track& track, Time time,
                                    bool finalized);

// How far the point with centerline coordinates (s, lateral) is outside the cone lines (0 when
// on the track).
[[nodiscard]] double off_track_distance(const Track& track, double s, double lateral);

// The DNF rules on the true car: stalled, too far off the track, out of time.
class DnfMonitor {
public:
    explicit DnfMonitor(const DnfRules& rules) : rules_(rules) {}

    // Called every tick with the true speed and off-track distance. Returns why the run is
    // over, or nothing.
    [[nodiscard]] std::optional<std::string> update(Time now, double speed, double off_track);

private:
    DnfRules rules_;
    Time slow_since_ = 0.0;
    bool slow_ = false;
};

}  // namespace apex::runner
