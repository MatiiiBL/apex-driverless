#pragma once

#include <cstdint>
#include <optional>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/types.hpp"

namespace apex::stack {

struct LapCounterConfig {
    // Big orange cones farther than this from the staging position are not the timekeeping
    // line's (FS stages the car 6 m behind it).
    double search_radius = 15.0;  // m
    // The mapped line is extended this far past each end, so localization error near a cone
    // cannot make the car pass beside it.
    double line_margin = 1.5;  // m
    // Distance driven between two counted crossings: rejects a second crossing when the line
    // moves (the map refining it) just as the car passes.
    double min_lap_distance = 30.0;  // m
    // Without mapped big orange cones, the line is assumed square to the staging heading this
    // far ahead (the FS staging distance), this long on each side of the car's path.
    double fallback_distance = 6.0;    // m
    double fallback_half_width = 3.0;  // m
};

void to_json(nlohmann::json& j, const LapCounterConfig& config);
void from_json(const nlohmann::json& j, LapCounterConfig& config);

// The timekeeping line as the car knows it: from its right end to its left end, seen in the
// driving direction.
struct TimingLine {
    Vec2 left = Vec2::Zero();
    Vec2 right = Vec2::Zero();
    bool mapped = false;  // from big orange landmarks, not the fallback
};

// The line through the big orange landmarks near the start: the mean of those on the left of
// the staging pose and the mean of those on its right. Nothing unless both sides have one.
[[nodiscard]] std::optional<TimingLine> timing_line_from_map(const ConeMap& map, const Pose2& start,
                                                             double search_radius);

// Counts laps from the car's own position estimate crossing the timekeeping line it mapped
// (docs/design.md, section 5.6): it never sees the race officials' timing. The first forward
// crossing starts lap 1; each later one completes a lap.
class LapCounter {
public:
    // Throws std::invalid_argument for non-positive distances.
    LapCounter(const Pose2& start, const LapCounterConfig& config);

    // Re-derives the line when the map has changed and still shows it; otherwise keeps the
    // previous line (or the fallback, before any).
    void update_line(const ConeMap& map);
    // Feeds the latest position estimate; true when this step crossed the line forward and
    // counted.
    bool update(const Vec2& position);

    [[nodiscard]] int crossings() const { return crossings_; }
    [[nodiscard]] int laps_completed() const { return crossings_ > 0 ? crossings_ - 1 : 0; }
    [[nodiscard]] const TimingLine& line() const { return line_; }
    [[nodiscard]] double distance_driven() const { return distance_; }

private:
    LapCounterConfig config_;
    Pose2 start_;
    TimingLine line_;
    std::uint64_t map_revision_ = 0;
    bool have_revision_ = false;
    std::optional<Vec2> previous_;
    double distance_ = 0.0;
    double distance_at_crossing_ = 0.0;
    int crossings_ = 0;
};

}  // namespace apex::stack
