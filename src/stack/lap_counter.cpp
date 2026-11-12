#include "apex/stack/lap_counter.hpp"

#include <stdexcept>

#include <nlohmann/json.hpp>

#include "apex/core/geometry.hpp"
#include "apex/core/json.hpp"

namespace apex::stack {

void to_json(nlohmann::json& j, const LapCounterConfig& c) {
    j = {{"search_radius", c.search_radius},
         {"line_margin", c.line_margin},
         {"min_lap_distance", c.min_lap_distance},
         {"fallback_distance", c.fallback_distance},
         {"fallback_half_width", c.fallback_half_width}};
}

void from_json(const nlohmann::json& j, LapCounterConfig& c) {
    reject_unknown_keys(j,
                        {"search_radius", "line_margin", "min_lap_distance", "fallback_distance",
                         "fallback_half_width"},
                        "lap_counter");
    read_optional(j, "search_radius", c.search_radius);
    read_optional(j, "line_margin", c.line_margin);
    read_optional(j, "min_lap_distance", c.min_lap_distance);
    read_optional(j, "fallback_distance", c.fallback_distance);
    read_optional(j, "fallback_half_width", c.fallback_half_width);
}

std::optional<TimingLine> timing_line_from_map(const ConeMap& map, const Pose2& start,
                                               double search_radius) {
    Vec2 left_sum = Vec2::Zero();
    Vec2 right_sum = Vec2::Zero();
    double left_count = 0.0;
    double right_count = 0.0;
    for (const MapCone& cone : map.cones) {
        if (cone.color != ConeColor::BigOrange) continue;
        if ((cone.position - start.position()).norm() > search_radius) continue;
        if (start.to_local(cone.position).y() > 0.0) {
            left_sum += cone.position;
            left_count += 1.0;
        } else {
            right_sum += cone.position;
            right_count += 1.0;
        }
    }
    if (left_count == 0.0 || right_count == 0.0) return std::nullopt;
    return TimingLine{left_sum / left_count, right_sum / right_count, true};
}

LapCounter::LapCounter(const Pose2& start, const LapCounterConfig& config)
    : config_(config), start_(start) {
    if (!(config_.search_radius > 0.0) || !(config_.line_margin >= 0.0) ||
        !(config_.min_lap_distance > 0.0) || !(config_.fallback_distance > 0.0) ||
        !(config_.fallback_half_width > 0.0)) {
        throw std::invalid_argument("lap_counter: distances must be positive");
    }
    const Vec2 ahead = start_.to_parent({config_.fallback_distance, 0.0});
    const Vec2 side = left_normal(heading_vector(start_.yaw)) * config_.fallback_half_width;
    line_ = {ahead + side, ahead - side, false};
}

void LapCounter::update_line(const ConeMap& map) {
    if (have_revision_ && map.revision == map_revision_) return;
    have_revision_ = true;
    map_revision_ = map.revision;
    if (const auto mapped = timing_line_from_map(map, start_, config_.search_radius)) {
        line_ = *mapped;
    }
}

bool LapCounter::update(const Vec2& position) {
    if (!previous_) {
        previous_ = position;
        return false;
    }
    const Vec2 from = *previous_;
    previous_ = position;
    distance_ += (position - from).norm();

    const Vec2 span = line_.left - line_.right;
    const double length = span.norm();
    if (!(length > 1e-6)) return false;
    const Vec2 along = span / length;
    // Driving forward over the line, its left end is on the car's left: the forward normal is
    // the line direction turned 90 degrees clockwise.
    const Vec2 forward{along.y(), -along.x()};
    const Vec2 middle = 0.5 * (line_.left + line_.right);
    const bool forward_crossing =
        (from - middle).dot(forward) < 0.0 && (position - middle).dot(forward) >= 0.0;
    if (!forward_crossing) return false;
    const Vec2 a = line_.right - config_.line_margin * along;
    const Vec2 b = line_.left + config_.line_margin * along;
    if (!segment_intersection(from, position, a, b)) return false;
    if (crossings_ > 0 && distance_ - distance_at_crossing_ < config_.min_lap_distance) {
        return false;
    }
    ++crossings_;
    distance_at_crossing_ = distance_;
    return true;
}

}  // namespace apex::stack
