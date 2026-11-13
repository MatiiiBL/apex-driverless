#pragma once

#include <optional>
#include <span>
#include <string>

#include "apex/core/types.hpp"
#include "apex/sim/track_rules.hpp"

// The track analysis on a plain sampled centerline, so the generator can check a candidate
// layout with the same code before paying for track_from_centerline.
namespace apex::sim::detail {

struct ProfileSample {
    double s = 0.0;  // arc length from the first sample (m)
    Vec2 position = Vec2::Zero();
    double curvature = 0.0;   // 1/m, positive turning left
    double half_width = 0.0;  // centerline to the farther cone line (m)
};

// `samples` cover a closed centerline of total length `length`, in increasing s from 0.
[[nodiscard]] TrackStats analyze_profile(std::span<const ProfileSample> samples, double length,
                                         const TrackRules& rules);

// The first rule `stats` break, or nothing. `margin` (m) tightens the radius and clearance limits
// for checks on a candidate that will still be resampled.
[[nodiscard]] std::optional<std::string> violated_rule(const TrackStats& stats,
                                                       const TrackRules& rules,
                                                       double margin = 0.0);

}  // namespace apex::sim::detail
