#include "apex/sim/track_rules.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <vector>

#include "apex/core/angles.hpp"

#include "track_profile.hpp"

namespace apex::sim {

namespace detail {

namespace {

constexpr double kHairpinMinAngle = deg_to_rad(130.0);
constexpr double kHairpinMaxRadius = 10.0;
constexpr double kSweeperMinAngle = deg_to_rad(30.0);
constexpr double kSweeperMinRadius = 14.0;
constexpr double kChicaneMinAngle = deg_to_rad(15.0);
constexpr double kChicaneMaxAngle = deg_to_rad(100.0);
constexpr double kChicaneMaxGap = 10.0;

struct Turn {
    double angle = 0.0;  // signed total heading change (rad)
    double max_curvature = 0.0;
    double s_begin = 0.0;
    double s_end = 0.0;
};

// Length of the centerline piece that sample i stands for: half of each neighbouring interval.
double sample_length(std::span<const ProfileSample> samples, double length, std::size_t i) {
    const std::size_t n = samples.size();
    const double next = (i + 1 < n) ? samples[i + 1].s : length;
    const double prev = (i > 0) ? samples[i - 1].s : samples[n - 1].s - length;
    return 0.5 * (next - prev);
}

// Index of a straight sample to start cyclic scans from, so no run is split by the wrap.
std::optional<std::size_t> straight_sample(std::span<const ProfileSample> samples,
                                           double threshold) {
    for (std::size_t i = 0; i < samples.size(); ++i) {
        if (std::abs(samples[i].curvature) < threshold) return i;
    }
    return std::nullopt;
}

double longest_straight(std::span<const ProfileSample> samples, double length, double threshold) {
    const std::optional<std::size_t> start = straight_sample(samples, threshold);
    if (!start) return 0.0;
    const std::size_t n = samples.size();
    double best = 0.0;
    double run = 0.0;
    for (std::size_t k = 0; k <= n; ++k) {
        const std::size_t i = (*start + k) % n;
        if (k < n && std::abs(samples[i].curvature) < threshold) {
            run += sample_length(samples, length, i);
        } else {
            best = std::max(best, run);
            run = 0.0;
        }
    }
    return best;
}

std::vector<Turn> find_turns(std::span<const ProfileSample> samples, double length,
                             double threshold) {
    std::vector<Turn> turns;
    const std::size_t n = samples.size();
    const std::optional<std::size_t> start = straight_sample(samples, threshold);
    if (!start) {
        // No straight at all (a circle): one turn around the whole loop.
        Turn turn;
        for (std::size_t i = 0; i < n; ++i) {
            turn.angle += samples[i].curvature * sample_length(samples, length, i);
            turn.max_curvature = std::max(turn.max_curvature, std::abs(samples[i].curvature));
        }
        turn.s_end = length;
        turns.push_back(turn);
        return turns;
    }
    bool in_turn = false;
    Turn current;
    for (std::size_t k = 0; k <= n; ++k) {
        const std::size_t i = (*start + k) % n;
        const double kappa = k < n ? samples[i].curvature : 0.0;
        const bool turning = std::abs(kappa) >= threshold;
        const bool same_sign = in_turn && (kappa > 0.0) == (current.angle > 0.0);
        if (in_turn && (!turning || !same_sign)) {
            turns.push_back(current);
            in_turn = false;
        }
        if (turning && !in_turn) {
            current = Turn{};
            current.s_begin = samples[i].s;
            in_turn = true;
        }
        if (in_turn) {
            current.angle += kappa * sample_length(samples, length, i);
            current.max_curvature = std::max(current.max_curvature, std::abs(kappa));
            current.s_end = samples[i].s;
        }
    }
    return turns;
}

bool is_hairpin(const Turn& turn) {
    return std::abs(turn.angle) >= kHairpinMinAngle &&
           turn.max_curvature >= 1.0 / kHairpinMaxRadius;
}

bool chicane_half(const Turn& turn) {
    const double angle = std::abs(turn.angle);
    return angle >= kChicaneMinAngle && angle <= kChicaneMaxAngle &&
           turn.max_curvature > 1.0 / kSweeperMinRadius;
}

void classify_turns(const std::vector<Turn>& turns, double length, TrackStats& stats) {
    const std::size_t n = turns.size();
    std::vector<bool> used(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        if (used[i]) continue;
        const std::size_t j = (i + 1) % n;
        if (n > 1 && !used[j] && j != i) {
            const Turn& a = turns[i];
            const Turn& b = turns[j];
            double gap = b.s_begin - a.s_end;
            if (gap < 0.0) gap += length;
            if ((a.angle > 0.0) != (b.angle > 0.0) && gap <= kChicaneMaxGap && chicane_half(a) &&
                chicane_half(b)) {
                ++stats.chicanes;
                used[i] = true;
                used[j] = true;
                continue;
            }
        }
        used[i] = true;
        const Turn& turn = turns[i];
        if (is_hairpin(turn)) {
            ++stats.hairpins;
        } else if (std::abs(turn.angle) >= kSweeperMinAngle &&
                   turn.max_curvature <= 1.0 / kSweeperMinRadius) {
            ++stats.sweepers;
        } else {
            ++stats.corners;
        }
    }
}

// Smallest gap between the cone lines of two centerline points more than `adjacency` apart
// along the loop. O(n^2) over ~1000 samples: about a millisecond.
double min_clearance(std::span<const ProfileSample> samples, double length, double adjacency) {
    double best = std::numeric_limits<double>::infinity();
    const std::size_t n = samples.size();
    for (std::size_t i = 0; i < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            const double ds = samples[j].s - samples[i].s;
            if (ds < adjacency || length - ds < adjacency) continue;
            const double distance = (samples[j].position - samples[i].position).norm();
            best = std::min(best, distance - samples[i].half_width - samples[j].half_width);
        }
    }
    return best;
}

}  // namespace

TrackStats analyze_profile(std::span<const ProfileSample> samples, double length,
                           const TrackRules& rules) {
    TrackStats stats;
    stats.length = length;
    if (samples.empty()) return stats;
    double max_curvature = 0.0;
    for (const ProfileSample& sample : samples) {
        max_curvature = std::max(max_curvature, std::abs(sample.curvature));
    }
    stats.min_radius =
        max_curvature > 0.0 ? 1.0 / max_curvature : std::numeric_limits<double>::infinity();
    stats.longest_straight = longest_straight(samples, length, rules.straight_curvature);
    stats.min_clearance = min_clearance(samples, length, rules.adjacency_length);
    classify_turns(find_turns(samples, length, rules.straight_curvature), length, stats);
    return stats;
}

std::optional<std::string> violated_rule(const TrackStats& stats, const TrackRules& rules,
                                         double margin) {
    if (stats.length < rules.min_length || stats.length > rules.max_length) {
        return std::format("length {:.1f} m is outside [{:.0f}, {:.0f}] m", stats.length,
                           rules.min_length, rules.max_length);
    }
    if (stats.min_radius < rules.min_radius + margin) {
        return std::format("radius {:.2f} m is below the {:.2f} m minimum", stats.min_radius,
                           rules.min_radius);
    }
    if (stats.longest_straight > rules.max_straight) {
        return std::format("a {:.1f} m straight exceeds {:.0f} m", stats.longest_straight,
                           rules.max_straight);
    }
    if (stats.min_clearance < rules.min_clearance + margin) {
        return std::format("{:.2f} m between non-adjacent parts is below the {:.2f} m minimum",
                           stats.min_clearance, rules.min_clearance);
    }
    return std::nullopt;
}

}  // namespace detail

namespace {

std::vector<detail::ProfileSample> centerline_profile(const Track& track) {
    std::vector<detail::ProfileSample> samples;
    samples.reserve(track.centerline.size());
    for (const TrajectoryPoint& p : track.centerline.points()) {
        samples.push_back({p.s, p.position, p.curvature, std::max(p.width_left, p.width_right)});
    }
    return samples;
}

}  // namespace

TrackStats analyze_track(const Track& track, const TrackRules& rules) {
    return detail::analyze_profile(centerline_profile(track), track.centerline.length(), rules);
}

std::optional<std::string> check_track_rules(const Track& track, const TrackRules& rules) {
    return detail::violated_rule(analyze_track(track, rules), rules);
}

}  // namespace apex::sim
