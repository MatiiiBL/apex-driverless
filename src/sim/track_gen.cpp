#include "apex/sim/track_gen.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <optional>
#include <stdexcept>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"

#include "track_layout.hpp"

namespace apex::sim {

namespace {

using detail::Element;
using detail::StraightSpec;
using detail::TurnSpec;

enum class Feature { Hairpin, Corner, Sweeper, Chicane };

// Feature dimensions, chosen to cover the FS trackdrive vocabulary: hairpins just above the
// minimum radius, medium corners, constant-radius sweepers up to a 50 m diameter, and chicanes
// of two short opposite turns.
constexpr double kHairpinMinAngle = deg_to_rad(150.0);
constexpr double kHairpinMaxAngle = deg_to_rad(190.0);
constexpr double kCornerMinRadius = 8.0;
constexpr double kCornerMaxRadius = 15.0;
constexpr double kSweeperMinRadius = 15.0;
constexpr double kSweeperMaxRadius = 25.0;
constexpr double kChicaneMaxRadius = 14.0;

// Candidates are checked on their exact layout with this much margin on radius and clearance,
// so the spline fit and resampling of track_from_centerline rarely push them over.
constexpr double kCandidateMargin = 0.3;

void validate_options(const TrackGenOptions& options) {
    const TrackRules& rules = options.rules;
    if (!(rules.min_length > 0.0) || !(rules.max_length > rules.min_length + 50.0)) {
        throw std::invalid_argument("generate_track: need 0 < min_length < max_length - 50 m");
    }
    if (!(rules.min_radius > options.build.half_width + 0.5) || !(rules.min_radius <= 10.0)) {
        throw std::invalid_argument(
            "generate_track: min_radius must exceed the half width + 0.5 m and be at most 10 m");
    }
    if (!(rules.max_straight >= 50.0) || !(rules.min_clearance >= 0.0) ||
        !(rules.adjacency_length > 0.0) || !(rules.straight_curvature > 0.0)) {
        throw std::invalid_argument("generate_track: invalid straight or clearance rule");
    }
    const std::array weights = {options.hairpin_weight, options.corner_weight,
                                options.sweeper_weight, options.chicane_weight};
    double sum = 0.0;
    for (const double w : weights) {
        if (!(w >= 0.0)) throw std::invalid_argument("generate_track: negative feature weight");
        sum += w;
    }
    if (!(sum > 0.0)) throw std::invalid_argument("generate_track: all feature weights are zero");
    if (options.max_attempts < 1) throw std::invalid_argument("generate_track: max_attempts < 1");
}

Feature draw_feature(const TrackGenOptions& options, Rng& rng) {
    const std::array weights = {options.hairpin_weight, options.corner_weight,
                                options.sweeper_weight, options.chicane_weight};
    double total = 0.0;
    for (const double w : weights) total += w;
    double u = rng.uniform(0.0, total);
    for (std::size_t i = 0; i < weights.size(); ++i) {
        if (u < weights[i]) return static_cast<Feature>(i);
        u -= weights[i];
    }
    return Feature::Corner;
}

// Turns go the loop's way (`direction` = +1 counter-clockwise) with probability `with`.
double turn_sign(double direction, double with, Rng& rng) {
    return rng.bernoulli(with) ? direction : -direction;
}

// Appends a feature's turns (and a chicane's short middle straight); returns its arc length.
double add_feature(Feature feature, double direction, const TrackRules& rules, Rng& rng,
                   std::vector<Element>& out) {
    switch (feature) {
        case Feature::Hairpin: {
            const double radius = rng.uniform(rules.min_radius + 0.5, rules.min_radius + 3.0);
            const double angle = rng.uniform(kHairpinMinAngle, kHairpinMaxAngle);
            out.emplace_back(TurnSpec{turn_sign(direction, 0.8, rng) * angle, radius, false});
            return angle * radius;
        }
        case Feature::Corner: {
            const double radius =
                rng.uniform(std::max(kCornerMinRadius, rules.min_radius + 1.0), kCornerMaxRadius);
            const double angle = rng.uniform(deg_to_rad(40.0), deg_to_rad(110.0));
            out.emplace_back(TurnSpec{turn_sign(direction, 0.7, rng) * angle, radius, true,
                                      deg_to_rad(25.0), deg_to_rad(150.0)});
            return angle * radius;
        }
        case Feature::Sweeper: {
            const double radius = rng.uniform(kSweeperMinRadius, kSweeperMaxRadius);
            const double angle = rng.uniform(deg_to_rad(40.0), deg_to_rad(100.0));
            out.emplace_back(TurnSpec{turn_sign(direction, 0.75, rng) * angle, radius, true,
                                      deg_to_rad(30.0), deg_to_rad(140.0)});
            return angle * radius;
        }
        case Feature::Chicane: {
            const double min_radius = std::max(kCornerMinRadius, rules.min_radius + 1.0);
            const double sign = rng.bernoulli(0.5) ? 1.0 : -1.0;
            const double r1 = rng.uniform(min_radius, kChicaneMaxRadius);
            const double r2 = rng.uniform(min_radius, kChicaneMaxRadius);
            const double a1 = rng.uniform(deg_to_rad(30.0), deg_to_rad(60.0));
            const double a2 = a1 * rng.uniform(0.8, 1.2);
            const double gap = rng.uniform(0.5, 4.0);
            out.emplace_back(TurnSpec{sign * a1, r1, false});
            out.emplace_back(StraightSpec{gap, false});
            out.emplace_back(TurnSpec{-sign * a2, r2, false});
            return a1 * r1 + gap + a2 * r2;
        }
    }
    return 0.0;
}

// Scales the scalable turns by one factor in [0.6, 1.6] so the total turning is exactly
// direction * 2 pi. If they turn too little the needed way for that, a random one turning the
// other way is flipped first (and the reverse if they turn too much), until a factor fits.
// False if none does with every turn left in its range.
bool close_heading(std::vector<Element>& elements, double direction, Rng& rng) {
    constexpr double kMinScale = 0.6;
    constexpr double kMaxScale = 1.6;
    double fixed = 0.0;
    std::vector<TurnSpec*> scalable;
    for (Element& element : elements) {
        if (auto* turn = std::get_if<TurnSpec>(&element)) {
            if (turn->scalable) {
                scalable.push_back(turn);
            } else {
                fixed += turn->angle;
            }
        }
    }
    const double needed = direction * kTwoPi - fixed;
    for (std::size_t flips = 0; flips <= scalable.size(); ++flips) {
        double total = 0.0;
        for (const TurnSpec* turn : scalable) total += turn->angle;
        const double factor = std::abs(total) > 1e-6 ? needed / total : 0.0;
        if (factor >= kMinScale && factor <= kMaxScale) {
            for (TurnSpec* turn : scalable) {
                turn->angle *= factor;
                const double magnitude = std::abs(turn->angle);
                if (magnitude < turn->min_angle || magnitude > turn->max_angle) return false;
            }
            return true;
        }
        // Too little turning the needed way: flip a turn that goes against it; too much: one
        // that goes with it.
        const bool too_little = factor > kMaxScale || factor <= 0.0;
        std::vector<TurnSpec*> candidates;
        for (TurnSpec* turn : scalable) {
            if ((turn->angle * needed < 0.0) == too_little) candidates.push_back(turn);
        }
        if (candidates.empty()) return false;
        TurnSpec* flipped = candidates[rng.index(candidates.size())];
        flipped->angle = -flipped->angle;
    }
    return false;
}

// One random candidate, driven counter-clockwise (+1) or clockwise (-1): the start straight
// after the timekeeping line, the features with straights between them, and the start straight
// before the line. Its heading closes; its position does not yet.
std::optional<std::vector<Element>> draw_candidate(const TrackGenOptions& options, Rng& rng) {
    const TrackRules& rules = options.rules;
    const double direction = rng.bernoulli(0.5) ? 1.0 : -1.0;
    const double span = rules.max_length - rules.min_length;
    const double target = rng.uniform(rules.min_length + 0.1 * span, rules.max_length - 0.1 * span);

    // Features are drawn until their length, with the start straight and the mean straight
    // between features, reaches the target; then shuffled, so the mandatory ones land anywhere.
    constexpr double kStartStraight = 45.0;
    constexpr double kMeanStraight = 17.5;
    std::vector<std::vector<Element>> features;
    double estimate = kStartStraight;
    auto add = [&](Feature feature) {
        std::vector<Element>& group = features.emplace_back();
        estimate += add_feature(feature, direction, rules, rng, group) + kMeanStraight;
    };
    add(Feature::Hairpin);
    add(Feature::Chicane);
    add(Feature::Sweeper);
    while (estimate < target) add(draw_feature(options, rng));
    for (std::size_t i = features.size(); i > 1; --i) {
        std::swap(features[i - 1], features[rng.index(i)]);
    }

    std::vector<Element> elements;
    elements.emplace_back(StraightSpec{rng.uniform(12.0, 20.0), true, 8.0, 30.0});
    for (std::size_t i = 0; i < features.size(); ++i) {
        elements.insert(elements.end(), features[i].begin(), features[i].end());
        if (i + 1 < features.size()) {
            elements.emplace_back(StraightSpec{rng.uniform(5.0, 30.0), true, 2.0, 60.0});
        }
    }
    elements.emplace_back(StraightSpec{rng.uniform(22.0, 35.0), true, 16.0, 45.0});
    if (!close_heading(elements, direction, rng)) return std::nullopt;
    return elements;
}

// The candidate as a track, or nothing if it breaks a rule.
std::optional<Track> build_candidate(const std::vector<Element>& elements,
                                     const TrackGenOptions& options, std::uint64_t seed) {
    detail::Layout layout(elements);
    if (!layout.close()) return std::nullopt;
    const std::vector<detail::ProfileSample> samples =
        layout.sample(detail::kWaypointSpacing, options.build.half_width);
    const TrackStats stats = detail::analyze_profile(samples, layout.length(), options.rules);
    if (detail::violated_rule(stats, options.rules, kCandidateMargin)) return std::nullopt;

    std::vector<Vec2> waypoints;
    waypoints.reserve(samples.size());
    for (const detail::ProfileSample& sample : samples) waypoints.push_back(sample.position);
    try {
        Track track = track_from_centerline(waypoints, options.build,
                                            std::format("generated-{}", seed), seed);
        if (check_track_rules(track, options.rules)) return std::nullopt;
        return track;
    } catch (const std::invalid_argument&) {
        return std::nullopt;
    }
}

}  // namespace

GeneratedTrack generate_track_with_stats(std::uint64_t seed, const TrackGenOptions& options) {
    validate_options(options);
    Rng rng(seed);
    for (int attempt = 1; attempt <= options.max_attempts; ++attempt) {
        const std::optional<std::vector<Element>> elements = draw_candidate(options, rng);
        if (!elements) continue;
        if (std::optional<Track> track = build_candidate(*elements, options, seed)) {
            return {std::move(*track), attempt};
        }
    }
    throw std::runtime_error(std::format(
        "generate_track: no valid track for seed {} in {} attempts", seed, options.max_attempts));
}

Track generate_track(std::uint64_t seed, const TrackGenOptions& options) {
    return generate_track_with_stats(seed, options).track;
}

}  // namespace apex::sim
