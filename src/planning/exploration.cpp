#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

#include "apex/core/geometry.hpp"
#include "apex/planning/midpoint_search.hpp"
#include "apex/planning/planning.hpp"
#include "apex/planning/smoothing.hpp"

// ExplorationPlanner (docs/planning.md, "Exploration"): select the cones near and ahead of the
// car, find the best midpoint path through them (midpoint_search.hpp), smooth it, and give it a
// speed profile that stops before the end of what has been seen.
namespace apex::planning {

namespace {

// The midpoints are resampled at this spacing (m) before smoothing.
constexpr double kSmoothingSpacing = 1.0;

// The side a cone's color votes for, weighted by their share: +1 left (blue), -1 right
// (yellow), 0 for orange or unknown.
double color_side(const MapCone& cone) {
    if (cone.color != ConeColor::Blue && cone.color != ConeColor::Yellow) return 0.0;
    const double total = std::accumulate(cone.color_votes.begin(), cone.color_votes.end(), 0.0);
    const double share = total > 0.0 ? cone.color_votes[index_of(cone.color)] / total : 1.0;
    return cone.color == ConeColor::Blue ? share : -share;
}

// The cones within range and not too far behind the car, the closest max_cones of them, in map
// order.
std::vector<SearchCone> select_cones(const ConeMap& map, const Pose2& car,
                                     const ExplorationConfig& config) {
    struct Candidate {
        double distance;
        std::size_t index;
    };
    std::vector<Candidate> candidates;
    for (std::size_t i = 0; i < map.cones.size(); ++i) {
        const Vec2& p = map.cones[i].position;
        if (!p.allFinite()) continue;
        const Vec2 local = car.to_local(p);
        const double distance = local.norm();
        if (distance <= config.cone_range && local.x() >= -config.behind_distance) {
            candidates.push_back({distance, i});
        }
    }
    if (candidates.size() > config.max_cones) {
        std::stable_sort(
            candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) { return a.distance < b.distance; });
        candidates.resize(config.max_cones);
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) { return a.index < b.index; });
    }
    std::vector<SearchCone> cones;
    cones.reserve(candidates.size());
    for (const Candidate& c : candidates) {
        cones.push_back({map.cones[c.index].position, color_side(map.cones[c.index])});
    }
    return cones;
}

// The cones on one side of the path, in order, without repeats: the cone line beside it. The
// path starts at the car, before the side's first cone, and may end a cone spacing past its
// last, so the line is extended backwards along its first segment past the car and forwards by
// `reach_ahead` meters along its last (both along the car's heading if it has a single cone).
std::vector<Vec2> cone_line(std::span<const SearchCone> cones, std::span<const std::int32_t> side,
                            const Pose2& car, double reach_ahead) {
    std::vector<Vec2> line;
    for (std::size_t i = 0; i < side.size(); ++i) {
        if (i == 0 || side[i] != side[i - 1]) {
            line.push_back(cones[static_cast<std::size_t>(side[i])].position);
        }
    }
    const std::size_t n = line.size();
    const Vec2 ahead = heading_vector(car.yaw);
    const Vec2 first = n >= 2 ? Vec2((line[1] - line[0]).normalized()) : ahead;
    const Vec2 last = n >= 2 ? Vec2((line[n - 1] - line[n - 2]).normalized()) : ahead;
    const double reach_back = (line.front() - car.position()).norm() + 1.0;
    line.push_back(line.back() + reach_ahead * last);
    line.insert(line.begin(), line.front() - reach_back * first);
    return line;
}

// The smoothed path from the car through the midpoints, sampled every path_spacing.
Trajectory smooth_path(const MidpointPath& found, const Pose2& car,
                       const ExplorationConfig& config) {
    std::vector<Vec2> raw{car.position()};
    raw.insert(raw.end(), found.midpoints.begin(), found.midpoints.end());
    const std::vector<Vec2> even = resample_polyline(raw, false, kSmoothingSpacing);
    const double length = cumulative_length(raw, false).back();
    SmoothingOptions options;
    options.spacing = length / static_cast<double>(even.size() - 1);
    options.cutoff_wavelength = config.smoothing_wavelength;
    options.start_direction = heading_vector(car.yaw);
    return Trajectory::from_waypoints(smooth_polyline(even, options), false, config.path_spacing);
}

// The speed profile from the car's speed down to a stop before the path's end: stop_margin
// before it, or stop_reserve of the path's length, whichever is more. The reserve is for the
// next plan: when the cones ahead turn out to be fewer than they seemed (a hairpin's apex not
// mapped yet), that plan's path ends closer, and the car must still be able to stop within it.
void stop_before_the_end(Trajectory& path, double speed_now, const VehicleParams& nominal,
                         const ExplorationConfig& config) {
    const double stop_s =
        path.length() - std::max(config.stop_margin, config.stop_reserve * path.length());
    std::vector<TrajectoryPoint> head;
    for (const TrajectoryPoint& p : path.points()) {
        if (p.s <= stop_s) head.push_back(p);
    }
    std::vector<double> speed(path.size(), 0.0);
    std::vector<double> accel(path.size(), 0.0);
    if (head.size() >= 2) {
        Trajectory braking(std::move(head), false);
        apply_speed_profile(braking, nominal, config.speed, std::max(0.0, speed_now), 0.0);
        for (std::size_t i = 0; i < braking.size(); ++i) {
            speed[i] = braking[i].speed;
            accel[i] = braking[i].accel;
        }
    }
    path.set_speed_profile(speed, accel);
}

bool finite(const Trajectory& path) {
    return std::all_of(path.points().begin(), path.points().end(), [](const TrajectoryPoint& p) {
        return p.position.allFinite() && std::isfinite(p.speed) && std::isfinite(p.accel) &&
               std::isfinite(p.width_left) && std::isfinite(p.width_right);
    });
}

}  // namespace

struct ExplorationPlanner::Impl {
    ExplorationConfig config;
    VehicleParams nominal;
    ExplorationStats stats;

    std::optional<Trajectory> plan(const ConeMap& map, const StateEstimate& state) const {
        const Pose2& car = state.pose;
        if (!state.valid || !std::isfinite(car.x) || !std::isfinite(car.y) ||
            !std::isfinite(car.yaw)) {
            return std::nullopt;
        }
        const std::vector<SearchCone> cones = select_cones(map, car, config);
        const std::optional<MidpointPath> found = search_midpoint_path(cones, car, config);
        if (!found || found->length < config.min_path_length) return std::nullopt;

        Trajectory path = smooth_path(*found, car, config);
        const std::vector<Vec2> left = cone_line(cones, found->left, car, config.cone_spacing);
        const std::vector<Vec2> right = cone_line(cones, found->right, car, config.cone_spacing);
        std::vector<double> width_left;
        std::vector<double> width_right;
        for (const TrajectoryPoint& p : path.points()) {
            width_left.push_back(distance_to_polyline(p.position, left, false));
            width_right.push_back(distance_to_polyline(p.position, right, false));
        }
        path.set_widths(width_left, width_right);
        stop_before_the_end(path, state.vx, nominal, config);
        if (!finite(path)) return std::nullopt;
        return path;
    }
};

ExplorationPlanner::ExplorationPlanner(const ExplorationConfig& config,
                                       const VehicleParams& nominal)
    : impl_(std::make_unique<Impl>()) {
    const bool ok = config.cone_range > 0.0 && config.behind_distance >= 0.0 &&
                    config.max_cones >= 3 && config.beam_width >= 1 && config.max_edges >= 1 &&
                    config.max_path_length > 0.0 && config.start_radius > 0.0 &&
                    config.min_width > 0.0 && config.max_width > config.min_width &&
                    config.max_edge_length > 0.0 && config.max_cone_spacing > 0.0 &&
                    config.max_turn_deg > 0.0 && config.track_width > 0.0 &&
                    config.min_cone_clearance >= 0.0 && config.curvature_scale > 0.0 &&
                    config.cone_spacing > 0.0 && config.heading_scale_deg > 0.0 &&
                    config.smoothing_wavelength > 0.0 && config.path_spacing > 0.0 &&
                    config.stop_margin >= 0.0 && config.stop_reserve >= 0.0 &&
                    config.stop_reserve < 1.0 && config.min_path_length > config.stop_margin;
    if (!ok) throw std::invalid_argument("ExplorationPlanner: invalid ExplorationConfig");
    impl_->config = config;
    impl_->nominal = nominal;
}

ExplorationPlanner::~ExplorationPlanner() = default;
ExplorationPlanner::ExplorationPlanner(ExplorationPlanner&&) noexcept = default;
ExplorationPlanner& ExplorationPlanner::operator=(ExplorationPlanner&&) noexcept = default;

std::optional<Trajectory> ExplorationPlanner::plan(const ConeMap& map, const StateEstimate& state) {
    const Stopwatch stopwatch;
    std::optional<Trajectory> path = impl_->plan(map, state);
    ++impl_->stats.plans;
    if (!path) ++impl_->stats.failures;
    impl_->stats.plan_time.add(stopwatch.elapsed_seconds());
    return path;
}

const ExplorationStats& ExplorationPlanner::stats() const {
    return impl_->stats;
}

}  // namespace apex::planning
