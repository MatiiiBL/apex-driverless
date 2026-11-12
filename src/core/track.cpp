#include "apex/core/track.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <fstream>
#include <stdexcept>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/json.hpp"

namespace apex {

namespace {

Vec2 edge_point(const TrajectoryPoint& p, double offset) {
    return p.position + offset * left_normal(heading_vector(p.yaw));
}

std::vector<Vec2> positions(const Trajectory& trajectory) {
    std::vector<Vec2> result;
    result.reserve(trajectory.size());
    for (const TrajectoryPoint& p : trajectory.points()) result.push_back(p.position);
    return result;
}

// Cone spacing that keeps the chord between neighbours within `sagitta` of an edge with
// curvature `kappa`: a chord c on a circle of radius R bulges c^2 / (8 R) from the arc.
double cone_spacing(double kappa, const TrackBuildOptions& options) {
    const double k = std::abs(kappa);
    if (k < 1e-9) return options.max_cone_spacing;
    return std::clamp(std::sqrt(8.0 * options.max_chord_sagitta / k), options.min_cone_spacing,
                      options.max_cone_spacing);
}

// Cones along one edge (side = +1 left, -1 right) strictly between the big orange cones at
// s = big_cone_offset and s = length - big_cone_offset. Positions follow a density of
// 1 / cone_spacing, divided into equal shares, so no gap exceeds the local spacing.
std::vector<Vec2> place_edge_cones(const Trajectory& center, double side,
                                   const TrackBuildOptions& options) {
    const double offset = side * options.half_width;
    const double s_begin = options.big_cone_offset;
    const double s_end = center.length() - options.big_cone_offset;
    std::vector<Vec2> edge;
    std::vector<double> density;
    auto add = [&](const TrajectoryPoint& p) {
        // Curvature of the offset curve: the edge on the inside of a turn is tighter.
        const double kappa = p.curvature / (1.0 - offset * p.curvature);
        edge.push_back(edge_point(p, offset));
        density.push_back(1.0 / cone_spacing(kappa, options));
    };
    add(center.at(s_begin));
    for (const TrajectoryPoint& p : center.points()) {
        if (p.s > s_begin && p.s < s_end) add(p);
    }
    add(center.at(s_end));

    std::vector<double> cumulative(edge.size(), 0.0);
    for (std::size_t i = 1; i < edge.size(); ++i) {
        const double length = (edge[i] - edge[i - 1]).norm();
        cumulative[i] = cumulative[i - 1] + 0.5 * (density[i] + density[i - 1]) * length;
    }
    const double total = cumulative.back();
    const auto intervals =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(total - 1e-9)));
    std::vector<Vec2> cones;
    std::size_t j = 0;
    for (std::size_t k = 1; k < intervals; ++k) {
        const double target = total * static_cast<double>(k) / static_cast<double>(intervals);
        while (j + 1 < cumulative.size() && cumulative[j + 1] < target) ++j;
        const double span = cumulative[j + 1] - cumulative[j];
        const double u = span > 0.0 ? (target - cumulative[j]) / span : 0.0;
        cones.push_back(edge[j] + u * (edge[j + 1] - edge[j]));
    }
    return cones;
}

}  // namespace

std::vector<Vec2> Track::left_boundary() const {
    std::vector<Vec2> result;
    result.reserve(centerline.size());
    for (const TrajectoryPoint& p : centerline.points())
        result.push_back(edge_point(p, p.width_left));
    return result;
}

std::vector<Vec2> Track::right_boundary() const {
    std::vector<Vec2> result;
    result.reserve(centerline.size());
    for (const TrajectoryPoint& p : centerline.points()) {
        result.push_back(edge_point(p, -p.width_right));
    }
    return result;
}

Track track_from_centerline(std::span<const Vec2> waypoints, const TrackBuildOptions& options,
                            std::string name, std::uint64_t seed) {
    Track track;
    track.name = std::move(name);
    track.seed = seed;
    track.centerline = Trajectory::from_waypoints(waypoints, true, options.centerline_spacing);
    const std::vector<double> widths(track.centerline.size(), options.half_width);
    track.centerline.set_widths(widths, widths);
    const Trajectory& center = track.centerline;
    const double h = options.half_width;

    auto add_cone = [&track](const Vec2& position, ConeColor color) {
        const auto id = static_cast<std::int32_t>(track.cones.size());
        track.cones.push_back({id, position, color});
    };
    for (const double side : {1.0, -1.0}) {
        add_cone(edge_point(center.at(options.big_cone_offset), side * h), ConeColor::BigOrange);
        add_cone(edge_point(center.at(center.length() - options.big_cone_offset), side * h),
                 ConeColor::BigOrange);
    }
    for (const Vec2& p : place_edge_cones(center, 1.0, options)) add_cone(p, ConeColor::Blue);
    for (const Vec2& p : place_edge_cones(center, -1.0, options)) add_cone(p, ConeColor::Yellow);

    const TrajectoryPoint line = center.at(0.0);
    track.timing_line_left = edge_point(line, h);
    track.timing_line_right = edge_point(line, -h);
    const TrajectoryPoint staging = center.at(center.length() - options.staging_distance);
    track.start_pose = {staging.position.x(), staging.position.y(), staging.yaw};

    validate_track(track, options);
    return track;
}

Track make_oval_track(double straight_length, double radius, const TrackBuildOptions& options) {
    if (!(straight_length > 0.0) || !(radius > 0.0)) {
        throw std::invalid_argument("make_oval_track: lengths must be positive");
    }
    constexpr double kStep = 2.0;
    const double half = 0.5 * straight_length;
    std::vector<Vec2> waypoints;
    auto straight = [&](const Vec2& from, const Vec2& to, bool include_end) {
        const double length = (to - from).norm();
        const auto steps =
            std::max<std::size_t>(1, static_cast<std::size_t>(std::round(length / kStep)));
        const std::size_t last = include_end ? steps : steps - 1;
        for (std::size_t i = 0; i <= last; ++i) {
            waypoints.push_back(from + (to - from) *
                                           (static_cast<double>(i) / static_cast<double>(steps)));
        }
    };
    auto arc = [&](const Vec2& center, double from_angle) {
        const auto steps =
            std::max<std::size_t>(4, static_cast<std::size_t>(std::round(kPi * radius / kStep)));
        for (std::size_t i = 1; i < steps; ++i) {
            const double a = from_angle + kPi * static_cast<double>(i) / static_cast<double>(steps);
            waypoints.push_back(center + radius * heading_vector(a));
        }
    };
    straight({0.0, -radius}, {half, -radius}, true);
    arc({half, 0.0}, -0.5 * kPi);
    straight({half, radius}, {-half, radius}, true);
    arc({-half, 0.0}, 0.5 * kPi);
    straight({-half, -radius}, {0.0, -radius}, false);
    return track_from_centerline(waypoints, options, "oval");
}

void validate_track(const Track& track, const TrackBuildOptions& options) {
    const Trajectory& center = track.centerline;
    if (!center.closed() || center.size() < 3) {
        throw std::invalid_argument("track: the centerline must be a closed path");
    }
    const std::vector<Vec2> center_points = positions(center);
    if (polyline_self_intersects(center_points, true)) {
        throw std::invalid_argument("track: the centerline crosses itself");
    }
    const std::vector<Vec2> left = track.left_boundary();
    const std::vector<Vec2> right = track.right_boundary();
    if (polyline_self_intersects(left, true)) {
        throw std::invalid_argument("track: the left boundary crosses itself");
    }
    if (polyline_self_intersects(right, true)) {
        throw std::invalid_argument("track: the right boundary crosses itself");
    }
    if (polylines_intersect(left, true, right, true)) {
        throw std::invalid_argument("track: the boundaries cross each other");
    }
    for (const TrajectoryPoint& p : center.points()) {
        const double min_radius = std::max(p.width_left, p.width_right) + 0.5;
        if (std::abs(p.curvature) * min_radius > 1.0) {
            throw std::invalid_argument(
                std::format("track: radius {:.2f} m at s = {:.1f} m is below the {:.2f} m minimum",
                            1.0 / std::abs(p.curvature), p.s, min_radius));
        }
    }
    std::vector<std::pair<double, Vec2>> sides[2];  // [0] left, [1] right, by s
    for (std::size_t i = 0; i < track.cones.size(); ++i) {
        const Cone& cone = track.cones[i];
        if (cone.id != static_cast<std::int32_t>(i)) {
            throw std::invalid_argument("track: cone ids must equal their index");
        }
        const Trajectory::Projection proj = center.project(cone.position);
        bool left_side = proj.lateral > 0.0;
        if (cone.color == ConeColor::Blue) left_side = true;
        if (cone.color == ConeColor::Yellow) left_side = false;
        sides[left_side ? 0 : 1].emplace_back(proj.s, cone.position);
    }
    for (auto& side : sides) {
        if (side.size() < 3) throw std::invalid_argument("track: too few cones on a side");
        std::sort(side.begin(), side.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        for (std::size_t i = 0; i < side.size(); ++i) {
            const double gap = (side[(i + 1) % side.size()].second - side[i].second).norm();
            if (gap > options.max_cone_spacing + 0.05) {
                throw std::invalid_argument(
                    std::format("track: {:.2f} m between cones at s = {:.1f} m exceeds {:.2f} m",
                                gap, side[i].first, options.max_cone_spacing));
            }
        }
    }
}

void save_track(const Track& track, const std::filesystem::path& path) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("save_track: cannot open " + path.string());
    out << nlohmann::json(track).dump(1) << '\n';
    if (!out) throw std::runtime_error("save_track: write failed for " + path.string());
}

Track load_track(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("load_track: cannot open " + path.string());
    Track track = nlohmann::json::parse(in).get<Track>();
    return track;
}

}  // namespace apex
