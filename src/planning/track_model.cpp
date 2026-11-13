#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/log.hpp"
#include "apex/planning/planning.hpp"
#include "apex/planning/smoothing.hpp"

// build_track_model (docs/planning.md, "Track model").
//
// The driven lap-1 path is a line inside the track, so each cone's side is the side of that line
// it lies on: geometry decides, not the camera's color votes, which a real map gets wrong for a
// few percent of its cones. Along a reference line r(s) with unit left normals n(s), the
// centerline is r + c n and the cone lines are r + (c +/- w) n, with c(s) the centerline's
// offset and w(s) the half width. Every cone measures c + w (left) or c - w (right) at its arc
// length; c and w come from one sparse least-squares problem
//
//   minimize  sum_cones (c(s_j) +/- w(s_j) - lateral_j)^2
//           + lambda_c sum_i |third difference of (r_i + c_i n_i)|^2
//           + lambda_w sum_i (second difference of w_i)^2.
//
// The shape prior is on the centerline's geometry, not on c: where both sides miss cones the
// solution bridges the gap with the smoothest curve, instead of copying the reference's own
// wiggles (the driven path's wander) into the model. It penalizes the third difference, the
// rate of change of curvature, because a curvature prior pulls every corner inward: a circle of
// radius R shrinks by (cutoff / 2 pi)^4 / R^3 under the second difference and only by
// (cutoff / 2 pi)^6 / R^5 under the third, 3 cm against 2 mm on a 6 m hairpin at a 10 m cutoff.
// The stiffer width prior lets each side's cones shape the other side across its gaps. The
// first pass uses the driven path as the reference; each further pass uses the previous
// centerline, which places the cones better.
namespace apex::planning {

namespace {

// A cone placed relative to a closed reference line.
struct Placed {
    double s = 0.0;        // arc length of its projection, in [0, length)
    double lateral = 0.0;  // signed distance, positive to the left
    Vec2 position = Vec2::Zero();
};

struct Sides {
    std::vector<Placed> left;
    std::vector<Placed> right;
};

// The centerline's offset c and the half width w at each reference point.
struct Profile {
    Eigen::VectorXd offset;
    Eigen::VectorXd half_width;
};

Vec2 normal_at(const TrajectoryPoint& p) {
    return left_normal(heading_vector(p.yaw));
}

std::optional<TrackModel> fail(std::string_view reason) {
    log(LogLevel::Warn, "build_track_model: {}", reason);
    return std::nullopt;
}

void check_config(const TrackModelConfig& c) {
    const bool ok = c.reference_spacing > 0.0 && c.min_lateral >= 0.0 &&
                    c.max_lateral > c.min_lateral && c.outlier_distance > 0.0 &&
                    c.fill_spacing > 0.0 && c.fill_gap > 0.0 && c.max_gap >= c.fill_gap &&
                    c.max_closure_gap > 0.0 && c.smoothing_wavelength > 0.0 &&
                    c.width_wavelength > 0.0 && c.centerline_spacing > 0.0;
    if (!ok) throw std::invalid_argument("build_track_model: invalid TrackModelConfig");
}

// The lap within the driven path, resampled every ~reference_spacing meters: from its first
// pose to the point of its second half that comes closest to that pose again.
std::optional<std::vector<Vec2>> driven_loop(std::span<const Pose2> path,
                                             const TrackModelConfig& config) {
    std::vector<Vec2> raw;
    raw.reserve(path.size());
    for (const Pose2& pose : path) {
        const Vec2 p = pose.position();
        if (!p.allFinite()) return std::nullopt;
        if (raw.empty() || (p - raw.back()).norm() > 1e-6) raw.push_back(p);
    }
    if (raw.size() < 3) return std::nullopt;
    std::vector<Vec2> points = resample_polyline(raw, false, config.reference_spacing);
    std::size_t closure = points.size() - 1;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t j = points.size() / 2; j < points.size(); ++j) {
        const double d = (points[j] - points.front()).norm();
        if (d < best) {
            best = d;
            closure = j;
        }
    }
    if (!(best <= config.max_closure_gap)) return std::nullopt;
    // The closing segment joins the last point to the first; drop a last point on top of it.
    points.resize(best < 0.5 * config.reference_spacing ? closure : closure + 1);
    if (points.size() < 4) return std::nullopt;
    return points;
}

// Drops the cones whose lateral offset is far from the median offset of their side's nearest
// cones along the track: clutter beyond a cone line, or a cone of another part of the track.
// Counting neighbors rather than using a fixed window keeps the test working in the long gaps
// of a map that misses cones, where clutter would otherwise have no neighbors to differ from.
std::vector<Placed> without_outliers(const std::vector<Placed>& side, double length,
                                     const TrackModelConfig& config) {
    const std::size_t k = std::min(config.outlier_neighbors, side.size() - 1);
    if (side.empty() || k < 2) return side;
    std::vector<Placed> kept;
    std::vector<std::pair<double, double>> others;  // (distance along the track, lateral)
    std::vector<double> nearest(k);
    for (std::size_t i = 0; i < side.size(); ++i) {
        others.clear();
        for (std::size_t j = 0; j < side.size(); ++j) {
            if (j == i) continue;
            others.emplace_back(std::abs(std::remainder(side[j].s - side[i].s, length)),
                                side[j].lateral);
        }
        std::partial_sort(others.begin(), others.begin() + static_cast<std::ptrdiff_t>(k),
                          others.end());
        for (std::size_t n = 0; n < k; ++n) nearest[n] = others[n].second;
        std::sort(nearest.begin(), nearest.end());
        const double median = 0.5 * (nearest[(k - 1) / 2] + nearest[k / 2]);
        if (std::abs(side[i].lateral - median) <= config.outlier_distance) {
            kept.push_back(side[i]);
        }
    }
    return kept;
}

// Every cone within [min_lateral, max_lateral] of the reference, by side and sorted along it,
// outliers removed.
Sides place_cones(const ConeMap& map, const Trajectory& reference, const TrackModelConfig& config) {
    Sides sides;
    for (const MapCone& cone : map.cones) {
        if (!cone.position.allFinite()) continue;
        const Trajectory::Projection proj = reference.project(cone.position);
        if (proj.distance < config.min_lateral || proj.distance > config.max_lateral) continue;
        const Placed placed{reference.wrap_s(proj.s), proj.lateral, cone.position};
        (proj.lateral > 0.0 ? sides.left : sides.right).push_back(placed);
    }
    for (std::vector<Placed>* side : {&sides.left, &sides.right}) {
        std::stable_sort(side->begin(), side->end(),
                         [](const Placed& a, const Placed& b) { return a.s < b.s; });
        *side = without_outliers(*side, reference.length(), config);
    }
    return sides;
}

// The reference interval containing arc length s (in [0, length)): its first point's index and
// the interpolation weight of the next point.
std::pair<Eigen::Index, double> locate(const Trajectory& reference, double s) {
    const auto& points = reference.points();
    const auto after =
        std::upper_bound(points.begin(), points.end(), s,
                         [](double value, const TrajectoryPoint& p) { return value < p.s; });
    const auto k = static_cast<std::size_t>(
        std::max<std::ptrdiff_t>(0, std::distance(points.begin(), after) - 1));
    const double end = k + 1 < points.size() ? points[k + 1].s : reference.length();
    const double t = std::clamp((s - points[k].s) / (end - points[k].s), 0.0, 1.0);
    return {static_cast<Eigen::Index>(k), t};
}

// The least-squares estimate of the offset and half-width profiles (see the top of the file).
// Unknown i is c_i, unknown n + i is w_i. Each prior's weight is set so that its cutoff
// wavelength means what it says for the data's density: a k-th difference penalty halves a
// sinusoid of wavelength L at lambda = (L / 2 pi h)^(2k) per unit-weight data point (as in
// apex/planning/smoothing.hpp for k = 2), times the number of cones per reference point.
std::optional<Profile> estimate_profile(const Trajectory& reference, const Sides& sides,
                                        const TrackModelConfig& config) {
    const auto& points = reference.points();
    const auto n = static_cast<Eigen::Index>(points.size());
    const double h = reference.length() / static_cast<double>(n);
    const double density =
        static_cast<double>(sides.left.size() + sides.right.size()) / static_cast<double>(n);
    const auto prior_weight = [&](double wavelength, double order) {
        return std::sqrt(density * std::pow(wavelength / (kTwoPi * h), 2.0 * order));
    };
    const double shape_weight = prior_weight(config.smoothing_wavelength, 3.0);
    const double width_weight = prior_weight(config.width_wavelength, 2.0);

    std::vector<Eigen::Triplet<double>> entries;
    std::vector<double> targets;
    Eigen::Index row = 0;
    for (const auto& [side, sign] : {std::pair{&sides.left, 1.0}, std::pair{&sides.right, -1.0}}) {
        for (const Placed& cone : *side) {
            const auto [k, t] = locate(reference, cone.s);
            const Eigen::Index next = (k + 1) % n;
            entries.emplace_back(row, k, 1.0 - t);
            entries.emplace_back(row, next, t);
            entries.emplace_back(row, n + k, sign * (1.0 - t));
            entries.emplace_back(row, n + next, sign * t);
            targets.push_back(cone.lateral);
            ++row;
        }
    }
    constexpr std::array<double, 4> kThirdDifference = {-1.0, 3.0, -3.0, 1.0};
    constexpr std::array<double, 3> kSecondDifference = {1.0, -2.0, 1.0};
    for (Eigen::Index i = 0; i < n; ++i) {
        const std::array<Eigen::Index, 4> index = {(i + n - 1) % n, i, (i + 1) % n, (i + 2) % n};
        Vec2 reference_jerk = Vec2::Zero();
        for (std::size_t a = 0; a < 4; ++a) {
            reference_jerk +=
                kThirdDifference[a] * points[static_cast<std::size_t>(index[a])].position;
        }
        for (Eigen::Index axis = 0; axis < 2; ++axis) {
            for (std::size_t a = 0; a < 4; ++a) {
                const Vec2 normal = normal_at(points[static_cast<std::size_t>(index[a])]);
                entries.emplace_back(row, index[a],
                                     shape_weight * kThirdDifference[a] * normal[axis]);
            }
            targets.push_back(-shape_weight * reference_jerk[axis]);
            ++row;
        }
        for (std::size_t a = 0; a < 3; ++a) {
            entries.emplace_back(row, n + index[a], width_weight * kSecondDifference[a]);
        }
        targets.push_back(0.0);
        ++row;
    }

    Eigen::SparseMatrix<double> design(row, 2 * n);
    design.setFromTriplets(entries.begin(), entries.end());
    const Eigen::Map<const Eigen::VectorXd> target(targets.data(), row);
    const Eigen::SparseMatrix<double> normal = design.transpose() * design;
    const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(normal);
    if (solver.info() != Eigen::Success) return std::nullopt;
    const Eigen::VectorXd solution = solver.solve(design.transpose() * target);
    if (!solution.allFinite()) return std::nullopt;
    return Profile{solution.head(n), solution.tail(n)};
}

// The point at arc length s offset by c + sign * w along the reference's normal.
Vec2 profile_point(const Trajectory& reference, const Profile& profile, double s, double sign) {
    const auto [k, t] = locate(reference, s);
    const Eigen::Index next = (k + 1) % profile.offset.size();
    const auto lerp = [t](double a, double b) { return a + t * (b - a); };
    const double offset = lerp(profile.offset[k], profile.offset[next]) +
                          sign * lerp(profile.half_width[k], profile.half_width[next]);
    const TrajectoryPoint p = reference.at(s);
    return p.position + offset * normal_at(p);
}

std::vector<Vec2> centerline_points(const Trajectory& reference, const Profile& profile) {
    std::vector<Vec2> points;
    points.reserve(reference.size());
    for (std::size_t i = 0; i < reference.size(); ++i) {
        points.push_back(reference[i].position +
                         profile.offset[static_cast<Eigen::Index>(i)] * normal_at(reference[i]));
    }
    return points;
}

// A side's closed cone line in order, with points on its estimated line across every gap
// longer than fill_gap. Nothing if a gap exceeds max_gap.
std::optional<std::vector<Vec2>> cone_line(const std::vector<Placed>& side, double sign,
                                           const Trajectory& reference, const Profile& profile,
                                           const TrackModelConfig& config) {
    std::vector<Vec2> line;
    const auto append = [&line](const Vec2& p) {
        if (line.empty() || (p - line.back()).norm() > 1e-3) line.push_back(p);
    };
    for (std::size_t i = 0; i < side.size(); ++i) {
        const Placed& a = side[i];
        const Placed& b = side[(i + 1) % side.size()];
        const double gap = i + 1 < side.size() ? b.s - a.s : b.s + reference.length() - a.s;
        if (gap > config.max_gap) return std::nullopt;
        append(a.position);
        if (gap <= config.fill_gap) continue;
        const auto pieces = static_cast<std::size_t>(std::ceil(gap / config.fill_spacing));
        for (std::size_t k = 1; k < pieces; ++k) {
            const double s =
                reference.wrap_s(a.s + gap * static_cast<double>(k) / static_cast<double>(pieces));
            append(profile_point(reference, profile, s, sign));
        }
    }
    if (line.size() > 1 && (line.front() - line.back()).norm() <= 1e-3) line.pop_back();
    return line;
}

// Arc length of the timekeeping line on the reference: the middle of the largest cluster of
// big orange cones, or the projection of `fallback` if fewer than two are mapped.
double timing_line_s(const ConeMap& map, const Trajectory& reference, const Vec2& fallback,
                     const TrackModelConfig& config) {
    std::vector<double> s;
    for (const MapCone& cone : map.cones) {
        if (cone.color != ConeColor::BigOrange || !cone.position.allFinite()) continue;
        const Trajectory::Projection proj = reference.project(cone.position);
        if (proj.distance <= config.max_lateral) s.push_back(proj.s);
    }
    if (s.size() < 2) return reference.project(fallback).s;
    constexpr double kCluster = 3.0;  // m: the four cones stand 1 m apart along the track
    std::size_t best = 0;
    std::size_t best_count = 0;
    for (std::size_t i = 0; i < s.size(); ++i) {
        std::size_t count = 0;
        for (const double other : s) {
            if (std::abs(reference.delta_s(s[i], other)) <= kCluster) ++count;
        }
        if (count > best_count) {
            best = i;
            best_count = count;
        }
    }
    double sum = 0.0;
    for (const double other : s) {
        const double d = reference.delta_s(s[best], other);
        if (std::abs(d) <= kCluster) sum += d;
    }
    return reference.wrap_s(s[best] + sum / static_cast<double>(best_count));
}

}  // namespace

std::optional<TrackModel> build_track_model(const ConeMap& map, std::span<const Pose2> driven_path,
                                            const TrackModelConfig& config) {
    check_config(config);
    const std::optional<std::vector<Vec2>> loop = driven_loop(driven_path, config);
    if (!loop) return fail("the driven path does not close a lap");
    Trajectory reference = Trajectory::from_waypoints(*loop, true, config.reference_spacing);

    Sides sides;
    std::optional<Profile> profile;
    for (std::size_t pass = 0;; ++pass) {
        sides = place_cones(map, reference, config);
        if (sides.left.size() < config.min_cones_per_side ||
            sides.right.size() < config.min_cones_per_side) {
            return fail("too few cones on a side");
        }
        profile = estimate_profile(reference, sides, config);
        if (!profile) return fail("the cones do not determine a track");
        if (pass == config.passes) break;
        reference = Trajectory::from_waypoints(centerline_points(reference, *profile), true,
                                               config.reference_spacing);
    }

    TrackModel model;
    std::optional<std::vector<Vec2>> left = cone_line(sides.left, 1.0, reference, *profile, config);
    std::optional<std::vector<Vec2>> right =
        cone_line(sides.right, -1.0, reference, *profile, config);
    if (!left || !right) return fail("a gap between cones is too long");
    model.left = std::move(*left);
    model.right = std::move(*right);

    // The centerline starts on the timekeeping line.
    const Trajectory center = Trajectory::from_waypoints(centerline_points(reference, *profile),
                                                         true, config.reference_spacing);
    const double length = center.length();
    const double start = timing_line_s(map, center, loop->front(), config);
    const auto count = static_cast<std::size_t>(std::round(length / config.reference_spacing));
    std::vector<Vec2> waypoints;
    waypoints.reserve(count);
    for (std::size_t k = 0; k < count; ++k) {
        waypoints.push_back(
            center.at(start + length * static_cast<double>(k) / static_cast<double>(count))
                .position);
    }
    model.centerline = Trajectory::from_waypoints(waypoints, true, config.centerline_spacing);

    std::vector<Vec2> centerline;
    std::vector<double> width_left;
    std::vector<double> width_right;
    for (const TrajectoryPoint& p : model.centerline.points()) {
        centerline.push_back(p.position);
        width_left.push_back(distance_to_polyline(p.position, model.left, true));
        width_right.push_back(distance_to_polyline(p.position, model.right, true));
    }
    model.centerline.set_widths(width_left, width_right);

    if (polyline_self_intersects(model.left, true) || polyline_self_intersects(model.right, true) ||
        polylines_intersect(model.left, true, model.right, true)) {
        return fail("the cone lines cross");
    }
    if (polylines_intersect(centerline, true, model.left, true) ||
        polylines_intersect(centerline, true, model.right, true)) {
        return fail("the centerline leaves the cone lines");
    }
    return model;
}

}  // namespace apex::planning
