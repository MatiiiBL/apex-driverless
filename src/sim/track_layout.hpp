#pragma once

#include <variant>
#include <vector>

#include "apex/core/types.hpp"

#include "track_profile.hpp"

// Circuits described by their curvature profile: straights, circular arcs and the clothoids
// (curvature linear in arc length) that join them, as road and railway alignments are laid out.
// Clothoid transitions keep the curvature continuous, so the spline that track_from_centerline
// fits through the samples reproduces the designed radii instead of overshooting at each corner
// entry. Shared by the random generator and the named reference layouts.
namespace apex::sim::detail {

// A turn of signed heading change `angle` (rad, positive left) at `radius` (m), entered and left
// through clothoids. A scalable turn may have its angle scaled to close the loop's heading, as
// long as it stays within [min_angle, max_angle] in magnitude.
struct TurnSpec {
    double angle = 0.0;
    double radius = 0.0;
    bool scalable = false;
    double min_angle = 0.0;
    double max_angle = 0.0;
};

// A straight. An adjustable one may be lengthened or shortened within [min_length, max_length]
// to close the loop's position.
struct StraightSpec {
    double length = 0.0;
    bool adjustable = false;
    double min_length = 0.0;
    double max_length = 0.0;
};

using Element = std::variant<TurnSpec, StraightSpec>;

// Spacing of the waypoints a layout hands to track_from_centerline (m). The spline through them
// matches the layout's curvature to well under 1%.
inline constexpr double kWaypointSpacing = 1.0;

// Length of each clothoid that enters or leaves a turn: half the radius, or less for a shallow
// turn, which then still keeps most of its angle on the arc.
[[nodiscard]] double transition_length(double angle, double radius);

// A piece of centerline whose curvature varies linearly from k0 to k1 along its length.
struct Piece {
    double length = 0.0;
    double k0 = 0.0;
    double k1 = 0.0;
    int adjustable = -1;  // index into the adjustable straights, or -1
};

// A path from the origin, heading +x, through the pieces in order. Positions are integrated with
// Gauss-Legendre quadrature on chunks of at most 1 m, accurate to ~1e-12 m.
class Layout {
public:
    explicit Layout(const std::vector<Element>& elements);

    [[nodiscard]] double length() const;
    [[nodiscard]] double total_turning() const;  // rad
    [[nodiscard]] Vec2 end_point() const;        // the origin when the loop closes

    // Changes the adjustable straights' lengths, by as little as possible in the least-squares
    // sense and within their bounds, so the path ends at the origin. The heading must already
    // close (total_turning() = +/-2 pi): heading does not depend on straight lengths, which makes
    // the end point linear in them. Returns false if the bounds leave no solution.
    bool close();

    // Samples every ~`spacing` meters (evenly dividing the length), from s = 0.
    [[nodiscard]] std::vector<ProfileSample> sample(double spacing, double half_width) const;

private:
    struct Bounds {
        double min_length;
        double max_length;
    };
    [[nodiscard]] std::vector<double> start_headings() const;

    std::vector<Piece> pieces_;
    std::vector<Bounds> bounds_;  // per adjustable straight
};

}  // namespace apex::sim::detail
