#pragma once

#include <optional>
#include <span>
#include <vector>

#include "apex/core/types.hpp"

// Planar geometry on points, segments, polylines and simple polygons. Polylines and polygons are
// point lists; a closed one connects its last point back to the first without repeating it.
namespace apex {

// z component of the 3D cross product: positive when b is counter-clockwise from a.
[[nodiscard]] inline double cross(const Vec2& a, const Vec2& b) {
    return a.x() * b.y() - a.y() * b.x();
}

// The unit vector 90 degrees counter-clockwise (to the left) of the unit vector `direction`.
[[nodiscard]] inline Vec2 left_normal(const Vec2& direction) {
    return {-direction.y(), direction.x()};
}

// The unit vector at heading `yaw`.
[[nodiscard]] Vec2 heading_vector(double yaw);

struct SegmentProjection {
    double t = 0.0;  // in [0, 1]: closest point = a + t (b - a)
    Vec2 point = Vec2::Zero();
    double distance = 0.0;
};
// The point of segment ab closest to p. A degenerate segment (a == b) projects to a.
[[nodiscard]] SegmentProjection project_on_segment(const Vec2& p, const Vec2& a, const Vec2& b);

// The intersection of segments a0-a1 and b0-b1, or nothing if they do not meet. Collinear
// overlapping segments report one point of the overlap.
[[nodiscard]] std::optional<Vec2> segment_intersection(const Vec2& a0, const Vec2& a1,
                                                       const Vec2& b0, const Vec2& b1);

// Cumulative length at each point: result[0] = 0, result[i] = length of points[0..i]. For a
// closed polyline the result has one extra entry, the total length including the closing segment.
[[nodiscard]] std::vector<double> cumulative_length(std::span<const Vec2> points, bool closed);

// Signed area of a polygon: positive when the points run counter-clockwise.
[[nodiscard]] double polygon_signed_area(std::span<const Vec2> polygon);

// Even-odd rule. Points exactly on the boundary may land either way.
[[nodiscard]] bool point_in_polygon(const Vec2& p, std::span<const Vec2> polygon);

// True if any two non-adjacent segments of the polyline intersect. O(n^2): for validation and
// tests, not for hot paths.
[[nodiscard]] bool polyline_self_intersects(std::span<const Vec2> points, bool closed);

// True if any segment of `a` intersects any segment of `b`. O(n m), for validation and tests.
[[nodiscard]] bool polylines_intersect(std::span<const Vec2> a, bool a_closed,
                                       std::span<const Vec2> b, bool b_closed);

// Distance from p to the nearest point of the polyline.
[[nodiscard]] double distance_to_polyline(const Vec2& p, std::span<const Vec2> points, bool closed);

// Signed curvature (1/m, positive turning left) of the circle through three points; 0 for
// collinear points.
[[nodiscard]] double menger_curvature(const Vec2& a, const Vec2& b, const Vec2& c);

}  // namespace apex
