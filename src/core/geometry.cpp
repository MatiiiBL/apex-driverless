#include "apex/core/geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace apex {

Vec2 heading_vector(double yaw) {
    return {std::cos(yaw), std::sin(yaw)};
}

SegmentProjection project_on_segment(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const double length_sq = ab.squaredNorm();
    double t = 0.0;
    if (length_sq > 0.0) t = std::clamp((p - a).dot(ab) / length_sq, 0.0, 1.0);
    const Vec2 point = a + t * ab;
    return {t, point, (p - point).norm()};
}

std::optional<Vec2> segment_intersection(const Vec2& a0, const Vec2& a1, const Vec2& b0,
                                         const Vec2& b1) {
    const Vec2 r = a1 - a0;
    const Vec2 s = b1 - b0;
    const double denom = cross(r, s);
    const Vec2 qp = b0 - a0;
    constexpr double kEps = 1e-12;
    if (std::abs(denom) < kEps * std::max(1.0, r.squaredNorm() * s.squaredNorm())) {
        // Parallel. Collinear only if b0 lies on the line through a.
        if (std::abs(cross(qp, r)) > kEps * std::max(1.0, r.squaredNorm())) return std::nullopt;
        const double rr = r.squaredNorm();
        if (rr == 0.0) {
            if (project_on_segment(a0, b0, b1).distance < 1e-12) return a0;
            return std::nullopt;
        }
        const double t0 = qp.dot(r) / rr;
        const double t1 = t0 + s.dot(r) / rr;
        const double lo = std::max(0.0, std::min(t0, t1));
        const double hi = std::min(1.0, std::max(t0, t1));
        if (lo > hi) return std::nullopt;
        return a0 + lo * r;
    }
    const double t = cross(qp, s) / denom;
    const double u = cross(qp, r) / denom;
    if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) return std::nullopt;
    return a0 + t * r;
}

std::vector<double> cumulative_length(std::span<const Vec2> points, bool closed) {
    std::vector<double> result;
    if (points.empty()) return result;
    result.reserve(points.size() + 1);
    result.push_back(0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        result.push_back(result.back() + (points[i] - points[i - 1]).norm());
    }
    if (closed) result.push_back(result.back() + (points.front() - points.back()).norm());
    return result;
}

double polygon_signed_area(std::span<const Vec2> polygon) {
    double twice_area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        twice_area += cross(polygon[i], polygon[(i + 1) % polygon.size()]);
    }
    return 0.5 * twice_area;
}

bool point_in_polygon(const Vec2& p, std::span<const Vec2> polygon) {
    bool inside = false;
    const std::size_t n = polygon.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = polygon[i];
        const Vec2& b = polygon[j];
        if ((a.y() > p.y()) != (b.y() > p.y())) {
            const double x_cross = a.x() + (p.y() - a.y()) * (b.x() - a.x()) / (b.y() - a.y());
            if (p.x() < x_cross) inside = !inside;
        }
    }
    return inside;
}

namespace {

std::size_t segment_count(std::size_t points, bool closed) {
    if (points < 2) return 0;
    return closed ? points : points - 1;
}

}  // namespace

bool polyline_self_intersects(std::span<const Vec2> points, bool closed) {
    const std::size_t n = points.size();
    const std::size_t segments = segment_count(n, closed);
    for (std::size_t i = 0; i < segments; ++i) {
        for (std::size_t j = i + 2; j < segments; ++j) {
            // In a closed polyline the last segment is adjacent to the first.
            if (closed && i == 0 && j == segments - 1) continue;
            if (segment_intersection(points[i], points[(i + 1) % n], points[j],
                                     points[(j + 1) % n])) {
                return true;
            }
        }
    }
    return false;
}

bool polylines_intersect(std::span<const Vec2> a, bool a_closed, std::span<const Vec2> b,
                         bool b_closed) {
    const std::size_t na = segment_count(a.size(), a_closed);
    const std::size_t nb = segment_count(b.size(), b_closed);
    for (std::size_t i = 0; i < na; ++i) {
        const Vec2& a0 = a[i];
        const Vec2& a1 = a[(i + 1) % a.size()];
        for (std::size_t j = 0; j < nb; ++j) {
            if (segment_intersection(a0, a1, b[j], b[(j + 1) % b.size()])) return true;
        }
    }
    return false;
}

double distance_to_polyline(const Vec2& p, std::span<const Vec2> points, bool closed) {
    if (points.size() == 1) return (p - points.front()).norm();
    double best = std::numeric_limits<double>::infinity();
    const std::size_t segments = segment_count(points.size(), closed);
    for (std::size_t i = 0; i < segments; ++i) {
        best = std::min(best,
                        project_on_segment(p, points[i], points[(i + 1) % points.size()]).distance);
    }
    return best;
}

double menger_curvature(const Vec2& a, const Vec2& b, const Vec2& c) {
    const double ab = (b - a).norm();
    const double bc = (c - b).norm();
    const double ca = (a - c).norm();
    const double denom = ab * bc * ca;
    if (denom == 0.0) return 0.0;
    return 2.0 * cross(b - a, c - b) / denom;
}

}  // namespace apex
