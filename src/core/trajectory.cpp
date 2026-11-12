#include "apex/core/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/spline.hpp"

namespace apex {

namespace {

bool finite(const TrajectoryPoint& p) {
    return std::isfinite(p.position.x()) && std::isfinite(p.position.y()) && std::isfinite(p.yaw) &&
           std::isfinite(p.curvature) && std::isfinite(p.speed) && std::isfinite(p.accel) &&
           std::isfinite(p.width_left) && std::isfinite(p.width_right);
}

}  // namespace

Trajectory::Trajectory(std::vector<TrajectoryPoint> points, bool closed)
    : points_(std::move(points)), closed_(closed) {
    const std::size_t min_points = closed_ ? 3 : 2;
    if (points_.size() < min_points) throw std::invalid_argument("Trajectory: too few points");
    double s = 0.0;
    for (std::size_t i = 0; i < points_.size(); ++i) {
        if (!finite(points_[i])) throw std::invalid_argument("Trajectory: non-finite point");
        if (i > 0) {
            const double step = (points_[i].position - points_[i - 1].position).norm();
            if (!(step > 1e-9)) throw std::invalid_argument("Trajectory: repeated points");
            s += step;
        }
        points_[i].s = s;
    }
    length_ = s;
    if (closed_) {
        const double closing = (points_.front().position - points_.back().position).norm();
        if (!(closing > 1e-9)) {
            throw std::invalid_argument("Trajectory: closed path repeats its first point");
        }
        length_ += closing;
    }
}

Trajectory Trajectory::from_waypoints(std::span<const Vec2> waypoints, bool closed,
                                      double spacing) {
    const Spline2D spline(waypoints, closed);
    std::vector<TrajectoryPoint> points;
    for (const Spline2D::Sample& sample : spline.resample(spacing)) {
        TrajectoryPoint point;
        point.position = sample.position;
        point.yaw = sample.heading;
        point.curvature = sample.curvature;
        points.push_back(point);
    }
    return Trajectory(std::move(points), closed);
}

void Trajectory::set_speed_profile(std::span<const double> speed, std::span<const double> accel) {
    if (speed.size() != points_.size() || (!accel.empty() && accel.size() != points_.size())) {
        throw std::invalid_argument("Trajectory::set_speed_profile: size mismatch");
    }
    for (std::size_t i = 0; i < points_.size(); ++i) {
        points_[i].speed = speed[i];
        if (!accel.empty()) points_[i].accel = accel[i];
    }
}

void Trajectory::set_widths(std::span<const double> left, std::span<const double> right) {
    if (left.size() != points_.size() || right.size() != points_.size()) {
        throw std::invalid_argument("Trajectory::set_widths: size mismatch");
    }
    for (std::size_t i = 0; i < points_.size(); ++i) {
        points_[i].width_left = left[i];
        points_[i].width_right = right[i];
    }
}

double Trajectory::wrap_s(double s) const {
    if (points_.empty()) return 0.0;
    if (!closed_) return std::clamp(s, 0.0, length_);
    double wrapped = std::fmod(s, length_);
    if (wrapped < 0.0) wrapped += length_;
    if (wrapped >= length_) wrapped = 0.0;
    return wrapped;
}

double Trajectory::delta_s(double s_from, double s_to) const {
    const double d = s_to - s_from;
    if (!closed_) return d;
    double wrapped = std::remainder(d, length_);  // [-L/2, L/2]
    if (wrapped <= -0.5 * length_) wrapped += length_;
    return wrapped;
}

std::size_t Trajectory::segment_count() const {
    if (points_.size() < 2) return 0;
    return closed_ ? points_.size() : points_.size() - 1;
}

std::size_t Trajectory::segment_at(double s) const {
    const auto it =
        std::upper_bound(points_.begin(), points_.end(), s,
                         [](double value, const TrajectoryPoint& p) { return value < p.s; });
    const auto index = static_cast<std::size_t>(std::distance(points_.begin(), it));
    return std::min(index == 0 ? 0 : index - 1, segment_count() - 1);
}

TrajectoryPoint Trajectory::at(double s) const {
    if (points_.empty()) throw std::logic_error("Trajectory::at on an empty trajectory");
    s = wrap_s(s);
    const std::size_t i = segment_at(s);
    const TrajectoryPoint& a = points_[i];
    const TrajectoryPoint& b = points_[(i + 1) % points_.size()];
    const double end_s = (i + 1 == points_.size()) ? length_ : b.s;
    const double span = end_s - a.s;
    const double u = span > 0.0 ? std::clamp((s - a.s) / span, 0.0, 1.0) : 0.0;
    auto lerp = [u](double x, double y) { return x + u * (y - x); };
    TrajectoryPoint result;
    result.s = s;
    result.position = a.position + u * (b.position - a.position);
    result.yaw = wrap_angle(a.yaw + u * angle_diff(b.yaw, a.yaw));
    result.curvature = lerp(a.curvature, b.curvature);
    result.speed = lerp(a.speed, b.speed);
    result.accel = lerp(a.accel, b.accel);
    result.width_left = lerp(a.width_left, b.width_left);
    result.width_right = lerp(a.width_right, b.width_right);
    return result;
}

Trajectory::Projection Trajectory::project_segment(const Vec2& p, std::size_t segment) const {
    const TrajectoryPoint& a = points_[segment];
    const TrajectoryPoint& b = points_[(segment + 1) % points_.size()];
    const SegmentProjection proj = project_on_segment(p, a.position, b.position);
    const double seg_length = (b.position - a.position).norm();
    const double side = cross(b.position - a.position, p - proj.point);
    Projection result;
    result.s = wrap_s(a.s + proj.t * seg_length);
    result.distance = proj.distance;
    result.lateral = side >= 0.0 ? proj.distance : -proj.distance;
    result.segment = segment;
    return result;
}

Trajectory::Projection Trajectory::project(const Vec2& p) const {
    if (points_.empty()) throw std::logic_error("Trajectory::project on an empty trajectory");
    Projection best;
    best.distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < segment_count(); ++i) {
        const Projection candidate = project_segment(p, i);
        if (candidate.distance < best.distance) best = candidate;
    }
    return best;
}

Trajectory::Projection Trajectory::project(const Vec2& p, double s_hint, double window) const {
    if (points_.empty()) throw std::logic_error("Trajectory::project on an empty trajectory");
    if (window * 2.0 >= length_) return project(p);
    const double start =
        closed_ ? wrap_s(s_hint - window) : std::clamp(s_hint - window, 0.0, length_);
    const double span = closed_ ? 2.0 * window : std::clamp(s_hint + window, 0.0, length_) - start;
    const std::size_t segments = segment_count();
    std::size_t segment = segment_at(start);
    // Path length walked past `start` at the end of the current segment.
    double walked = points_[segment].s - start;
    Projection best;
    best.distance = std::numeric_limits<double>::infinity();
    for (std::size_t visited = 0; visited < segments; ++visited) {
        const Projection candidate = project_segment(p, segment);
        if (candidate.distance < best.distance) best = candidate;
        const double end_s = (segment + 1 == points_.size()) ? length_ : points_[segment + 1].s;
        walked += end_s - points_[segment].s;
        if (walked >= span) break;
        if (!closed_ && segment + 1 == segments) break;
        segment = (segment + 1) % segments;
    }
    return best;
}

}  // namespace apex
