#include "apex/control/reference.hpp"

#include <algorithm>
#include <cmath>

#include "apex/core/geometry.hpp"

namespace apex::control {

namespace {

// The local search window: at least this far either side of the expected station (m).
constexpr double kMinWindow = 4.0;
// A local result farther than this from the path triggers a global search (m).
constexpr double kRelocateDistance = 3.0;

TrajectoryPoint extrapolate(const TrajectoryPoint& end, double along) {
    TrajectoryPoint p = end;
    p.position = end.position + along * heading_vector(end.yaw);
    p.curvature = 0.0;
    p.accel = 0.0;
    return p;
}

}  // namespace

TrajectoryPoint sample_reference(const Trajectory& reference, double s) {
    if (!reference.closed()) {
        if (s > reference.length()) {
            TrajectoryPoint p = extrapolate(reference.points().back(), s - reference.length());
            p.s = s;
            return p;
        }
        if (s < 0.0) {
            TrajectoryPoint p = extrapolate(reference.points().front(), s);
            p.s = s;
            return p;
        }
    }
    return reference.at(s);
}

PathCoordinates extend_projection(const Trajectory& reference, const Vec2& p,
                                  const Trajectory::Projection& projection) {
    PathCoordinates out{projection.s, projection.lateral, projection.distance};
    if (reference.closed()) return out;
    constexpr double kEndTolerance = 1e-9;
    const bool at_end = projection.s >= reference.length() - kEndTolerance;
    const bool at_start = projection.s <= kEndTolerance;
    if (!at_end && !at_start) return out;
    const TrajectoryPoint& end = at_end ? reference.points().back() : reference.points().front();
    const Vec2 tangent = heading_vector(end.yaw);
    const Vec2 offset = p - end.position;
    const double along = offset.dot(tangent);
    if ((at_end && along > 0.0) || (at_start && along < 0.0)) {
        out.s = (at_end ? reference.length() : 0.0) + along;
        out.lateral = cross(tangent, offset);
        out.distance = std::abs(out.lateral);
    }
    return out;
}

PathCoordinates ReferenceTracker::locate(const Trajectory& reference, const Vec2& p,
                                         double travel) {
    const Signature signature{reference.size(), reference.closed(), reference.length(),
                              reference.points().front().position,
                              reference.points().back().position};
    changed_ = !signature_.has_value() || !(*signature_ == signature);
    signature_ = signature;
    Trajectory::Projection projection;
    if (changed_ || !s_.has_value()) {
        projection = reference.project(p);
    } else {
        const double hint = reference.wrap_s(*s_);
        projection = reference.project(p, hint, kMinWindow + std::max(travel, 0.0));
        if (projection.distance > kRelocateDistance) {
            const Trajectory::Projection global = reference.project(p);
            if (global.distance < projection.distance) projection = global;
        }
    }
    s_ = projection.s;
    return extend_projection(reference, p, projection);
}

void ReferenceTracker::reset() {
    signature_.reset();
    s_.reset();
    changed_ = true;
}

}  // namespace apex::control
