#pragma once

#include <cstddef>
#include <optional>

#include "apex/core/trajectory.hpp"
#include "apex/core/types.hpp"

// A controller's view of its reference trajectory. Open references (the exploration path) end
// where the car must have stopped; a controller that reaches or overshoots the end still needs
// a well-defined error, so an open reference is continued straight along its end tangents, at
// its end speed, zero curvature and its end widths.
namespace apex::control {

// The reference at arc length s, extended beyond the ends of an open reference.
[[nodiscard]] TrajectoryPoint sample_reference(const Trajectory& reference, double s);

// A point in path coordinates: station s (beyond [0, length] past the ends of an open
// reference; within [0, length) on a closed one) and lateral offset, positive to the left.
struct PathCoordinates {
    double s = 0.0;
    double lateral = 0.0;
    double distance = 0.0;  // to the (extended) path
};

// The path coordinates of p from a projection onto the reference's polyline, continuing past
// the ends of an open reference. Inverse of sample_reference: p = sample(s).position +
// lateral * left normal.
[[nodiscard]] PathCoordinates extend_projection(const Trajectory& reference, const Vec2& p,
                                                const Trajectory::Projection& projection);

// Follows a moving point (the car, one of its axles) along a reference across calls: a local
// projection around its last station while the reference stays the same, which is cheap and
// cannot jump to a nearby part of a track that loops back; a global one when the reference has
// changed (a replanned path, the switch to the raceline) or the point is far from where the
// local search looked.
class ReferenceTracker {
public:
    // The coordinates of p on `reference`. `travel` bounds how far p can have moved along the
    // path since the previous call (m).
    PathCoordinates locate(const Trajectory& reference, const Vec2& p, double travel);
    // Whether the last locate() saw a different reference from the call before.
    [[nodiscard]] bool reference_changed() const { return changed_; }
    void reset();

private:
    // A cheap fingerprint: a replanned or switched reference differs in at least one of these.
    struct Signature {
        std::size_t size = 0;
        bool closed = false;
        double length = 0.0;
        Vec2 first = Vec2::Zero();
        Vec2 last = Vec2::Zero();
        bool operator==(const Signature&) const = default;
    };

    std::optional<Signature> signature_;
    std::optional<double> s_;
    bool changed_ = true;
};

}  // namespace apex::control
