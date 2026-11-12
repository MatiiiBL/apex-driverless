#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "apex/core/types.hpp"

namespace apex {

// One sample of a path, with the reference speed profile and the free space beside it.
struct TrajectoryPoint {
    double s = 0.0;  // arc length from the first point (m)
    Vec2 position = Vec2::Zero();
    double yaw = 0.0;          // path heading (rad)
    double curvature = 0.0;    // signed, 1/m, positive turning left
    double speed = 0.0;        // reference speed (m/s)
    double accel = 0.0;        // reference longitudinal acceleration (m/s^2)
    double width_left = 0.0;   // free lateral distance from the path to the left boundary (m)
    double width_right = 0.0;  // free lateral distance from the path to the right boundary (m)
};

// A densely sampled path: a planner's output, a controller's reference, a track's centerline.
// Between samples every field is interpolated linearly (yaw along the shorter arc). A closed
// trajectory connects its last point back to the first; the first point is not repeated.
class Trajectory {
public:
    Trajectory() = default;
    // Recomputes s from the positions. Throws std::invalid_argument for fewer than 2 points (3 if
    // closed), repeated consecutive points, or non-finite values.
    Trajectory(std::vector<TrajectoryPoint> points, bool closed);

    // Fits a Spline2D through the waypoints and samples it every ~`spacing` meters. Yaw and
    // curvature come from the spline; speed, accel and widths are zero.
    [[nodiscard]] static Trajectory from_waypoints(std::span<const Vec2> waypoints, bool closed,
                                                   double spacing);

    [[nodiscard]] bool empty() const { return points_.empty(); }
    [[nodiscard]] std::size_t size() const { return points_.size(); }
    [[nodiscard]] bool closed() const { return closed_; }
    // Total length; for a closed trajectory it includes the closing segment.
    [[nodiscard]] double length() const { return length_; }
    [[nodiscard]] const std::vector<TrajectoryPoint>& points() const { return points_; }
    [[nodiscard]] const TrajectoryPoint& operator[](std::size_t i) const { return points_[i]; }

    // Replaces the speed profile (and optionally accel); sizes must equal size().
    void set_speed_profile(std::span<const double> speed, std::span<const double> accel = {});
    // Replaces the free-space widths; sizes must equal size().
    void set_widths(std::span<const double> left, std::span<const double> right);

    // Closed: s wrapped into [0, length). Open: s clamped to [0, length].
    [[nodiscard]] double wrap_s(double s) const;
    // Signed distance along the path from s_from to s_to. Closed: the shorter way around.
    [[nodiscard]] double delta_s(double s_from, double s_to) const;

    // The interpolated sample at arc length s (wrapped or clamped as wrap_s).
    [[nodiscard]] TrajectoryPoint at(double s) const;

    struct Projection {
        double s = 0.0;        // arc length of the closest point on the path
        double lateral = 0.0;  // signed distance from the path, positive to the left
        double distance = 0.0;
        std::size_t segment = 0;  // index of the segment's first point
    };
    // Closest point over the whole path.
    [[nodiscard]] Projection project(const Vec2& p) const;
    // Closest point among segments overlapping [s_hint - window, s_hint + window]: cheaper, and
    // keeps a tracker from jumping to a nearby part of the track that loops back.
    [[nodiscard]] Projection project(const Vec2& p, double s_hint, double window) const;

private:
    [[nodiscard]] std::size_t segment_count() const;
    [[nodiscard]] std::size_t segment_at(double s) const;  // s already wrapped/clamped
    [[nodiscard]] Projection project_segment(const Vec2& p, std::size_t segment) const;

    std::vector<TrajectoryPoint> points_;
    double length_ = 0.0;
    bool closed_ = false;
};

}  // namespace apex
