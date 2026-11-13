#include "apex/stack/braking.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

#include "apex/core/geometry.hpp"

namespace apex::stack {

namespace {

constexpr double kSpacing = 0.5;  // m between samples of the braking reference
// Any car brakes harder than this, so a stop from `speed` fits in speed^2 / (2 kMinDecel).
constexpr double kMinDecel = 1.0;            // m/s^2
constexpr double kSpeedTolerance = 1e-3;     // m/s
constexpr double kDistanceTolerance = 0.05;  // m, of the stopping distance found
constexpr double kMinStep = 0.01;            // m: a shorter last step is dropped

// The point at arc length `s` of `path`; past the end of an open path, straight on along its
// last heading.
TrajectoryPoint point_at(const Trajectory& path, double s) {
    if (path.closed() || s <= path.length()) return path.at(s);
    TrajectoryPoint p = path.at(path.length());
    p.position += (s - path.length()) * heading_vector(p.yaw);
    p.curvature = 0.0;
    return p;
}

// `length` of `path` from arc length `start` on: samples every kSpacing, and one exactly at the
// end.
std::vector<TrajectoryPoint> sample(const Trajectory& path, double start, double length) {
    std::vector<TrajectoryPoint> points;
    points.reserve(static_cast<std::size_t>(length / kSpacing) + 2);
    for (double d = 0.0; d < length - kMinStep; d += kSpacing) {
        points.push_back(point_at(path, start + d));
    }
    points.push_back(point_at(path, start + length));
    return points;
}

}  // namespace

Trajectory braking_reference(const Trajectory& path, double s_start, double speed,
                             const VehicleParams& nominal,
                             const planning::SpeedProfileConfig& envelope, double margin) {
    if (path.empty() || !(speed >= 0.0) || !(margin >= 0.0)) {
        throw std::invalid_argument("braking_reference: needs a path, speed >= 0, margin >= 0");
    }
    const double start = path.wrap_s(s_start);
    // The envelope's fastest profile over `length` that starts at `speed` and ends at rest.
    const auto stop_within = [&](double length) {
        Trajectory t(sample(path, start, length), false);
        planning::apply_speed_profile(t, nominal, envelope, speed, 0.0);
        return t;
    };

    // Over a long enough stretch the profile starts at the car's speed (or at the cornering
    // speed where the car is, if it is beyond it). The shortest stretch over which it still
    // does is the shortest stop; its start speed falls as the stretch shrinks, so bisect.
    double hi = speed * speed / (2.0 * kMinDecel) + 2.0 * kSpacing;
    double lo = 2.0 * kSpacing;
    const double reachable = stop_within(hi)[0].speed;
    while (hi - lo > kDistanceTolerance) {
        const double mid = 0.5 * (lo + hi);
        (stop_within(mid)[0].speed >= reachable - kSpeedTolerance ? hi : lo) = mid;
    }
    std::vector<TrajectoryPoint> points = stop_within(hi).points();
    // A stop never speeds up: below the envelope's speeds the car holds its own until the braking
    // curve meets it, which keeps the profile feasible (the curve brakes from any speed on it).
    for (TrajectoryPoint& p : points) p.speed = std::min(p.speed, speed);
    points.back().speed = 0.0;
    std::vector<TrajectoryPoint> hold = sample(path, start + points.back().s, margin);
    for (std::size_t k = 1; k < hold.size(); ++k) {
        hold[k].speed = 0.0;
        points.push_back(hold[k]);
    }
    // The acceleration that takes each point's speed to the next one's: a = d(v^2)/ds / 2.
    for (std::size_t k = 0; k + 1 < points.size(); ++k) {
        const double v0 = points[k].speed;
        const double v1 = points[k + 1].speed;
        const double ds = (points[k + 1].position - points[k].position).norm();
        points[k].accel = (v1 * v1 - v0 * v0) / (2.0 * ds);
    }
    points.back().accel = 0.0;
    return Trajectory(std::move(points), false);
}

}  // namespace apex::stack
