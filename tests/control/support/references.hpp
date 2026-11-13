#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/trajectory.hpp"

// Reference trajectories for the control tests and benchmarks: straight lines and circles with
// exact geometry, and a speed profile for any path. The planning module's speed profile is
// richer (the full g-g-v envelope); this one is the simple friction-circle version a test
// needs: a lateral limit sets the speed in corners, and forward and backward passes cap the
// acceleration and braking between them.
namespace apex::control::testing {

inline constexpr double kHalfWidth = 1.75;  // m, the track's centerline to its cone lines

// A straight path along `heading` from `start`, sampled every `spacing` meters, at constant
// `speed`, with the standard track widths.
[[nodiscard]] inline Trajectory straight_reference(const Vec2& start, double heading, double length,
                                                   double speed, double spacing = 0.5) {
    const auto count = static_cast<std::size_t>(std::round(length / spacing)) + 1;
    std::vector<TrajectoryPoint> points(count);
    const Vec2 direction = heading_vector(heading);
    for (std::size_t i = 0; i < count; ++i) {
        TrajectoryPoint& p = points[i];
        p.position = start + (static_cast<double>(i) * spacing) * direction;
        p.yaw = heading;
        p.speed = speed;
        p.width_left = kHalfWidth;
        p.width_right = kHalfWidth;
    }
    return Trajectory(std::move(points), false);
}

// A closed circle around `center`, driven counter-clockwise from its lowest point, at constant
// `speed`.
[[nodiscard]] inline Trajectory circle_reference(const Vec2& center, double radius, double speed,
                                                 double spacing = 0.5) {
    const auto count = static_cast<std::size_t>(std::round(kTwoPi * radius / spacing));
    std::vector<TrajectoryPoint> points(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double angle =
            -0.5 * kPi + kTwoPi * static_cast<double>(i) / static_cast<double>(count);
        TrajectoryPoint& p = points[i];
        p.position = center + radius * heading_vector(angle);
        p.yaw = wrap_angle(angle + 0.5 * kPi);
        p.curvature = 1.0 / radius;
        p.speed = speed;
        p.width_left = kHalfWidth;
        p.width_right = kHalfWidth;
    }
    return Trajectory(std::move(points), true);
}

struct SpeedLimits {
    double lateral = 12.0;  // m/s^2
    double accel = 6.0;     // m/s^2
    double brake = 10.0;    // m/s^2
    double top = 25.0;      // m/s
    // Open paths: the speed at the first and last points (the exploration path ends at rest).
    double start = 0.0;
    double end = 0.0;
};

// Replaces the speed profile of `path` with the fastest one within `limits`: v^2 kappa below
// the lateral limit, and the forward and backward passes of v^2 = v0^2 + 2 a ds. Curvature is
// smoothed over a few meters first, since a centerline's sampled curvature is noisy.
inline void apply_speed_limits(Trajectory& path, const SpeedLimits& limits) {
    const std::vector<TrajectoryPoint>& pts = path.points();
    const std::size_t n = pts.size();
    const bool closed = path.closed();
    auto next = [&](std::size_t i) { return (i + 1) % n; };
    auto gap = [&](std::size_t i) {  // arc length from point i to the next
        return i + 1 < n ? pts[i + 1].s - pts[i].s : path.length() - pts[i].s;
    };
    constexpr int kSmoothing = 4;  // points either side
    std::vector<double> v2(n);
    for (std::size_t i = 0; i < n; ++i) {
        double sum = 0.0;
        int used = 0;
        for (int d = -kSmoothing; d <= kSmoothing; ++d) {
            const long j = static_cast<long>(i) + d;
            const long m = static_cast<long>(n);
            if (!closed && (j < 0 || j >= m)) continue;
            sum += std::abs(pts[static_cast<std::size_t>(((j % m) + m) % m)].curvature);
            ++used;
        }
        const double kappa = sum / used;
        const double corner = kappa > 1e-6 ? limits.lateral / kappa : limits.top * limits.top;
        v2[i] = std::min(corner, limits.top * limits.top);
    }
    if (!closed) {
        v2.front() = std::min(v2.front(), limits.start * limits.start);
        v2.back() = std::min(v2.back(), limits.end * limits.end);
    }
    // Closed paths go around twice so the passes carry across the start.
    const std::size_t passes = closed ? 2 * n : n - 1;
    for (std::size_t k = 0; k < passes; ++k) {
        const std::size_t i = k % n;
        if (!closed && i + 1 >= n) continue;
        const std::size_t j = next(i);
        v2[j] = std::min(v2[j], v2[i] + 2.0 * limits.accel * gap(i));
    }
    for (std::size_t k = passes; k-- > 0;) {
        const std::size_t i = k % n;
        if (!closed && i + 1 >= n) continue;
        const std::size_t j = next(i);
        v2[i] = std::min(v2[i], v2[j] + 2.0 * limits.brake * gap(i));
    }
    std::vector<double> speed(n);
    std::vector<double> accel(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) speed[i] = std::sqrt(std::max(v2[i], 0.0));
    for (std::size_t i = 0; i < n; ++i) {
        if (!closed && i + 1 >= n) break;
        accel[i] = (v2[next(i)] - v2[i]) / (2.0 * gap(i));
    }
    if (!closed && n > 1) accel[n - 1] = accel[n - 2];
    path.set_speed_profile(speed, accel);
}

}  // namespace apex::control::testing
