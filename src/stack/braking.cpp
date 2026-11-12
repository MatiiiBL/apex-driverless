#include "apex/stack/braking.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "apex/core/geometry.hpp"

namespace apex::stack {

namespace {

constexpr double kSpacing = 0.5;  // m between samples of the braking reference

}  // namespace

Trajectory braking_reference(const Trajectory& path, double s_start, double speed, double decel,
                             double margin) {
    if (!(decel > 0.0) || !(speed >= 0.0) || !(margin >= 0.0) || path.empty()) {
        throw std::invalid_argument("braking_reference: needs a path, decel > 0, speed >= 0");
    }
    const double stop_distance = speed * speed / (2.0 * decel);
    const double length = std::max(stop_distance + margin, 2.0 * kSpacing);
    const auto samples = static_cast<std::size_t>(std::ceil(length / kSpacing)) + 1;

    const double start = path.wrap_s(s_start);
    std::vector<TrajectoryPoint> points;
    points.reserve(samples);
    for (std::size_t k = 0; k < samples; ++k) {
        const double d = static_cast<double>(k) * kSpacing;
        TrajectoryPoint p;
        if (path.closed() || start + d <= path.length()) {
            p = path.at(start + d);
        } else {
            // Past the end of an open path: straight on along its last heading.
            p = path.at(path.length());
            p.position += (start + d - path.length()) * heading_vector(p.yaw);
            p.curvature = 0.0;
        }
        const double braking = std::sqrt(std::max(speed * speed - 2.0 * decel * d, 0.0));
        p.speed = std::min(p.speed, braking);
        if (k + 1 == samples) p.speed = 0.0;
        points.push_back(p);
    }
    // Consistent accelerations from the speeds: a = (v1^2 - v0^2) / (2 ds).
    for (std::size_t k = 0; k + 1 < points.size(); ++k) {
        const double v0 = points[k].speed;
        const double v1 = points[k + 1].speed;
        points[k].accel = (v1 * v1 - v0 * v0) / (2.0 * kSpacing);
    }
    points.back().accel = 0.0;
    return Trajectory(std::move(points), false);
}

}  // namespace apex::stack
