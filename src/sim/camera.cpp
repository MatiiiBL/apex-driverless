#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

#include "apex/core/angles.hpp"
#include "apex/sim/lidar.hpp"
#include "apex/sim/sensors.hpp"

namespace apex::sim {

namespace {

// The colors a detector can report: the four FS cone classes.
constexpr std::array<ConeColor, 4> kClasses = {ConeColor::Blue, ConeColor::Yellow,
                                               ConeColor::Orange, ConeColor::BigOrange};

struct Candidate {
    Vec2 position = Vec2::Zero();  // relative to the camera mount, body axes
    double range = 0.0;
    double bearing = 0.0;
    const Cone* cone = nullptr;
};

double detection_probability(const CameraNoise& noise, double range) {
    return noise.detection_probability /
           (1.0 + std::exp((range - noise.detection_half_range) / noise.detection_falloff));
}

// True if the line of sight from the camera to the middle of the target's body passes through
// another cone's body first.
bool occluded(const Candidate& target, const std::vector<Candidate>& others, double height) {
    const ConeShape shape = cone_shape(target.cone->color);
    const Vec3 origin(0.0, 0.0, height);
    const Vec3 to_target =
        Vec3(target.position.x(), target.position.y(), 0.5 * shape.height) - origin;
    const double length = to_target.norm();
    const Vec3 direction = to_target / length;
    for (const Candidate& other : others) {
        if (other.cone == target.cone) continue;
        const ConeShape blocker = cone_shape(other.cone->color);
        if (other.range - blocker.base_radius >= target.range) continue;
        // Outside the blocker's angular half-width the sight line cannot touch it.
        const double half_width = std::asin(std::min(1.0, blocker.base_radius / other.range));
        if (std::abs(angle_diff(other.bearing, target.bearing)) > half_width + 1e-6) {
            continue;
        }
        const std::optional<double> t = intersect_cone(origin, direction, other.position, blocker);
        if (t && *t < length) return true;
    }
    return false;
}

ConeColor confused_color(ConeColor truth, Rng& rng) {
    std::array<ConeColor, 3> others{};
    std::size_t n = 0;
    for (const ConeColor c : kClasses) {
        if (c != truth && n < others.size()) others[n++] = c;
    }
    return others[rng.index(n)];
}

}  // namespace

CameraFrame simulate_camera(const CameraGeometry& geometry, const CameraNoise& noise,
                            const VehicleState& state, std::span<const Cone> cones, Time stamp,
                            Rng& rng) {
    const Vec2 mount = geometry.mount.head<2>();
    const double half_fov = 0.5 * geometry.horizontal_fov;
    std::vector<Candidate> in_view;
    for (const Cone& cone : cones) {
        Candidate c;
        c.position = state.pose.to_local(cone.position) - mount;
        c.range = c.position.norm();
        c.bearing = std::atan2(c.position.y(), c.position.x());
        c.cone = &cone;
        if (c.range <= geometry.max_range && std::abs(c.bearing) <= half_fov) {
            in_view.push_back(c);
        }
    }

    CameraFrame frame;
    frame.stamp = stamp;
    for (const Candidate& c : in_view) {
        if (c.range < noise.min_range || occluded(c, in_view, geometry.mount.z())) continue;
        const double p = detection_probability(noise, c.range);
        if (!rng.bernoulli(p)) continue;
        CameraDetection d;
        d.bearing = c.bearing + rng.normal(0.0, noise.bearing_stddev);
        const double range_stddev =
            noise.range_stddev + noise.range_stddev_quadratic * c.range * c.range;
        d.range = std::max(0.1, c.range + rng.normal(0.0, range_stddev));
        const double confusion =
            noise.confusion_near +
            (noise.confusion_far - noise.confusion_near) * c.range / geometry.max_range;
        d.color = rng.bernoulli(confusion) ? confused_color(c.cone->color, rng) : c.cone->color;
        d.confidence = std::clamp(p + rng.normal(0.0, noise.confidence_stddev), 0.0, 1.0);
        d.truth_id = c.cone->id;
        frame.detections.push_back(d);
    }
    const std::uint32_t false_positives = rng.poisson(noise.false_positives);
    for (std::uint32_t k = 0; k < false_positives; ++k) {
        CameraDetection d;
        d.bearing = rng.uniform(-half_fov, half_fov);
        d.range = rng.uniform(noise.min_range, geometry.max_range);
        d.color = kClasses[rng.index(kClasses.size())];
        d.confidence = rng.uniform(0.1, 0.6);
        frame.detections.push_back(d);
    }
    std::stable_sort(
        frame.detections.begin(), frame.detections.end(),
        [](const CameraDetection& a, const CameraDetection& b) { return a.bearing < b.bearing; });
    return frame;
}

}  // namespace apex::sim
