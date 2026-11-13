#include "apex/sim/lidar.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

#include "apex/core/angles.hpp"
#include "apex/sim/sensors.hpp"

namespace apex::sim {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();
constexpr double kHalfPi = 0.5 * kPi;
// Widening of the culling bounds (rad), far above their rounding error, so culling never drops
// a cone a ray could hit.
constexpr double kCullMargin = 1e-6;

// The scan's ray directions in the sensor frame: column i, channel j points along
// (cos el_j cos az_i, cos el_j sin az_i, sin el_j).
struct Rays {
    std::vector<double> azimuth;
    std::vector<double> cos_azimuth;
    std::vector<double> sin_azimuth;
    std::vector<double> cos_elevation;
    std::vector<double> sin_elevation;

    explicit Rays(const LidarGeometry& g) {
        const auto columns = static_cast<std::size_t>(std::floor(
                                 (g.azimuth_max - g.azimuth_min) / g.azimuth_step + 1e-9)) +
                             1;
        for (std::size_t i = 0; i < columns; ++i) {
            const double a = g.azimuth_min + static_cast<double>(i) * g.azimuth_step;
            azimuth.push_back(a);
            cos_azimuth.push_back(std::cos(a));
            sin_azimuth.push_back(std::sin(a));
        }
        for (const double e : g.elevations) {
            cos_elevation.push_back(std::cos(e));
            sin_elevation.push_back(std::sin(e));
        }
    }
    [[nodiscard]] Vec3 direction(std::size_t i, std::size_t j) const {
        return {cos_elevation[j] * cos_azimuth[i], cos_elevation[j] * sin_azimuth[i],
                sin_elevation[j]};
    }
};

// A cone in the sensor frame (origin at the mount, axes parallel to the body).
struct LocalCone {
    Vec2 center = Vec2::Zero();
    ConeShape shape{};
    std::int32_t id = -1;
    double distance = 0.0;  // horizontal, sensor to the cone's axis
};

std::vector<LocalCone> to_sensor_frame(const LidarGeometry& geometry, const VehicleState& state,
                                       std::span<const Cone> cones) {
    const Vec2 mount = state.pose.to_parent(geometry.mount.head<2>());
    const Pose2 sensor{mount.x(), mount.y(), state.pose.yaw};
    std::vector<LocalCone> local;
    local.reserve(cones.size());
    for (const Cone& cone : cones) {
        LocalCone c;
        c.center = sensor.to_local(cone.position);
        c.shape = cone_shape(cone.color);
        c.id = cone.id;
        c.distance = c.center.norm();
        local.push_back(c);
    }
    return local;
}

// Distance along the ray to the ground, or infinity for rays at or above the horizon.
double ground_hit(double height, double sin_elevation) {
    return sin_elevation < 0.0 ? -height / sin_elevation : kInfinity;
}

void emit(const Vec3& direction, double range, std::int32_t id, LidarScan& scan) {
    const Vec3 p = range * direction;
    scan.points.push_back(
        {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()), id});
}

// Column ranges of the scan whose rays can meet a cone: those within its angular half-width
// asin(R / distance) of its bearing (every horizontal section of the cone lies inside its base
// circle). Tried at the bearing shifted by -2 pi, 0 and 2 pi, so scans that wrap around the back
// are handled; overlapping ranges only add duplicate candidates, which change no result.
template <class Visit>
void for_each_column(const Rays& rays, const LidarGeometry& g, const LocalCone& cone, Visit visit) {
    const std::size_t columns = rays.azimuth.size();
    if (cone.distance <= cone.shape.base_radius) {  // the sensor stands over the base
        for (std::size_t i = 0; i < columns; ++i) visit(i);
        return;
    }
    const double bearing = std::atan2(cone.center.y(), cone.center.x());
    const double half_width = std::asin(cone.shape.base_radius / cone.distance) + kCullMargin;
    for (const double shift : {-kTwoPi, 0.0, kTwoPi}) {
        const double lo = (bearing + shift - half_width - g.azimuth_min) / g.azimuth_step;
        const double hi = (bearing + shift + half_width - g.azimuth_min) / g.azimuth_step;
        if (hi < 0.0 || lo > static_cast<double>(columns - 1)) continue;
        const auto first = static_cast<std::size_t>(std::max(0.0, std::ceil(lo)));
        const auto last = std::min(columns - 1, static_cast<std::size_t>(std::floor(hi)));
        for (std::size_t i = first; i <= last; ++i) visit(i);
    }
}

// Bounds on the elevation of any point of the cone seen from a sensor `height` above the
// ground: the lowest point is on the ground at least distance - R away, the highest at most H up
// and at least distance - R away (at most distance + R when below the sensor).
struct ElevationBounds {
    double low = -kHalfPi;
    double high = kHalfPi;
};
ElevationBounds elevation_bounds(const LocalCone& cone, double height) {
    const double near = cone.distance - cone.shape.base_radius;
    const double far = cone.distance + cone.shape.base_radius;
    if (near <= 0.0) return {};
    const double top = cone.shape.height - height;
    return {std::atan2(-height, height > 0.0 ? near : far) - kCullMargin,
            std::atan2(top, top > 0.0 ? near : far) + kCullMargin};
}

}  // namespace

std::optional<double> intersect_cone(const Vec3& origin, const Vec3& direction,
                                     const Vec2& base_center, const ConeShape& shape) {
    const double k = shape.base_radius / shape.height;
    const double k2 = k * k;
    const Vec3 q = origin - Vec3(base_center.x(), base_center.y(), shape.height);
    const Vec3& d = direction;
    const double a = d.x() * d.x() + d.y() * d.y() - k2 * d.z() * d.z();
    const double b = 2.0 * (q.x() * d.x() + q.y() * d.y() - k2 * q.z() * d.z());
    const double c = q.x() * q.x() + q.y() * q.y() - k2 * q.z() * q.z();
    double roots[2] = {kInfinity, kInfinity};
    if (std::abs(a) < 1e-12) {  // parallel to a generator: one crossing
        if (b == 0.0) return std::nullopt;
        roots[0] = -c / b;
    } else {
        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0) return std::nullopt;
        const double half = -0.5 * (b + std::copysign(std::sqrt(discriminant), b));
        roots[0] = half / a;
        roots[1] = half != 0.0 ? c / half : roots[0];
        if (roots[1] < roots[0]) std::swap(roots[0], roots[1]);
    }
    for (const double t : roots) {
        if (!(t > 0.0) || !std::isfinite(t)) continue;
        const double z = origin.z() + t * d.z();
        if (z >= 0.0 && z <= shape.height) return t;
    }
    return std::nullopt;
}

LidarScan simulate_lidar(const LidarGeometry& geometry, const LidarNoise& noise,
                         const VehicleState& state, std::span<const Cone> cones, Time stamp,
                         Rng& rng) {
    const Rays rays(geometry);
    const double height = geometry.mount.z();
    const Vec3 origin(0.0, 0.0, height);
    std::vector<LocalCone> local = to_sensor_frame(geometry, state, cones);
    // Nearest first, so each ray stops testing cones once the next is beyond its best hit.
    std::erase_if(local, [&geometry](const LocalCone& c) {
        return c.distance - c.shape.base_radius > geometry.max_range;
    });
    std::sort(local.begin(), local.end(), [](const LocalCone& a, const LocalCone& b) {
        return a.distance < b.distance || (a.distance == b.distance && a.id < b.id);
    });
    std::vector<ElevationBounds> bounds;
    bounds.reserve(local.size());
    for (const LocalCone& c : local) bounds.push_back(elevation_bounds(c, height));

    // Candidate cones per column, in compressed rows (nearest first within each).
    const std::size_t columns = rays.azimuth.size();
    std::vector<std::uint32_t> row_start(columns + 1, 0);
    for (const LocalCone& c : local) {
        for_each_column(rays, geometry, c, [&row_start](std::size_t i) { ++row_start[i + 1]; });
    }
    std::partial_sum(row_start.begin(), row_start.end(), row_start.begin());
    std::vector<std::uint32_t> candidates(row_start.back());
    std::vector<std::uint32_t> fill(row_start.begin(), row_start.end() - 1);
    for (std::uint32_t k = 0; k < local.size(); ++k) {
        for_each_column(rays, geometry, local[k],
                        [&](std::size_t i) { candidates[fill[i]++] = k; });
    }

    LidarScan scan;
    scan.stamp = stamp;
    scan.points.reserve(columns * geometry.elevations.size());
    for (std::size_t i = 0; i < columns; ++i) {
        for (std::size_t j = 0; j < geometry.elevations.size(); ++j) {
            const Vec3 d = rays.direction(i, j);
            const double elevation = geometry.elevations[j];
            double best = ground_hit(height, rays.sin_elevation[j]);
            std::int32_t id = -1;
            for (std::uint32_t r = row_start[i]; r < row_start[i + 1]; ++r) {
                const std::uint32_t k = candidates[r];
                const LocalCone& c = local[k];
                // A hit is at least as far as the nearest point of the cone's base circle.
                if (c.distance - c.shape.base_radius >= best) break;
                if (elevation < bounds[k].low || elevation > bounds[k].high) continue;
                const std::optional<double> t = intersect_cone(origin, d, c.center, c.shape);
                if (t && *t < best) {
                    best = *t;
                    id = c.id;
                }
            }
            if (best < geometry.min_range || best > geometry.max_range) continue;
            if (rng.bernoulli(noise.dropout)) continue;
            const double range = best + rng.normal(0.0, noise.range_stddev);
            if (range > 0.0) emit(d, range, id, scan);
        }
    }
    return scan;
}

LidarScan simulate_lidar_brute_force(const LidarGeometry& geometry, const VehicleState& state,
                                     std::span<const Cone> cones, Time stamp) {
    const Rays rays(geometry);
    const double height = geometry.mount.z();
    const Vec3 origin(0.0, 0.0, height);
    const std::vector<LocalCone> local = to_sensor_frame(geometry, state, cones);
    LidarScan scan;
    scan.stamp = stamp;
    for (std::size_t i = 0; i < rays.azimuth.size(); ++i) {
        for (std::size_t j = 0; j < geometry.elevations.size(); ++j) {
            const Vec3 d = rays.direction(i, j);
            double best = ground_hit(height, rays.sin_elevation[j]);
            std::int32_t id = -1;
            for (const LocalCone& c : local) {
                const std::optional<double> t = intersect_cone(origin, d, c.center, c.shape);
                if (t && *t < best) {
                    best = *t;
                    id = c.id;
                }
            }
            if (best < geometry.min_range || best > geometry.max_range) continue;
            emit(d, best, id, scan);
        }
    }
    return scan;
}

}  // namespace apex::sim
