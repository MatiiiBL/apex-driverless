#include "scene.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "apex/core/angles.hpp"

namespace apex::perception::testing {
namespace {

// First intersection (t > 0) of the ray o + t d with a cone standing at `center` (level frame).
std::optional<double> intersect_cone(const Vec3& o, const Vec3& d, const Vec2& center,
                                     const ConeShape& shape) {
    const double k2 = std::pow(shape.base_radius / shape.height, 2);
    const Vec3 rel(o.x() - center.x(), o.y() - center.y(), o.z() - shape.height);
    const double a = d.x() * d.x() + d.y() * d.y() - k2 * d.z() * d.z();
    const double b = 2.0 * (rel.x() * d.x() + rel.y() * d.y() - k2 * rel.z() * d.z());
    const double c = rel.x() * rel.x() + rel.y() * rel.y() - k2 * rel.z() * rel.z();
    double roots[2];
    int count = 0;
    if (std::abs(a) < 1e-12) {
        if (std::abs(b) < 1e-12) return std::nullopt;
        roots[count++] = -c / b;
    } else {
        const double disc = b * b - 4.0 * a * c;
        if (disc < 0.0) return std::nullopt;
        const double sq = std::sqrt(disc);
        const double t0 = (-b - sq) / (2.0 * a);
        const double t1 = (-b + sq) / (2.0 * a);
        roots[count++] = std::min(t0, t1);
        roots[count++] = std::max(t0, t1);
    }
    for (int i = 0; i < count; ++i) {
        const double t = roots[i];
        const double z = o.z() + t * d.z();
        if (t > 0.0 && z >= 0.0 && z <= shape.height) return t;
    }
    return std::nullopt;
}

}  // namespace

LidarScan cast_lidar(const LidarGeometry& geometry, const Pose2& sensor_pose,
                     std::span<const Cone> cones, const LidarSimOptions& options, Rng& rng,
                     Time stamp) {
    LidarScan scan;
    scan.stamp = stamp;
    const auto columns =
        static_cast<std::size_t>(std::floor(
            (geometry.azimuth_max - geometry.azimuth_min) / geometry.azimuth_step + 1e-9)) +
        1;

    // Cones each column can hit: those whose angular extent covers its azimuth.
    std::vector<std::vector<std::size_t>> by_column(columns);
    std::vector<Vec2> local(cones.size());
    for (std::size_t i = 0; i < cones.size(); ++i) {
        local[i] = sensor_pose.to_local(cones[i].position);
        const double distance = local[i].norm();
        const double radius = cone_shape(cones[i].color).base_radius;
        if (distance > geometry.max_range + radius) continue;
        const double half = distance > radius ? std::asin(radius / distance) + 1e-3 : kPi;
        const double bearing = std::atan2(local[i].y(), local[i].x());
        for (std::size_t k = 0; k < columns; ++k) {
            const double azimuth =
                geometry.azimuth_min + static_cast<double>(k) * geometry.azimuth_step;
            if (std::abs(angle_diff(azimuth, bearing)) <= half) by_column[k].push_back(i);
        }
    }

    const double cp = std::cos(options.pitch);
    const double sp = std::sin(options.pitch);
    const Vec3 origin(0.0, 0.0, options.sensor_height);
    for (std::size_t k = 0; k < columns; ++k) {
        const double azimuth =
            geometry.azimuth_min + static_cast<double>(k) * geometry.azimuth_step;
        for (const double elevation : geometry.elevations) {
            const Vec3 ds(std::cos(elevation) * std::cos(azimuth),
                          std::cos(elevation) * std::sin(azimuth), std::sin(elevation));
            const Vec3 dw(cp * ds.x() + sp * ds.z(), ds.y(), -sp * ds.x() + cp * ds.z());
            double best = std::numeric_limits<double>::infinity();
            std::int32_t id = -1;
            if (dw.z() < 0.0) best = -origin.z() / dw.z();
            for (const std::size_t i : by_column[k]) {
                const auto t = intersect_cone(origin, dw, local[i], cone_shape(cones[i].color));
                if (t && *t < best) {
                    best = *t;
                    id = cones[i].id;
                }
            }
            if (best < geometry.min_range || best > geometry.max_range) continue;
            if (options.dropout > 0.0 && rng.bernoulli(options.dropout)) continue;
            const double range =
                options.range_sigma > 0.0 ? best + rng.normal(0.0, options.range_sigma) : best;
            const Vec3 p = range * ds;
            scan.points.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()),
                                   static_cast<float>(p.z()), id});
        }
    }
    return scan;
}

CameraFrame detect_cones(const CameraGeometry& geometry, const Pose2& body_pose,
                         std::span<const Cone> cones, const CameraSimOptions& options, Rng& rng,
                         Time stamp) {
    CameraFrame frame;
    frame.stamp = stamp;
    for (const Cone& cone : cones) {
        const Vec2 rel = body_pose.to_local(cone.position) - geometry.mount.head<2>();
        const double bearing = std::atan2(rel.y(), rel.x());
        const double range = rel.norm();
        if (std::abs(bearing) > 0.5 * geometry.horizontal_fov || range > geometry.max_range) {
            continue;
        }
        CameraDetection d;
        d.bearing =
            bearing + (options.bearing_sigma > 0.0 ? rng.normal(0.0, options.bearing_sigma) : 0.0);
        d.range = range * (1.0 + (options.range_sigma_fraction > 0.0
                                      ? rng.normal(0.0, options.range_sigma_fraction)
                                      : 0.0));
        d.color = cone.color;
        d.confidence = options.confidence;
        d.truth_id = cone.id;
        frame.detections.push_back(d);
    }
    return frame;
}

Pose2 lidar_pose(const Pose2& body_pose, const LidarGeometry& geometry) {
    return body_pose.compose({geometry.mount.x(), geometry.mount.y(), 0.0});
}

}  // namespace apex::perception::testing
