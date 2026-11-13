#pragma once

#include <optional>
#include <span>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

// The geometry behind simulate_lidar (sim/sensors.hpp), exposed for its tests and benchmark.
namespace apex::sim {

// The distance t > 0 along the unit `direction` from `origin` at which the ray first meets the
// lateral surface of a right circular cone standing on the ground (z = 0) at `base_center`, with
// base radius and height from `shape`; nothing if it misses.
//
// With apex a = (cx, cy, H) and k = R / H, the surface is (x - cx)^2 + (y - cy)^2 = k^2 (z - H)^2
// for 0 <= z <= H. Substituting origin + t direction gives a quadratic in t, solved in the
// cancellation-free form of Press et al., "Numerical Recipes", 3rd ed., 2007, section 5.6; the
// roots on the upper nappe (z > H) or below the ground are discarded. The base disk lies on the
// ground plane, which the caller intersects separately.
[[nodiscard]] std::optional<double> intersect_cone(const Vec3& origin, const Vec3& direction,
                                                   const Vec2& base_center, const ConeShape& shape);

// The noise-free scan with no spatial culling: every ray against the ground and every cone. The
// reference simulate_lidar is tested against; slow (tens of milliseconds per scan).
[[nodiscard]] LidarScan simulate_lidar_brute_force(const LidarGeometry& geometry,
                                                   const VehicleState& state,
                                                   std::span<const Cone> cones, Time stamp);

}  // namespace apex::sim
