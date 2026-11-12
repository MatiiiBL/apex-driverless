#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

#include <Eigen/Core>

namespace apex {

// Seconds of simulated time since the run started. Every message carries one.
using Time = double;

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat2 = Eigen::Matrix2d;
using Mat3 = Eigen::Matrix3d;

// A planar pose: position in meters and heading in radians, counter-clockwise from the parent
// frame's +x axis, kept in (-pi, pi]. A pose also names a frame: x forward, y to the left.
struct Pose2 {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;

    [[nodiscard]] Vec2 position() const { return {x, y}; }
    // A point given in this pose's frame, expressed in the parent frame.
    [[nodiscard]] Vec2 to_parent(const Vec2& local) const;
    // A point given in the parent frame, expressed in this pose's frame.
    [[nodiscard]] Vec2 to_local(const Vec2& parent) const;
    // This pose followed by `delta`, where `delta` is expressed in this pose's frame.
    [[nodiscard]] Pose2 compose(const Pose2& delta) const;
    [[nodiscard]] Pose2 inverse() const;
    // The pose `other` expressed in this pose's frame: inverse().compose(other).
    [[nodiscard]] Pose2 between(const Pose2& other) const;
};

// Cone colors per the Formula Student rules. Blue cones mark the left edge of the track and
// yellow cones the right edge, relative to the driving direction. Big orange cones mark the
// start/finish (timekeeping) line; small orange cones mark exits and are unused on a trackdrive.
enum class ConeColor : std::uint8_t { Blue, Yellow, Orange, BigOrange, Unknown };
inline constexpr std::size_t kConeColorCount = 5;

[[nodiscard]] std::string_view to_string(ConeColor color);
[[nodiscard]] std::optional<ConeColor> cone_color_from_string(std::string_view name);
[[nodiscard]] constexpr std::size_t index_of(ConeColor color) {
    return static_cast<std::size_t>(color);
}

// Physical cone size from the FS rules: small cones have a 228 mm base and stand 325 mm tall,
// big orange cones 285 mm and 505 mm. The radius is that of the body at ground level.
struct ConeShape {
    double base_radius;
    double height;
};
[[nodiscard]] ConeShape cone_shape(ConeColor color);

// A cone as placed on the track: ground truth, known only to the simulator and the evaluation.
// The car's software never sees one.
struct Cone {
    std::int32_t id = -1;  // index into Track::cones
    Vec2 position = Vec2::Zero();
    ConeColor color = ConeColor::Unknown;
};

}  // namespace apex
