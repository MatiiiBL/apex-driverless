#include "apex/core/types.hpp"

#include <cmath>

#include "apex/core/angles.hpp"
#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/vehicle.hpp"

namespace apex {

Vec2 Pose2::to_parent(const Vec2& local) const {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {x + c * local.x() - s * local.y(), y + s * local.x() + c * local.y()};
}

Vec2 Pose2::to_local(const Vec2& parent) const {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double dx = parent.x() - x;
    const double dy = parent.y() - y;
    return {c * dx + s * dy, -s * dx + c * dy};
}

Pose2 Pose2::compose(const Pose2& delta) const {
    const Vec2 p = to_parent({delta.x, delta.y});
    return {p.x(), p.y(), wrap_angle(yaw + delta.yaw)};
}

Pose2 Pose2::inverse() const {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {-c * x - s * y, s * x - c * y, wrap_angle(-yaw)};
}

Pose2 Pose2::between(const Pose2& other) const {
    return inverse().compose(other);
}

std::string_view to_string(ConeColor color) {
    switch (color) {
        case ConeColor::Blue:
            return "blue";
        case ConeColor::Yellow:
            return "yellow";
        case ConeColor::Orange:
            return "orange";
        case ConeColor::BigOrange:
            return "big_orange";
        case ConeColor::Unknown:
            return "unknown";
    }
    return "unknown";
}

std::optional<ConeColor> cone_color_from_string(std::string_view name) {
    for (std::size_t i = 0; i < kConeColorCount; ++i) {
        const auto color = static_cast<ConeColor>(i);
        if (to_string(color) == name) return color;
    }
    return std::nullopt;
}

ConeShape cone_shape(ConeColor color) {
    if (color == ConeColor::BigOrange) return {0.1425, 0.505};
    return {0.114, 0.325};
}

double VehicleParams::downforce(double v) const {
    return 0.5 * air_density * downforce_area * v * v;
}

double VehicleParams::drag(double v) const {
    return 0.5 * air_density * drag_area * v * v;
}

std::vector<double> LidarGeometry::default_elevations() {
    constexpr int kChannels = 64;
    constexpr double kLow = -12.0;
    constexpr double kHigh = 4.0;
    std::vector<double> elevations;
    elevations.reserve(kChannels);
    for (int i = 0; i < kChannels; ++i) {
        elevations.push_back(deg_to_rad(kLow + (kHigh - kLow) * i / (kChannels - 1)));
    }
    return elevations;
}

Time stamp_of(const SensorMessage& message) {
    return std::visit([](const auto& m) { return m.stamp; }, message);
}

}  // namespace apex
