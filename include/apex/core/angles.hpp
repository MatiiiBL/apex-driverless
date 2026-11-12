#pragma once

#include <cmath>
#include <numbers>

namespace apex {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline constexpr double kGravity = 9.81;  // m/s^2

[[nodiscard]] constexpr double deg_to_rad(double degrees) {
    return degrees * kPi / 180.0;
}
[[nodiscard]] constexpr double rad_to_deg(double radians) {
    return radians * 180.0 / kPi;
}

// The equivalent angle in (-pi, pi].
[[nodiscard]] inline double wrap_angle(double angle) {
    double wrapped = std::remainder(angle, kTwoPi);  // [-pi, pi]
    if (wrapped <= -kPi) wrapped += kTwoPi;
    return wrapped;
}

// The signed smallest rotation that takes `from` to `to`, in (-pi, pi].
[[nodiscard]] inline double angle_diff(double to, double from) {
    return wrap_angle(to - from);
}

}  // namespace apex
