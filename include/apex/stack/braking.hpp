#pragma once

#include "apex/core/trajectory.hpp"

namespace apex::stack {

// The reference for the stop after the last lap: `path` from arc length `s_start` on, its speed
// profile capped by braking at a constant `decel` from `speed` to a standstill,
//   v(d) = sqrt(max(speed^2 - 2 decel d, 0))   at distance d along the path,
// and held at zero for `margin` past the stopping point. An open path too short for that is
// extended straight along its final heading. Throws std::invalid_argument for non-positive
// decel or a negative speed or margin.
[[nodiscard]] Trajectory braking_reference(const Trajectory& path, double s_start, double speed,
                                           double decel, double margin = 5.0);

}  // namespace apex::stack
