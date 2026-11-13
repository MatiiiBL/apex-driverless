#pragma once

#include "apex/core/trajectory.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/planning/planning.hpp"

namespace apex::stack {

// The reference for the stop after the last lap: `path` from arc length `s_start` on, with the
// speed profile of the nominal car's g-g-v envelope (planning::apply_speed_profile with
// `envelope`, the limits the raceline is planned with) that starts at `speed` and comes to rest
// in the shortest distance the envelope allows, then holds at rest for `margin`. Braking along a
// curve therefore leaves the tires the lateral grip the curve needs, as on any racing lap. An
// open path too short for the stop is extended straight along its final heading. Throws
// std::invalid_argument for an empty path or a negative speed or margin.
[[nodiscard]] Trajectory braking_reference(const Trajectory& path, double s_start, double speed,
                                           const VehicleParams& nominal,
                                           const planning::SpeedProfileConfig& envelope,
                                           double margin = 5.0);

}  // namespace apex::stack
