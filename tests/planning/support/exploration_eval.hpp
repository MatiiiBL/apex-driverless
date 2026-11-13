#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "apex/core/track.hpp"
#include "apex/planning/planning.hpp"

#include "cone_maps.hpp"

// The exploration planner's evaluation (docs/planning.md, "Exploration"): plans at poses along a
// track's centerline from the cones a car driving there would have mapped, scored against the
// true track. Shared by the tests and bench/planning.
namespace apex::planning::test_support {

struct ExplorationScenario {
    MapErrors errors{};
    std::uint64_t seed = 1;
    double plan_every = 5.0;     // m along the centerline between plans
    double sensor_range = 20.0;  // m, front half-plane
    double lateral_error = 0.4;  // m: the car is up to this far off the centerline...
    double yaw_error = 0.1;      // rad: ...and turned up to this much from its heading
    double speed = 8.0;          // m/s, the car's speed when it plans
};

struct PlanRecord {
    double s = 0.0;  // where on the lap (m from the staging pose)
    bool found = false;
    bool inside = false;          // every point of the path between the true cone lines
    double length = 0.0;          // m
    double max_deviation = 0.0;   // from the true centerline (m)
    double mean_deviation = 0.0;  // m
    double seconds = 0.0;         // wall-clock time of plan()
};

// Called after every plan with its record, the map and pose it was planned from, and the path.
using PlanObserver = std::function<void(const PlanRecord&, const ConeMap&, const Pose2&,
                                        const std::optional<Trajectory>&)>;

// One lap: the car drives the centerline from its staging pose, mapping the cones in range,
// and plans every plan_every meters from a perturbed pose.
[[nodiscard]] std::vector<PlanRecord> evaluate_exploration(const Track& track,
                                                           const ExplorationScenario& scenario,
                                                           const ExplorationConfig& config = {},
                                                           const PlanObserver& observer = {});

}  // namespace apex::planning::test_support
