#include "exploration_eval.hpp"

#include <algorithm>
#include <optional>

#include "apex/core/timing.hpp"

namespace apex::planning::test_support {

std::vector<PlanRecord> evaluate_exploration(const Track& track,
                                             const ExplorationScenario& scenario,
                                             const ExplorationConfig& config,
                                             const PlanObserver& observer) {
    const Rng rng(scenario.seed);
    Rng map_rng = rng.split(1);
    Rng pose_rng = rng.split(2);
    Sightings sightings(make_cone_map(track, scenario.errors, map_rng), scenario.sensor_range);
    ExplorationPlanner planner(config, VehicleParams{});

    std::vector<PlanRecord> records;
    const double start = staging_s(track);
    const double length = track.centerline.length();
    constexpr double kStep = 0.5;  // m between the poses the car maps from
    double next_plan = 0.0;
    for (double driven = 0.0; driven < length; driven += kStep) {
        sightings.observe(centerline_pose(track, start + driven));
        if (driven < next_plan) continue;
        next_plan += scenario.plan_every;

        const double lateral = pose_rng.uniform(-scenario.lateral_error, scenario.lateral_error);
        const double yaw = pose_rng.uniform(-scenario.yaw_error, scenario.yaw_error);
        const Pose2 pose = centerline_pose(track, start + driven, lateral, yaw);
        StateEstimate state;
        state.pose = pose;
        state.vx = scenario.speed;
        state.valid = true;
        const ConeMap map = sightings.map();
        const Stopwatch stopwatch;
        const std::optional<Trajectory> path = planner.plan(map, state);
        PlanRecord record;
        record.seconds = stopwatch.elapsed_seconds();
        record.s = driven;
        record.found = path.has_value();
        if (path) {
            record.length = path->length();
            record.inside = true;
            double sum = 0.0;
            // Near the car, so a window around its own arc length is enough and keeps a nearby
            // part of the track from capturing the projection.
            const double s_car = track.centerline.project(pose.position()).s;
            for (const TrajectoryPoint& p : path->points()) {
                const Trajectory::Projection proj =
                    track.centerline.project(p.position, s_car + p.s, 15.0);
                const TrajectoryPoint truth = track.centerline.at(proj.s);
                const double width = proj.lateral > 0.0 ? truth.width_left : truth.width_right;
                record.inside = record.inside && proj.distance < width;
                record.max_deviation = std::max(record.max_deviation, proj.distance);
                sum += proj.distance;
            }
            record.mean_deviation = sum / static_cast<double>(path->size());
        }
        if (observer) observer(record, map, pose, path);
        records.push_back(record);
    }
    return records;
}

}  // namespace apex::planning::test_support
