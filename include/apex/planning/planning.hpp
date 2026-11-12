#pragma once

#include <memory>
#include <optional>
#include <span>
#include <vector>

#include "apex/core/messages.hpp"
#include "apex/core/timing.hpp"
#include "apex/core/trajectory.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

// Path and speed planning: the exploration planner for the unknown first lap, the track model
// built from the finished map, and the minimum-curvature raceline for the laps after it. The
// planning module owns every *Config struct's fields and defaults (docs/planning.md).
namespace apex::planning {

// ---- Speed profiles (shared by the exploration path and the raceline) ----

struct SpeedProfileConfig {};

// Fills speed and accel along `path` from a g-g-v envelope of the nominal car (friction ellipse
// with downforce, power and drive-force limits, drag) by forward-backward integration.
// An open path starts at `start_speed` and ends at `end_speed`; a closed path is periodic and
// ignores both.
void apply_speed_profile(Trajectory& path, const VehicleParams& nominal,
                         const SpeedProfileConfig& config, double start_speed = 0.0,
                         double end_speed = 0.0);

// Lap time of a closed path's speed profile (sum of ds / v).
[[nodiscard]] double profile_time(const Trajectory& path);

// ---- Lap 1: exploration ----

struct ExplorationConfig {};

struct ExplorationStats {
    LatencyStats plan_time;
    std::size_t plans = 0;
    std::size_t failures = 0;  // plan() found no path
};

// A path from the car through the cones mapped so far: Delaunay triangulation of the cones,
// candidate midpoint sequences scored against FS track rules (width, curvature, blue on the
// left and yellow on the right where colors are known), then smoothed and given a speed profile
// that can stop before the end of what has been seen.
class ExplorationPlanner {
public:
    ExplorationPlanner(const ExplorationConfig& config, const VehicleParams& nominal);
    ~ExplorationPlanner();
    ExplorationPlanner(ExplorationPlanner&&) noexcept;
    ExplorationPlanner& operator=(ExplorationPlanner&&) noexcept;

    // An open trajectory starting at the car, or nothing if no path can be found (the stack
    // then brakes on its last path).
    [[nodiscard]] std::optional<Trajectory> plan(const ConeMap& map, const StateEstimate& state);
    [[nodiscard]] const ExplorationStats& stats() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// ---- After lap 1: the track model ----

struct TrackModelConfig {};

struct TrackModel {
    Trajectory centerline;    // closed, in the driving direction; widths reach the cone lines
    std::vector<Vec2> left;   // ordered blue cone line, closed
    std::vector<Vec2> right;  // ordered yellow cone line, closed
};

// Orders the mapped cones into the two closed boundaries, using the path the car drove on lap 1
// to resolve ambiguities and fill gaps from missing or misclassified cones. Nothing if the map
// cannot form a closed track.
[[nodiscard]] std::optional<TrackModel> build_track_model(const ConeMap& map,
                                                          std::span<const Pose2> driven_path,
                                                          const TrackModelConfig& config);

// ---- The raceline ----

struct RacelineConfig {};

struct Raceline {
    Trajectory path;  // closed; speed, accel and free-space widths filled
    double predicted_lap_time = 0.0;
    double optimize_seconds = 0.0;  // wall-clock time spent
};

// Minimum-curvature raceline within the track model (minus the car's half width and a safety
// margin), solved as a sequence of QPs, then given a speed profile.
[[nodiscard]] Raceline optimize_raceline(const TrackModel& track, const VehicleParams& nominal,
                                         const RacelineConfig& config);

}  // namespace apex::planning
