#pragma once

#include <memory>
#include <optional>
#include <span>
#include <vector>

#include <nlohmann/json_fwd.hpp>

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

struct SpeedProfileConfig {
    // Safety factor on the nominal tires' friction coefficient: the profile plans with
    // grip_factor * mu on each axle's friction ellipse. In (0, 1].
    double grip_factor = 0.85;
    // Speed cap (m/s) on top of the car's own gearing limit, VehicleParams::max_speed.
    double max_speed = 30.0;
};

// Fills speed and accel along `path` from a g-g-v envelope of the nominal car (each axle's
// friction ellipse with its share of the forces, downforce and load transfer; power and
// drive-force limits; drag) by forward-backward integration (apex/planning/ggv.hpp).
// An open path starts at `start_speed` and ends at `end_speed`; a closed path is periodic and
// ignores both.
void apply_speed_profile(Trajectory& path, const VehicleParams& nominal,
                         const SpeedProfileConfig& config, double start_speed = 0.0,
                         double end_speed = 0.0);

// Lap time of a closed path's speed profile (sum of ds / v).
[[nodiscard]] double profile_time(const Trajectory& path);

// ---- Lap 1: exploration ----

struct ExplorationConfig {
    // Cones triangulated: within cone_range (m) of the car and at most behind_distance (m)
    // behind it, the closest max_cones of them.
    double cone_range = 25.0;
    double behind_distance = 4.0;
    std::size_t max_cones = 80;

    // Beam search over sequences of triangulation edges, from an edge whose midpoint is ahead
    // of the car within start_radius (m).
    std::size_t beam_width = 12;
    std::size_t max_edges = 24;
    double max_path_length = 30.0;  // m along the midpoints
    double start_radius = 6.0;

    // Hard limits: the track is min_width to max_width (m) wide where the path crosses an edge
    // (midpoint_search.hpp says how it is measured), and the edge at most max_edge_length long;
    // the cone line between two crossed edges is at most max_cone_spacing long (two missing
    // cones on a straight); the path turns at most max_turn_deg between consecutive midpoints (and
    // from the car's heading at the start); every midpoint, and the path between consecutive
    // ones, stays at least min_cone_clearance (m) from every mapped cone, not only from the
    // cones of the edges it crosses: closer, the path would run over a cone.
    double min_width = 2.0;
    double max_width = 5.5;
    double max_edge_length = 12.0;
    double max_cone_spacing = 15.0;
    double max_turn_deg = 50.0;
    double min_cone_clearance = 1.0;

    // Scores, per crossed edge, all dimensionless; the path with the lowest total wins.
    // width = width_weight ((w - track_width) / track_width)^2
    // curvature = curvature_weight (kappa / curvature_scale)^2
    // color = color_weight per cone on the wrong side, times its share of the color votes
    // spacing = spacing_weight ((l - cone_spacing)+ / cone_spacing)^2
    // heading = heading_weight (turn from the car's heading / heading_scale)^2
    double track_width = 3.5;  // m
    double width_weight = 6.0;
    double curvature_scale = 0.15;  // 1/m
    double curvature_weight = 3.0;
    double color_weight = 4.0;
    double cone_spacing = 5.0;  // m
    double spacing_weight = 1.0;
    double heading_scale_deg = 30.0;
    double heading_weight = 1.0;
    double length_reward = 1.0;  // subtracted per meter of path

    // The best midpoint sequence is smoothed (Whittaker, this cutoff wavelength in m), sampled
    // every path_spacing (m), and given a speed profile that reaches 0 m/s before its end:
    // stop_margin (m) before it, or stop_reserve times the path's length, whichever is more, so
    // that the next plan's path may end that much closer and the car can still stop within it.
    // Shorter paths than min_path_length (m) count as no path.
    double smoothing_wavelength = 8.0;
    double path_spacing = 0.5;
    double stop_margin = 1.0;
    double stop_reserve = 0.4;
    double min_path_length = 3.0;
    SpeedProfileConfig speed{.grip_factor = 0.7, .max_speed = 12.0};
};

struct ExplorationStats {
    LatencyStats plan_time;
    std::size_t plans = 0;     // plan() calls
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

struct TrackModelConfig {
    // Arc-length spacing of the reference line the cones are ordered along and of the samples
    // the centerline is computed at (m).
    double reference_spacing = 1.0;
    // Cones whose lateral distance from the reference is outside [min_lateral, max_lateral] (m)
    // are clutter: on the line the car drove, or beyond any cone line.
    double min_lateral = 0.4;
    double max_lateral = 3.5;
    // A cone further than outlier_distance (m) from the median lateral offset of the
    // outlier_neighbors cones of its side nearest to it along the track is an outlier. Six
    // neighbors outvote a cluster of three clutter landmarks.
    double outlier_distance = 0.6;
    std::size_t outlier_neighbors = 6;
    // Gaps between consecutive cones of a side longer than fill_gap (m) get points every
    // fill_spacing (m), at the side's interpolated offset; a gap longer than max_gap (m) means
    // the map cannot form a closed track. Cones stand up to 5 m apart on straights, so with a
    // fifth of them missing, gaps of four or five spacings are expected on a long track.
    double fill_gap = 5.5;
    double fill_spacing = 2.0;
    double max_gap = 30.0;
    std::size_t min_cones_per_side = 8;
    // The driven path must come back within this distance (m) of its start to close a lap.
    double max_closure_gap = 5.0;
    // Cutoff wavelengths (m) of the smoothness priors on the centerline's shape and on the
    // track's width: shorter wiggles (cone noise) are damped, longer ones pass. The width
    // changes slowly, so its prior also carries one side's shape across the other's gaps.
    double smoothing_wavelength = 10.0;
    double width_wavelength = 40.0;
    // Estimation passes: the first places the cones along the driven path, each further one
    // along the previous pass's centerline.
    std::size_t passes = 2;
    double centerline_spacing = 0.5;  // m, sampling of TrackModel::centerline
};

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

struct RacelineConfig {
    // Arc length between optimization points (m). The QP is dense in their number, so a 500 m
    // track at 2.5 m has 200 variables.
    double step = 2.5;
    // Kept from each cone line on top of half the car's body width (m): the cones' own base
    // radius (0.11 m), the track model's error (up to 0.3 m with a nominal map), the
    // controller's tracking error and localization. At 0.5 m the body stays at least 0.2 m from
    // the true cone lines on racelines optimized on nominal-map models (docs/planning.md).
    double safety_margin = 0.5;
    // QPs solved, each linearized around the previous one's solution (Heilmeier et al. report
    // two or three to be enough).
    std::size_t iterations = 3;
    // Bound on the path's curvature (1/m), below the nominal car's kinematic limit
    // tan(max_steer) / wheelbase = 0.27 1/m.
    double max_curvature = 0.25;
    // Weight of the offsets' squares against the curvature term: it makes the QP strictly
    // convex, so its solution is unique, and keeps the offsets from drifting where the
    // curvature does not depend on them (constant offsets on straights).
    double regularization = 1e-6;
    std::size_t qp_max_iterations = 60;
    double qp_tolerance = 1e-9;
    double path_spacing = 0.5;  // m, sampling of Raceline::path
    // The racing laps' speed profile. In closed loop (both oracles, Stanley, nine tracks) a
    // grip factor of 0.85 knocked 0.8 cones per run, 0.75 one in nine runs, 0.70 none.
    SpeedProfileConfig speed{.grip_factor = 0.70, .max_speed = 30.0};
};

struct Raceline {
    Trajectory path;  // closed; speed, accel and free-space widths filled
    double predicted_lap_time = 0.0;
    double optimize_seconds = 0.0;  // wall-clock time spent
};

// Minimum-curvature raceline within the track model (minus the car's half width and a safety
// margin), solved as a sequence of QPs, then given a speed profile.
[[nodiscard]] Raceline optimize_raceline(const TrackModel& track, const VehicleParams& nominal,
                                         const RacelineConfig& config);

// ---- JSON (missing keys keep their defaults, unknown keys throw std::invalid_argument) ----

void to_json(nlohmann::json& j, const SpeedProfileConfig& config);
void from_json(const nlohmann::json& j, SpeedProfileConfig& config);
void to_json(nlohmann::json& j, const ExplorationConfig& config);
void from_json(const nlohmann::json& j, ExplorationConfig& config);
void to_json(nlohmann::json& j, const TrackModelConfig& config);
void from_json(const nlohmann::json& j, TrackModelConfig& config);
void to_json(nlohmann::json& j, const RacelineConfig& config);
void from_json(const nlohmann::json& j, RacelineConfig& config);

}  // namespace apex::planning
