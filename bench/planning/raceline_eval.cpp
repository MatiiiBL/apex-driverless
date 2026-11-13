// The track model and raceline evaluation, quoted in docs/planning.md ("Track model",
// "Raceline"). Release:
//
//   make PRESET=release build
//   ./build/release/bench/planning/apex_planning_raceline_eval \
//       --out results/planning/raceline.json --example results/planning/raceline_example.json
//
// On 20 generated tracks: the track model built from imperfect maps (two levels) and a lap-1
// drive that wanders 0.5 m around the centerline, its centerline's distance to the true one;
// then the raceline optimized on the true track and on each model, its predicted lap time
// against the centerline's (same speed profile), its clearance from the true cone lines, and
// the optimization time. --example writes one track's model and raceline for the docs figure.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/geometry.hpp"
#include "apex/core/json.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/timing.hpp"
#include "apex/planning/planning.hpp"
#include "apex/sim/track_gen.hpp"

#include "cone_maps.hpp"

namespace {

using apex::Track;
using apex::Trajectory;
using apex::planning::Raceline;
using apex::planning::TrackModel;
using apex::planning::test_support::MapErrors;
using nlohmann::json;

constexpr std::uint64_t kTracks = 20;

struct Summary {
    std::vector<double> values{};
    void add(double v) { values.push_back(v); }
    [[nodiscard]] json to_json() const {
        if (values.empty()) return nullptr;
        std::vector<double> sorted = values;
        std::sort(sorted.begin(), sorted.end());
        double sum = 0.0;
        for (const double v : sorted) sum += v;
        return {{"mean", sum / static_cast<double>(sorted.size())},
                {"min", sorted.front()},
                {"median", sorted[sorted.size() / 2]},
                {"max", sorted.back()}};
    }
};

// The largest distance between the model's centerline and the true one, either way, and the
// RMS of the true centerline's distances to the model's.
std::pair<double, double> centerline_error(const TrackModel& model, const Track& track) {
    double worst = 0.0;
    for (const apex::TrajectoryPoint& p : model.centerline.points()) {
        worst = std::max(worst, track.centerline.project(p.position).distance);
    }
    double sum = 0.0;
    for (const apex::TrajectoryPoint& p : track.centerline.points()) {
        const double d = model.centerline.project(p.position).distance;
        worst = std::max(worst, d);
        sum += d * d;
    }
    return {worst, std::sqrt(sum / static_cast<double>(track.centerline.size()))};
}

// The path with the raceline's speed profile, for a like-for-like lap time.
Trajectory profiled(Trajectory path, const apex::planning::RacelineConfig& config) {
    apex::planning::apply_speed_profile(path, apex::VehicleParams{}, config.speed);
    return path;
}

double lap_time(const Trajectory& path, const apex::planning::RacelineConfig& config) {
    return apex::planning::profile_time(profiled(path, config));
}

// The smallest gap between the car's body (half its width beside the path) and a true cone
// line, along the whole raceline (m; negative if the body would cross one).
double body_clearance(const Raceline& raceline, const Track& track) {
    const std::vector<apex::Vec2> left = track.left_boundary();
    const std::vector<apex::Vec2> right = track.right_boundary();
    double clearance = 1e9;
    for (const apex::TrajectoryPoint& p : raceline.path.points()) {
        const double gap = std::min(apex::distance_to_polyline(p.position, left, true),
                                    apex::distance_to_polyline(p.position, right, true));
        clearance = std::min(clearance, gap - 0.5 * apex::VehicleParams{}.body_width);
    }
    return clearance;
}

struct Level {
    std::string name{};
    MapErrors errors{};
    std::size_t built = 0;
    Summary max_error{};
    Summary rms_error{};
    Summary gain{};
    Summary clearance{};
};

std::vector<std::vector<double>> points(const std::vector<apex::Vec2>& line) {
    std::vector<std::vector<double>> out;
    for (const apex::Vec2& p : line) out.push_back({p.x(), p.y()});
    return out;
}

}  // namespace

// --config <file> evaluates a RacelineConfig from JSON instead of the defaults.
int main(int argc, char** argv) {
    std::string out_path;
    std::string example_path;
    apex::planning::RacelineConfig config;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string_view flag = argv[i];
        if (flag == "--out") out_path = argv[i + 1];
        if (flag == "--example") example_path = argv[i + 1];
        if (flag == "--config") {
            json config_json;
            std::ifstream(argv[i + 1]) >> config_json;
            config = config_json.get<apex::planning::RacelineConfig>();
        }
    }
    const apex::VehicleParams car;
    std::vector<Level> levels = {
        {.name = "nominal", .errors = MapErrors{}},
        {.name = "worst", .errors = apex::planning::test_support::worst_case_errors()}};
    Summary truth_gain;
    Summary truth_clearance;
    apex::LatencyStats optimize_time;
    json per_track = json::array();
    json example;

    for (std::uint64_t seed = 1; seed <= kTracks; ++seed) {
        const Track track = apex::sim::generate_track(seed);
        const TrackModel truth{track.centerline, track.left_boundary(), track.right_boundary()};
        const Raceline ideal = apex::planning::optimize_raceline(truth, car, config);
        const double center_time = lap_time(track.centerline, config);
        optimize_time.add(ideal.optimize_seconds);
        truth_gain.add(1.0 - ideal.predicted_lap_time / center_time);
        truth_clearance.add(body_clearance(ideal, track));
        json row = {{"seed", seed},
                    {"length_m", track.centerline.length()},
                    {"centerline_lap_s", center_time},
                    {"raceline_lap_s", ideal.predicted_lap_time},
                    {"optimize_ms", 1e3 * ideal.optimize_seconds}};

        for (Level& level : levels) {
            const apex::Rng rng(seed);
            apex::Rng map_rng = rng.split(1);
            apex::Rng drive_rng = rng.split(2);
            const apex::ConeMap map =
                apex::planning::test_support::make_cone_map(track, level.errors, map_rng);
            const std::vector<apex::Pose2> drive =
                apex::planning::test_support::driven_lap(track, 0.5, drive_rng);
            const std::optional<TrackModel> model =
                apex::planning::build_track_model(map, drive, {});
            if (!model) {
                row[level.name] = nullptr;
                continue;
            }
            ++level.built;
            const auto [worst, rms] = centerline_error(*model, track);
            level.max_error.add(worst);
            level.rms_error.add(rms);
            const Raceline raceline = apex::planning::optimize_raceline(*model, car, config);
            optimize_time.add(raceline.optimize_seconds);
            const double model_center_time = lap_time(model->centerline, config);
            level.gain.add(1.0 - raceline.predicted_lap_time / model_center_time);
            level.clearance.add(body_clearance(raceline, track));
            row[level.name] = {{"centerline_max_error_m", worst},
                               {"centerline_rms_error_m", rms},
                               {"raceline_lap_s", raceline.predicted_lap_time},
                               {"body_clearance_m", body_clearance(raceline, track)}};
            if (seed == 1 && level.name == "nominal") {
                json path = json::array();
                for (const apex::TrajectoryPoint& p : raceline.path.points()) {
                    path.push_back({p.position.x(), p.position.y(), p.speed});
                }
                json center = json::array();
                const Trajectory model_center = profiled(model->centerline, config);
                for (const apex::TrajectoryPoint& p : model_center.points()) {
                    center.push_back({p.position.x(), p.position.y(), p.speed});
                }
                example = {{"track", track.name},
                           {"true_left", points(track.left_boundary())},
                           {"true_right", points(track.right_boundary())},
                           {"model_left", points(model->left)},
                           {"model_right", points(model->right)},
                           {"centerline", center},
                           {"raceline", path},
                           {"centerline_lap_s", model_center_time},
                           {"raceline_lap_s", raceline.predicted_lap_time}};
            }
        }
        per_track.push_back(row);
    }

    json result = {{"tracks", kTracks},
                   {"config", config},
                   {"true_track",
                    {{"lap_time_gain", truth_gain.to_json()},
                     {"body_clearance_m", truth_clearance.to_json()}}},
                   {"optimize_time", optimize_time.summary()},
                   {"per_track", per_track}};
    for (const Level& level : levels) {
        result[level.name] = {{"models_built", level.built},
                              {"centerline_max_error_m", level.max_error.to_json()},
                              {"centerline_rms_error_m", level.rms_error.to_json()},
                              {"lap_time_gain", level.gain.to_json()},
                              {"body_clearance_m", level.clearance.to_json()}};
    }
    std::cout << result.dump(2) << '\n';
    if (!out_path.empty()) std::ofstream(out_path) << result.dump(1) << '\n';
    if (!example_path.empty()) std::ofstream(example_path) << example.dump() << '\n';
    return 0;
}
