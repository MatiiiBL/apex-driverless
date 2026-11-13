// The exploration planner's evaluation, quoted in docs/planning.md ("Exploration"). Release:
//
//   make PRESET=release build
//   ./build/release/bench/planning/apex_planning_exploration_eval \
//       --out results/planning/exploration.json --example results/planning/exploration_example.json
//
// On 20 generated tracks, at two levels of map imperfection, the car drives the centerline from
// its staging pose mapping the cones within 20 m ahead, and plans every 5 m from a pose up to
// 0.4 m and 0.1 rad off the centerline (test_support::evaluate_exploration). Prints JSON (and
// writes it to --out): how often a path is found, how often it stays between the true cone
// lines over its whole length, its deviation from the true centerline, its length, and plan()
// latency. --example writes one plan in a hairpin of the worst level for the docs figure.
#include "exploration_eval.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"
#include "apex/core/timing.hpp"
#include "apex/planning/planning.hpp"
#include "apex/sim/track_gen.hpp"

#include "cone_maps.hpp"

namespace {

using apex::planning::test_support::ExplorationScenario;
using apex::planning::test_support::MapErrors;
using apex::planning::test_support::PlanRecord;
using nlohmann::json;

constexpr std::uint64_t kTracks = 20;

double percentile(std::vector<double> values, double q) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const auto rank = static_cast<std::size_t>(std::ceil(q * static_cast<double>(values.size())));
    return values[std::clamp<std::size_t>(rank, 1, values.size()) - 1];
}

double mean(const std::vector<double>& values) {
    double sum = 0.0;
    for (const double v : values) sum += v;
    return values.empty() ? 0.0 : sum / static_cast<double>(values.size());
}

json evaluate_level(const MapErrors& errors, const apex::planning::ExplorationConfig& config) {
    std::size_t plans = 0;
    std::size_t found = 0;
    std::size_t inside = 0;
    std::vector<double> mean_deviation;
    std::vector<double> max_deviation;
    std::vector<double> length;
    apex::LatencyStats latency;
    json per_track = json::array();
    for (std::uint64_t seed = 1; seed <= kTracks; ++seed) {
        const apex::Track track = apex::sim::generate_track(seed);
        const std::vector<PlanRecord> records = apex::planning::test_support::evaluate_exploration(
            track, ExplorationScenario{.errors = errors, .seed = seed, .plan_every = 5.0}, config);
        std::size_t track_found = 0;
        std::size_t track_inside = 0;
        json failed_at = json::array();
        json outside_at = json::array();
        for (const PlanRecord& r : records) {
            latency.add(r.seconds);
            if (!r.found) {
                failed_at.push_back(r.s);
                continue;
            }
            ++track_found;
            if (r.inside) {
                ++track_inside;
            } else {
                outside_at.push_back(r.s);
            }
            mean_deviation.push_back(r.mean_deviation);
            max_deviation.push_back(r.max_deviation);
            length.push_back(r.length);
        }
        plans += records.size();
        found += track_found;
        inside += track_inside;
        per_track.push_back({{"seed", seed},
                             {"length_m", track.centerline.length()},
                             {"plans", records.size()},
                             {"found", track_found},
                             {"inside", track_inside},
                             {"failed_at_m", failed_at},
                             {"outside_at_m", outside_at}});
    }
    const auto ratio = [](std::size_t a, std::size_t b) {
        return b == 0 ? 0.0 : static_cast<double>(a) / static_cast<double>(b);
    };
    return {{"plans", plans},
            {"found_rate", ratio(found, plans)},
            {"inside_rate_of_found", ratio(inside, found)},
            {"inside_rate_of_all", ratio(inside, plans)},
            {"deviation_m",
             {{"mean", mean(mean_deviation)},
              {"p95_of_max", percentile(max_deviation, 0.95)},
              {"max", percentile(max_deviation, 1.0)}}},
            {"path_length_m", {{"mean", mean(length)}, {"p5", percentile(length, 0.05)}}},
            {"latency", latency.summary()},
            {"per_track", per_track}};
}

std::vector<double> xy(const apex::Vec2& p) {
    return {p.x(), p.y()};
}

// The plan made `s` meters into the lap (from the staging pose) of `track` with the scenario's
// map: the landmarks the car had mapped, its pose, the path and the true cone lines nearby.
json export_plan(const apex::Track& track, const ExplorationScenario& scenario, double s,
                 const apex::planning::ExplorationConfig& config) {
    json out;
    (void)apex::planning::test_support::evaluate_exploration(
        track, scenario, config,
        [&](const PlanRecord& record, const apex::ConeMap& map, const apex::Pose2& car,
            const std::optional<apex::Trajectory>& path) {
            if (std::abs(record.s - s) >= 0.5 * scenario.plan_every) return;
            json cones = json::array();
            for (const apex::MapCone& cone : map.cones) {
                if ((cone.position - car.position()).norm() < 30.0) {
                    cones.push_back({cone.position.x(), cone.position.y(), cone.color});
                }
            }
            json points = json::array();
            if (path) {
                for (const apex::TrajectoryPoint& p : path->points()) {
                    points.push_back({p.position.x(), p.position.y(), p.speed});
                }
            }
            json left = json::array();
            json right = json::array();
            for (const apex::Vec2& p : track.left_boundary()) {
                if ((p - car.position()).norm() < 40.0) left.push_back(xy(p));
            }
            for (const apex::Vec2& p : track.right_boundary()) {
                if ((p - car.position()).norm() < 40.0) right.push_back(xy(p));
            }
            out = {{"track", track.name}, {"car", {car.x, car.y, car.yaw}},
                   {"cones", cones},      {"path", points},
                   {"left", left},        {"right", right}};
        });
    return out;
}

// The docs figure: a plan 8 m before the tightest corner of the "technical" reference track,
// from a worst-case map.
json example(const apex::planning::ExplorationConfig& config) {
    const apex::Track track = apex::sim::reference_track("technical");
    const apex::Trajectory& center = track.centerline;
    double s_corner = 0.0;
    for (const apex::TrajectoryPoint& p : center.points()) {
        if (std::abs(p.curvature) > std::abs(center.at(s_corner).curvature)) s_corner = p.s;
    }
    const double start = apex::planning::test_support::staging_s(track);
    const double s = std::fmod(s_corner - start + 2.0 * center.length(), center.length()) - 8.0;
    return export_plan(
        track,
        {.errors = apex::planning::test_support::worst_case_errors(), .seed = 3, .plan_every = 1},
        std::round(s), config);
}

}  // namespace

// --config <file> evaluates an ExplorationConfig from JSON instead of the defaults.
// --plan <seed> <s> <nominal|worst> <file> writes the plan at s on generated track <seed> instead,
// for inspection: tools/planning/plot_planning.py --plan <file> <png> draws it.
int main(int argc, char** argv) {
    std::string out_path;
    std::string example_path;
    apex::planning::ExplorationConfig config;
    std::vector<std::string> plan;
    for (int i = 1; i < argc; ++i) {
        const std::string_view flag = argv[i];
        if (flag == "--out" && i + 1 < argc) out_path = argv[++i];
        if (flag == "--example" && i + 1 < argc) example_path = argv[++i];
        if (flag == "--config" && i + 1 < argc) {
            json config_json;
            std::ifstream(argv[++i]) >> config_json;
            config = config_json.get<apex::planning::ExplorationConfig>();
        }
        if (flag == "--plan" && i + 4 < argc) plan.assign(argv + i + 1, argv + i + 5);
    }
    if (!plan.empty()) {
        const auto seed = std::stoull(plan[0]);
        const MapErrors errors =
            plan[2] == "worst" ? apex::planning::test_support::worst_case_errors() : MapErrors{};
        std::ofstream(plan[3]) << export_plan(apex::sim::generate_track(seed),
                                              {.errors = errors, .seed = seed, .plan_every = 5.0},
                                              std::stod(plan[1]), config)
                                      .dump()
                               << '\n';
        return 0;
    }
    const json result = {
        {"tracks", kTracks},
        {"plan_every_m", 5.0},
        {"config", config},
        {"nominal", evaluate_level(MapErrors{}, config)},
        {"worst", evaluate_level(apex::planning::test_support::worst_case_errors(), config)}};
    std::cout << result.dump(2) << '\n';
    if (!out_path.empty()) std::ofstream(out_path) << result.dump(1) << '\n';
    if (!example_path.empty()) std::ofstream(example_path) << example(config).dump() << '\n';
    return 0;
}
