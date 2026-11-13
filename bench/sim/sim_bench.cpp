// Simulator performance, quoted in docs/sim.md ("Performance"). Release build, one core:
//
//   make PRESET=release build
//   ./build/release/bench/sim/apex_sim_bench --out results/sim/bench.json
//
// Prints the results as JSON (and writes them to --out): track generation time, LiDAR scan and
// camera frame cost from poses around three tracks (with the brute-force LiDAR for comparison),
// one vehicle model step, and the World's real-time factor over a lap with every sensor on.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/timing.hpp"
#include "apex/sim/lidar.hpp"
#include "apex/sim/sensors.hpp"
#include "apex/sim/track_gen.hpp"
#include "apex/sim/track_rules.hpp"
#include "apex/sim/vehicle_model.hpp"
#include "apex/sim/world.hpp"

namespace {

using apex::LatencyStats;
using apex::Stopwatch;
using apex::Track;
using apex::sim::World;
using nlohmann::json;

json track_generation() {
    LatencyStats time;
    int attempts = 0;
    double length = 0.0;
    apex::sim::TrackStats features;
    constexpr int kSeeds = 200;
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        Stopwatch watch;
        const apex::sim::GeneratedTrack generated = apex::sim::generate_track_with_stats(seed);
        time.add(watch.elapsed_seconds());
        attempts += generated.attempts;
        const apex::sim::TrackStats stats = apex::sim::analyze_track(generated.track);
        length += stats.length;
        features.hairpins += stats.hairpins;
        features.chicanes += stats.chicanes;
        features.sweepers += stats.sweepers;
        features.corners += stats.corners;
    }
    return {{"seeds", kSeeds},
            {"time", time.summary()},
            {"mean_attempts", attempts / static_cast<double>(kSeeds)},
            {"mean_length_m", length / kSeeds},
            {"per_track",
             {{"hairpins", features.hairpins / static_cast<double>(kSeeds)},
              {"chicanes", features.chicanes / static_cast<double>(kSeeds)},
              {"sweepers", features.sweepers / static_cast<double>(kSeeds)},
              {"corners", features.corners / static_cast<double>(kSeeds)}}}};
}

// The car on the centerline every `spacing` meters, facing along it.
std::vector<apex::VehicleState> poses(const Track& track, double spacing) {
    std::vector<apex::VehicleState> out;
    for (double s = 0.0; s < track.centerline.length(); s += spacing) {
        const apex::TrajectoryPoint p = track.centerline.at(s);
        apex::VehicleState state;
        state.pose = {p.position.x(), p.position.y(), p.yaw};
        out.push_back(state);
    }
    return out;
}

std::vector<Track> bench_tracks() {
    return {apex::sim::generate_track(1), apex::sim::reference_track("technical"),
            apex::sim::reference_track("flowing")};
}

json lidar() {
    const apex::LidarGeometry geometry;
    const apex::sim::LidarNoise noise;
    apex::Rng rng(1);
    LatencyStats fast;
    LatencyStats brute;
    double points = 0.0;
    double cone_points = 0.0;
    int scans = 0;
    for (const Track& track : bench_tracks()) {
        const std::vector<apex::VehicleState> states = poses(track, 5.0);
        for (std::size_t i = 0; i < states.size(); ++i) {
            Stopwatch watch;
            const apex::LidarScan scan =
                apex::sim::simulate_lidar(geometry, noise, states[i], track.cones, 0.0, rng);
            fast.add(watch.elapsed_seconds());
            points += static_cast<double>(scan.points.size());
            for (const apex::LidarPoint& p : scan.points) cone_points += p.truth_id >= 0 ? 1 : 0;
            ++scans;
            if (i % 20 == 0) {
                watch.restart();
                (void)apex::sim::simulate_lidar_brute_force(geometry, states[i], track.cones, 0.0);
                brute.add(watch.elapsed_seconds());
            }
        }
    }
    const auto columns =
        std::floor((geometry.azimuth_max - geometry.azimuth_min) / geometry.azimuth_step + 1e-9) +
        1.0;
    return {{"scans", scans},
            {"rays_per_scan", columns * static_cast<double>(geometry.elevations.size())},
            {"mean_returns", points / scans},
            {"mean_cone_returns", cone_points / scans},
            {"culled", fast.summary()},
            {"brute_force", brute.summary()},
            {"speedup_mean", brute.mean() / fast.mean()}};
}

json camera() {
    const apex::CameraGeometry geometry;
    const apex::sim::CameraNoise noise;
    apex::Rng rng(2);
    LatencyStats time;
    double detections = 0.0;
    int frames = 0;
    for (const Track& track : bench_tracks()) {
        for (const apex::VehicleState& state : poses(track, 2.0)) {
            Stopwatch watch;
            const apex::CameraFrame frame =
                apex::sim::simulate_camera(geometry, noise, state, track.cones, 0.0, rng);
            time.add(watch.elapsed_seconds());
            detections += static_cast<double>(frame.detections.size());
            ++frames;
        }
    }
    return {{"frames", frames}, {"mean_detections", detections / frames}, {"time", time.summary()}};
}

json vehicle_step() {
    apex::sim::VehicleModel model{apex::VehicleParams{}};
    apex::VehicleState start;
    start.vx = 10.0;
    model.reset(start);
    constexpr int kSteps = 1'000'000;
    Stopwatch watch;
    for (int i = 0; i < kSteps; ++i) {
        if (i % 20 == 0) {
            const double t = model.time();
            model.command({t, 0.1 * std::sin(t), 2.0 * std::cos(0.3 * t)});
        }
        model.step();
    }
    const double seconds = watch.elapsed_seconds();
    return {{"steps", kSteps}, {"ns_per_step", seconds / kSteps * 1e9}};
}

// A truth-fed driver: pure pursuit on the centerline (Coulter, CMU-RI-TR-92-01, 1992) at a speed
// that keeps the lateral acceleration on the next 15 m under 8 m/s^2.
apex::ControlCommand drive(const World& world) {
    const apex::VehicleState& s = world.truth();
    const apex::Trajectory& center = world.track().centerline;
    const double here = center.project(s.pose.position()).s;
    const apex::Vec2 local = s.pose.to_local(center.at(here + 6.0).position);
    const apex::VehicleParams& p = world.config().vehicle;
    const double steer = std::atan(2.0 * p.wheelbase() * local.y() / local.squaredNorm());
    double curvature = 0.0;
    for (double ds = 0.0; ds <= 15.0; ds += 1.0) {
        curvature = std::max(curvature, std::abs(center.at(here + ds).curvature));
    }
    const double speed = std::min(12.0, std::sqrt(8.0 / std::max(curvature, 1e-3)));
    const double resistance =
        p.drag(s.vx) + p.rolling_resistance * (p.mass * apex::kGravity + p.downforce(s.vx));
    return {world.time(), steer, resistance / p.mass + 2.0 * (speed - s.vx)};
}

json world_run() {
    World world(apex::sim::generate_track(1), apex::sim::WorldConfig{});
    LatencyStats tick;
    std::size_t messages = 0;
    Stopwatch total;
    while (world.race().laps_completed() < 1 && world.time() < 120.0) {
        if (world.tick() % 20 == 0) world.command(drive(world));
        Stopwatch watch;
        messages += world.step().size();
        tick.add(watch.elapsed_seconds());
    }
    const double wall = total.elapsed_seconds();
    return {{"track", world.track().name},
            {"simulated_s", world.time()},
            {"wall_s", wall},
            {"real_time_factor", world.time() / wall},
            {"laps", world.race().laps_completed()},
            {"lap_time_s", world.race().laps().empty() ? 0.0 : world.race().laps()[0].time()},
            {"penalty_s", world.race().penalty_seconds()},
            {"messages", messages},
            {"tick", tick.summary()}};
}

}  // namespace

int main(int argc, char** argv) {
    std::string out;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--out" && i + 1 < argc) {
            out = argv[++i];
        } else {
            std::cerr << "usage: apex_sim_bench [--out results.json]\n";
            return 2;
        }
    }
    json results;
    results["track_generation"] = track_generation();
    results["lidar"] = lidar();
    results["camera"] = camera();
    results["vehicle_step"] = vehicle_step();
    results["world"] = world_run();
    std::cout << results.dump(2) << "\n";
    if (!out.empty()) {
        std::ofstream file(out);
        if (!file) {
            std::cerr << "cannot write " << out << "\n";
            return 1;
        }
        file << results.dump(2) << "\n";
    }
    return 0;
}
