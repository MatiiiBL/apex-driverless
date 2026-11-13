// Closed-loop throughput of the full stack (docs/runner.md, "Performance").
//
//   make PRESET=release build
//   build/release/bench/runner/apex_runner_bench [laps] [controller]
//
// One run on each reference layout, reporting its real-time factor, the simulator's mean tick
// cost and every module's compute-time percentiles; then the same runs as a sweep on 1, 2 and 3
// worker threads, for the pool's scaling. Prints one JSON object. Wall-clock numbers depend on
// the machine and on what else runs on it.

#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/log.hpp"
#include "apex/core/timing.hpp"
#include "apex/runner/run_stack.hpp"
#include "apex/runner/sweep.hpp"
#include "apex/sim/track_gen.hpp"

namespace {

using apex::runner::RunConfig;
using apex::runner::RunResult;
using nlohmann::json;

std::vector<apex::runner::SweepRun> layout_runs(int laps, const std::string& controller) {
    std::vector<apex::runner::SweepRun> runs;
    for (const std::string& layout : apex::sim::reference_track_names()) {
        apex::runner::SweepRun run;
        run.variant = controller;
        run.track = layout;
        run.config.name = layout;
        run.config.track.source = apex::runner::TrackSpec::Source::Layout;
        run.config.track.layout = layout;
        run.config.stack.mission.laps = laps;
        run.config.stack.controller.kind = json(controller).get<apex::control::ControllerKind>();
        runs.push_back(std::move(run));
    }
    return runs;
}

json run_summary(const RunResult& r) {
    json j = json::object();
    j["track"] = r.track_name;
    j["track_length_m"] = r.track_length;
    j["completed"] = r.completed;
    j["laps_timed"] = r.lap_times.size();
    j["dnf_reason"] = r.dnf_reason;
    j["sim_time_s"] = r.sim_time;
    j["wall_time_s"] = r.wall_time;
    j["realtime_factor"] = r.compute.value("realtime_factor", 0.0);
    j["compute"] = r.compute;
    return j;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const int laps = argc > 1 ? std::stoi(argv[1]) : 3;
        const std::string controller = argc > 2 ? argv[2] : "mpc";
        apex::set_log_level(apex::LogLevel::Warn);
        const std::vector<apex::runner::SweepRun> runs = layout_runs(laps, controller);
        const apex::runner::RunFunction run = [](const RunConfig& c) {
            return apex::runner::run_stack(c);
        };

        json out = {{"laps", laps}, {"controller", controller}};
        json single = json::array();
        for (const apex::runner::SweepRun& r : runs) single.push_back(run_summary(run(r.config)));
        out["runs"] = std::move(single);

        json scaling = json::array();
        double serial = 0.0;
        for (const unsigned threads : {1U, 2U, 3U}) {
            const apex::Stopwatch watch;
            (void)apex::runner::run_parallel(runs, run, threads);
            const double wall = watch.elapsed_seconds();
            if (threads == 1) serial = wall;
            scaling.push_back(
                {{"threads", threads}, {"wall_time_s", wall}, {"speedup", serial / wall}});
        }
        out["sweep_scaling"] = std::move(scaling);
        std::cout << out.dump(2) << "\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "apex_runner_bench: " << e.what() << "\n";
        return 1;
    }
}
