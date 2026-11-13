#include <algorithm>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/log.hpp"
#include "apex/runner/config.hpp"
#include "apex/runner/recorders.hpp"
#include "apex/runner/run_stack.hpp"
#include "apex/runner/summary.hpp"
#include "apex/runner/sweep.hpp"

#include "cli.hpp"
#include "commands.hpp"

namespace apex::cli {

namespace {

using nlohmann::json;
using runner::RunConfig;
using runner::RunResult;

RunConfig read_run_config(const std::filesystem::path& path) {
    const json j = read_json(path);
    try {
        RunConfig config = j.get<RunConfig>();
        runner::validate(config);
        return config;
    } catch (const std::exception& e) {
        throw std::runtime_error(path.string() + ": " + e.what());
    }
}

// One line on how a run went.
std::string outcome(const RunResult& r) {
    if (!r.error.empty()) return "error: " + r.error;
    std::string line;
    if (r.completed) {
        line = std::format("{} laps in {:.2f} s + {:.0f} s penalties", r.lap_times.size(),
                           r.total_time(), r.penalty_seconds);
        if (const auto lap1 = r.lap1_time()) line += std::format("; lap 1 {:.2f} s", *lap1);
        if (const auto best = r.best_racing_lap()) line += std::format(", best {:.2f} s", *best);
        if (!r.stopped) line += "; did not come to rest";
    } else {
        line = std::format("DNF after {} of {} laps: {}", r.lap_times.size(), r.laps_target,
                           r.dnf_reason);
    }
    return line + std::format(" ({:.1f} s simulated in {:.1f} s)", r.sim_time, r.wall_time);
}

void set_verbosity(const Options& options, LogLevel normal) {
    set_log_level(options.has("quiet") ? LogLevel::Warn : normal);
}

int run_run(std::span<const std::string_view> args) {
    const Options options(args, {"config", "out", "log"}, {"quiet"});
    const RunConfig config = read_run_config(options.required("config"));
    const std::string out = options.required("out");
    set_verbosity(options, LogLevel::Info);

    std::ofstream csv;
    std::optional<runner::CsvLog> log;
    if (const auto path = options.get("log")) {
        if (std::filesystem::path(*path).has_parent_path()) {
            std::filesystem::create_directories(std::filesystem::path(*path).parent_path());
        }
        csv.open(*path);
        if (!csv) throw std::runtime_error("cannot write " + *path);
        log.emplace(csv, config.log_period);
    }
    const RunResult result = runner::run_stack(config, log ? &*log : nullptr);
    write_file(out, json(result).dump(2) + "\n");
    std::cout << (config.name.empty() ? "run" : config.name) << ": " << outcome(result) << "\n";
    return 0;
}

int run_sweep(std::span<const std::string_view> args) {
    const Options options(args, {"config", "out", "threads"}, {"quiet", "list"});
    const std::filesystem::path config_path = options.required("config");
    runner::SweepConfig sweep;
    std::vector<runner::SweepRun> runs;
    try {
        sweep = read_json(config_path).get<runner::SweepConfig>();
        runs = runner::expand_sweep(sweep);
    } catch (const std::exception& e) {
        throw std::runtime_error(config_path.string() + ": " + e.what());
    }
    if (options.has("list")) {
        for (const runner::SweepRun& run : runs) std::cout << run.config.name << "\n";
        std::cout << std::format("{}: {} runs\n", sweep.name.empty() ? "sweep" : sweep.name,
                                 runs.size());
        return 0;
    }
    const std::filesystem::path out = options.required("out");
    unsigned threads = sweep.threads;
    if (const auto requested = options.get_unsigned("threads")) {
        threads = static_cast<unsigned>(*requested);
    }
    if (threads == 0) threads = std::max(std::thread::hardware_concurrency(), 1U);
    // Mission logs from several runs at once would interleave; keep to warnings.
    set_log_level(LogLevel::Warn);
    const bool quiet = options.has("quiet");

    std::cerr << std::format("{}: {} runs on {} threads\n",
                             sweep.name.empty() ? "sweep" : sweep.name, runs.size(),
                             std::min<std::size_t>(threads, runs.size()));
    std::size_t done = 0;
    const std::vector<RunResult> results = runner::run_parallel(
        runs, [](const RunConfig& c) { return runner::run_stack(c); }, threads,
        [&](std::size_t index, const RunResult& result) {
            ++done;
            if (!quiet) {
                std::cerr << std::format("[{}/{}] {}: {}\n", done, runs.size(),
                                         runs[index].config.name, outcome(result));
            }
        });

    json all = json::array();
    for (std::size_t i = 0; i < runs.size(); ++i) {
        all.push_back({{"variant", runs[i].variant},
                       {"track", runs[i].track},
                       {"seed", runs[i].seed},
                       {"result", results[i]}});
    }
    const std::vector<runner::VariantSummary> summaries =
        runner::summarize(runs, results, sweep.oracle_variant);
    const std::string title = sweep.name.empty() ? "Sweep" : sweep.name;
    write_file(out / "sweep.json", json(sweep).dump(2) + "\n");
    write_file(out / "runs.json", all.dump(1) + "\n");
    write_file(out / "summary.json", json(summaries).dump(2) + "\n");
    write_file(out / "summary.md", runner::summary_markdown(summaries, title));
    std::cout << runner::summary_markdown(summaries, title);
    return 0;
}

int run_replay(std::span<const std::string_view> args) {
    const Options options(args, {"config", "out"}, {"quiet"});
    const RunConfig config = read_run_config(options.required("config"));
    const std::string out = options.required("out");
    set_verbosity(options, LogLevel::Info);
    runner::ReplayRecorder recorder;
    const RunResult result = runner::run_stack(config, &recorder);
    json replay = recorder.replay();
    replay["config"] = config;
    replay["result"] = runner::deterministic_json(result);
    write_file(out, replay.dump() + "\n");
    std::cout << (config.name.empty() ? "run" : config.name) << ": " << outcome(result) << "\n";
    return 0;
}

}  // namespace

const Command kRunCommand{"run", "race the stack on one track in the simulator",
                          R"(apex run: race the stack on one track in the simulator.

Usage:
  apex run --config run.json --out result.json [--log run.csv] [--quiet]

Options:
  --config PATH   the run: track, true and nominal car, noise, latencies, oracles, module
                  configs (see docs/evaluation.md; every key is optional)
  --out PATH      where to write the result (JSON: lap times, penalties, DNF reason, map,
                  localization, association and tracking errors, compute times)
  --log PATH      also write a CSV time series, one row per log_period of simulated time
  --quiet         only print warnings, not the mission's progress

A run that does not finish still exits with status 0; its result says why.
)",
                          run_run};

const Command kSweepCommand{"sweep", "run an experiment: many runs in parallel, summarized",
                            R"(apex sweep: run an experiment: many runs in parallel, summarized.

Usage:
  apex sweep --config sweep.json --out DIR [--threads N] [--quiet]
  apex sweep --config sweep.json --list

Options:
  --config PATH   the sweep: a base run, the tracks and seeds to cross it with, and the
                  variants to compare, each a JSON merge patch on the base
  --out DIR       where to write sweep.json (the config), runs.json (every result),
                  summary.json and summary.md (per-variant tables)
  --threads N     runs at a time (default: the config's "threads", else one per core); each
                  run is single-threaded and its result does not depend on N
  --quiet         do not print a line per finished run
  --list          validate the sweep and print its runs without running them
)",
                            run_sweep};

const Command kReplayCommand{"replay", "record one run for an animation",
                             R"(apex replay: record one run for an animation.

Usage:
  apex replay --config run.json --out replay.json [--quiet]

Writes the track, the car's true and estimated pose every 50 ms, the map as it is built and
the reference paths the controller tracked, with the run's config and result.
)",
                             run_replay};

}  // namespace apex::cli
