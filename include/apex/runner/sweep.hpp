#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/runner/config.hpp"
#include "apex/runner/result.hpp"

// Experiment sweeps (docs/design.md, section 8): the same base run repeated over tracks, seeds
// and variants of the configuration, run in parallel and summarized.
namespace apex::runner {

// One arm of an experiment: a name and the changes it makes to the base run, as a JSON merge
// patch (RFC 7386) on the base's JSON form. {"stack": {"controller": {"kind": "stanley"}}}
// changes the controller and leaves everything else as the base has it.
struct SweepVariant {
    std::string name;
    nlohmann::json patch = nlohmann::json::object();
};

// JSON:
//   {"name": "controllers",
//    "base": {...a RunConfig...},
//    "tracks": [{"seed": 1}, {"layout": "technical"}],   default: the base's track
//    "seeds": [1, 2, 3],                                  default: the base's seed
//    "variants": [{"name": "mpc", "patch": {...}}, ...],   default: the base alone
//    "oracle_variant": "oracle",                          optional, for the gap to the oracle
//    "threads": 4}                                        0: one per core
struct SweepConfig {
    std::string name;
    nlohmann::json base = nlohmann::json::object();
    std::vector<TrackSpec> tracks;
    std::vector<std::uint64_t> seeds;
    std::vector<SweepVariant> variants;
    // The variant every other one's racing laps are compared with, run for run (same track and
    // seed): usually one with both oracles on. Empty for none.
    std::string oracle_variant;
    unsigned threads = 0;
};

void to_json(nlohmann::json& j, const SweepConfig& config);
void from_json(const nlohmann::json& j, SweepConfig& config);

// One run of a sweep and where it sits in the experiment.
struct SweepRun {
    std::string variant;
    std::string track;  // track_label of its track
    std::uint64_t seed = 0;
    RunConfig config;
};

// A short, file-name-safe name for a track spec: "gen7", the layout's name, the file's stem,
// "oval".
[[nodiscard]] std::string track_label(const TrackSpec& spec);

// Every run of the sweep, variants outermost, then tracks, then seeds. Each run's config is the
// base patched by its variant, with the track and seed set and the name
// "<variant>/<track>/s<seed>". Throws std::invalid_argument naming the run for a config that
// does not parse or validate, for duplicate variant names, or for an oracle_variant that is not
// one of the variants.
[[nodiscard]] std::vector<SweepRun> expand_sweep(const SweepConfig& config);

using RunFunction = std::function<RunResult(const RunConfig&)>;
// Called once per finished run, from the worker thread that ran it, one call at a time.
using RunCallback = std::function<void(std::size_t index, const RunResult& result)>;

// Runs every config on a pool of `threads` workers (0: one per hardware thread; never more
// than there are runs), one run per worker at a time, and returns the results in the order of
// `runs`. Runs share nothing, so each result is the same whatever the thread count. A run that
// throws gives a result with `error` set instead of stopping the sweep.
[[nodiscard]] std::vector<RunResult> run_parallel(const std::vector<SweepRun>& runs,
                                                  const RunFunction& run, unsigned threads,
                                                  const RunCallback& on_done = {});

}  // namespace apex::runner
