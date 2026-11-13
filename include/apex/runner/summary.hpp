#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/runner/result.hpp"
#include "apex/runner/sweep.hpp"

// The headline numbers of a sweep, per variant (docs/design.md, section 8).
namespace apex::runner {

// Wall-clock compute time of one module over a variant's runs.
struct ComputeSummary {
    double p50_us = 0.0;  // mean over the runs of each run's median
    double p99_us = 0.0;  // worst over the runs of each run's 99th percentile
};

// Aggregates over the runs of one variant. Means are taken over the runs that have the value
// (a run that crashed on lap 1 has no racing laps), and are empty when none does.
struct VariantSummary {
    std::string variant;
    std::size_t runs = 0;
    std::size_t completed = 0;                       // every lap timed with no DNF
    std::size_t stopped = 0;                         // ... and came to rest after the last one
    std::size_t errors = 0;                          // could not be set up
    std::map<std::string, std::size_t> dnf_reasons;  // reason -> runs

    std::optional<double> lap1;         // s, mean lap 1 (exploration)
    std::optional<double> racing_mean;  // s, mean of each run's mean racing lap
    std::optional<double> racing_best;  // s, the best racing lap of any run
    double cones_hit = 0.0;             // per run
    double off_course = 0.0;            // per run
    double penalty_seconds = 0.0;       // per run
    // Mean over (track, seed) pairs of this variant's mean racing lap minus the oracle
    // variant's, where both completed.
    std::optional<double> gap_to_oracle;
    std::size_t gap_pairs = 0;

    std::optional<double> map_rms;              // m, landmark to true cone
    std::optional<double> map_coverage;         // share of true cones mapped
    std::optional<double> localization_lap1;    // m, position RMS while exploring
    std::optional<double> localization_racing;  // m, position RMS while racing
    std::optional<double> yaw_racing;           // rad
    std::optional<double> tracking_racing;      // m, lateral RMS from the raceline
    double max_lateral_true = 0.0;  // m, worst true distance from the centerline of any run
    // Pooled over the runs: sum of correct over sum of matched (matchable).
    double association_precision = 0.0;
    double association_recall = 0.0;

    std::map<std::string, ComputeSummary> compute;  // by module
};

void to_json(nlohmann::json& j, const VariantSummary& summary);

// One summary per variant, in the order the variants first appear in `runs`. `results[i]` is
// the result of `runs[i]`.
[[nodiscard]] std::vector<VariantSummary> summarize(const std::vector<SweepRun>& runs,
                                                    const std::vector<RunResult>& results,
                                                    std::string_view oracle_variant);

// Markdown tables of the summaries (race, estimation, compute) and the reasons runs did not
// finish, under the heading `title`.
[[nodiscard]] std::string summary_markdown(const std::vector<VariantSummary>& summaries,
                                           std::string_view title);

}  // namespace apex::runner
