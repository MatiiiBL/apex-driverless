#include "apex/runner/summary.hpp"

#include <algorithm>
#include <format>
#include <set>
#include <utility>

#include <nlohmann/json.hpp>

#include "apex/core/angles.hpp"

namespace apex::runner {

using nlohmann::json;

namespace {

class Mean {
public:
    void add(double value) {
        sum_ += value;
        ++count_;
    }
    void add(const std::optional<double>& value) {
        if (value) add(*value);
    }
    [[nodiscard]] std::optional<double> value() const {
        if (count_ == 0) return std::nullopt;
        return sum_ / static_cast<double>(count_);
    }

private:
    double sum_ = 0.0;
    std::size_t count_ = 0;
};

const PhaseMetrics& phase(const RunResult& r, stack::MissionState state) {
    return r.phases[stack::index_of(state)];
}

std::optional<double> rms_if_any(const ErrorStats& stats) {
    if (stats.samples == 0) return std::nullopt;
    return stats.rms;
}

json optional_json(const std::optional<double>& value) {
    return value ? json(*value) : json(nullptr);
}

std::string number(const std::optional<double>& value, int decimals) {
    if (!value) return "–";
    const std::string text = std::format("{:.{}f}", *value, decimals);
    // A value that rounds to zero is shown as zero, not as "-0.00".
    if (text.starts_with('-') && text.find_first_not_of("-0.") == std::string::npos) {
        return text.substr(1);
    }
    return text;
}

std::string ratio(std::size_t part, std::size_t whole) {
    return std::format("{}/{}", part, whole);
}

// Everything summarize() accumulates for one variant before taking the means.
struct Accumulator {
    VariantSummary summary;
    Mean lap1, racing_mean, cones, off_course, penalties, gaps;
    Mean map_rms, map_coverage, loc_lap1, loc_racing, yaw_racing, tracking;
    std::optional<double> best;
    std::uint64_t matched = 0, correct = 0, matchable = 0;
    struct Compute {
        Mean p50;
        double p99 = 0.0;
    };
    std::map<std::string, Compute> compute;

    void add(const RunResult& r, const std::optional<double>& oracle_racing_mean) {
        VariantSummary& s = summary;
        ++s.runs;
        if (!r.error.empty()) {
            ++s.errors;
            ++s.dnf_reasons["error: " + r.error];
            return;
        }
        if (r.completed) ++s.completed;
        if (r.completed && r.stopped) ++s.stopped;
        if (!r.dnf_reason.empty()) ++s.dnf_reasons[r.dnf_reason];

        lap1.add(r.lap1_time());
        racing_mean.add(r.mean_racing_lap());
        if (const auto b = r.best_racing_lap()) best = std::min(best.value_or(*b), *b);
        cones.add(static_cast<double>(r.cones_hit));
        off_course.add(static_cast<double>(r.off_course));
        penalties.add(r.penalty_seconds);
        if (r.completed && oracle_racing_mean && r.mean_racing_lap()) {
            gaps.add(*r.mean_racing_lap() - *oracle_racing_mean);
            ++s.gap_pairs;
        }

        if (r.map.matched > 0) map_rms.add(r.map.position.rms);
        if (r.map.cones > 0) map_coverage.add(r.map.coverage);
        loc_lap1.add(rms_if_any(phase(r, stack::MissionState::Exploring).position));
        loc_racing.add(rms_if_any(phase(r, stack::MissionState::Racing).position));
        yaw_racing.add(rms_if_any(phase(r, stack::MissionState::Racing).yaw));
        tracking.add(rms_if_any(phase(r, stack::MissionState::Racing).lateral));
        s.max_lateral_true = std::max(s.max_lateral_true, r.max_lateral_true);
        matched += r.association.matched;
        correct += r.association.correct;
        matchable += r.association.matchable;

        for (const auto& [module, stats] : r.compute.items()) {
            if (!stats.is_object() || !stats.contains("p99_us") || !stats.contains("p50_us") ||
                stats.value("count", 0) == 0) {
                continue;
            }
            Compute& c = compute[module];
            c.p50.add(stats.at("p50_us").get<double>());
            c.p99 = std::max(c.p99, stats.at("p99_us").get<double>());
        }
    }

    VariantSummary finish() {
        VariantSummary s = std::move(summary);
        s.lap1 = lap1.value();
        s.racing_mean = racing_mean.value();
        s.racing_best = best;
        s.cones_hit = cones.value().value_or(0.0);
        s.off_course = off_course.value().value_or(0.0);
        s.penalty_seconds = penalties.value().value_or(0.0);
        s.gap_to_oracle = gaps.value();
        s.map_rms = map_rms.value();
        s.map_coverage = map_coverage.value();
        s.localization_lap1 = loc_lap1.value();
        s.localization_racing = loc_racing.value();
        s.yaw_racing = yaw_racing.value();
        s.tracking_racing = tracking.value();
        s.association_precision =
            matched == 0 ? 0.0 : static_cast<double>(correct) / static_cast<double>(matched);
        s.association_recall =
            matchable == 0 ? 0.0 : static_cast<double>(correct) / static_cast<double>(matchable);
        for (const auto& [module, c] : compute) {
            s.compute[module] = {c.p50.value().value_or(0.0), c.p99};
        }
        return s;
    }
};

}  // namespace

void to_json(json& j, const VariantSummary& s) {
    json compute = json::object();
    for (const auto& [module, c] : s.compute) {
        compute[module] = {{"p50_us", c.p50_us}, {"p99_us", c.p99_us}};
    }
    j = {{"variant", s.variant},
         {"runs", s.runs},
         {"completed", s.completed},
         {"stopped", s.stopped},
         {"errors", s.errors},
         {"dnf_reasons", s.dnf_reasons},
         {"lap1", optional_json(s.lap1)},
         {"racing_mean", optional_json(s.racing_mean)},
         {"racing_best", optional_json(s.racing_best)},
         {"cones_hit", s.cones_hit},
         {"off_course", s.off_course},
         {"penalty_seconds", s.penalty_seconds},
         {"gap_to_oracle", optional_json(s.gap_to_oracle)},
         {"gap_pairs", s.gap_pairs},
         {"map_rms", optional_json(s.map_rms)},
         {"map_coverage", optional_json(s.map_coverage)},
         {"localization_lap1", optional_json(s.localization_lap1)},
         {"localization_racing", optional_json(s.localization_racing)},
         {"yaw_racing", optional_json(s.yaw_racing)},
         {"tracking_racing", optional_json(s.tracking_racing)},
         {"max_lateral_true", s.max_lateral_true},
         {"association_precision", s.association_precision},
         {"association_recall", s.association_recall},
         {"compute", std::move(compute)}};
}

std::vector<VariantSummary> summarize(const std::vector<SweepRun>& runs,
                                      const std::vector<RunResult>& results,
                                      std::string_view oracle_variant) {
    // The oracle's mean racing lap per (track, seed), from its completed runs.
    std::map<std::pair<std::string, std::uint64_t>, double> oracle;
    for (std::size_t i = 0; i < runs.size() && i < results.size(); ++i) {
        const auto mean = results[i].mean_racing_lap();
        if (runs[i].variant == oracle_variant && results[i].completed && mean) {
            oracle[{runs[i].track, runs[i].seed}] = *mean;
        }
    }

    std::vector<std::string> order;
    std::map<std::string, Accumulator> by_variant;
    for (std::size_t i = 0; i < runs.size() && i < results.size(); ++i) {
        const SweepRun& run = runs[i];
        auto [it, inserted] = by_variant.try_emplace(run.variant);
        if (inserted) {
            order.push_back(run.variant);
            it->second.summary.variant = run.variant;
        }
        std::optional<double> reference;
        if (!oracle_variant.empty() && run.variant != oracle_variant) {
            if (const auto o = oracle.find({run.track, run.seed}); o != oracle.end()) {
                reference = o->second;
            }
        }
        it->second.add(results[i], reference);
    }

    std::vector<VariantSummary> summaries;
    summaries.reserve(order.size());
    for (const std::string& variant : order) summaries.push_back(by_variant[variant].finish());
    return summaries;
}

std::string summary_markdown(const std::vector<VariantSummary>& summaries, std::string_view title) {
    std::string md = std::format("## {}\n\n", title);

    md +=
        "| Variant | Completed | Lap 1 (s) | Racing lap, mean (s) | Best lap (s) | Gap to oracle "
        "(s) | Cones / run | Off-course / run | Penalties (s / run) |\n"
        "|---|---|---|---|---|---|---|---|---|\n";
    for (const VariantSummary& s : summaries) {
        md += std::format("| {} | {} | {} | {} | {} | {} | {:.2f} | {:.2f} | {:.1f} |\n", s.variant,
                          ratio(s.completed, s.runs), number(s.lap1, 2), number(s.racing_mean, 2),
                          number(s.racing_best, 2), number(s.gap_to_oracle, 2), s.cones_hit,
                          s.off_course, s.penalty_seconds);
    }

    md +=
        "\n| Variant | Map RMS (m) | Map coverage | Localization RMS, lap 1 (m) | Localization "
        "RMS, racing (m) | Yaw RMS, racing (deg) | Association precision | Association recall "
        "| Tracking RMS, racing (m) | Max offset from centerline (m) |\n"
        "|---|---|---|---|---|---|---|---|---|---|\n";
    for (const VariantSummary& s : summaries) {
        const std::optional<double> yaw_deg =
            s.yaw_racing ? std::optional<double>(*s.yaw_racing * 180.0 / kPi) : std::nullopt;
        md += std::format("| {} | {} | {} | {} | {} | {} | {:.3f} | {:.3f} | {} | {:.2f} |\n",
                          s.variant, number(s.map_rms, 3), number(s.map_coverage, 3),
                          number(s.localization_lap1, 3), number(s.localization_racing, 3),
                          number(yaw_deg, 2), s.association_precision, s.association_recall,
                          number(s.tracking_racing, 3), s.max_lateral_true);
    }

    std::set<std::string> modules;
    for (const VariantSummary& s : summaries) {
        for (const auto& entry : s.compute) modules.insert(entry.first);
    }
    if (!modules.empty()) {
        md += "\nCompute time per call, p50 / worst p99 (µs):\n\n| Module |";
        std::string rule = "|---|";
        for (const VariantSummary& s : summaries) {
            md += std::format(" {} |", s.variant);
            rule += "---|";
        }
        md += "\n" + rule + "\n";
        for (const std::string& module : modules) {
            md += std::format("| {} |", module);
            for (const VariantSummary& s : summaries) {
                const auto it = s.compute.find(module);
                md += it == s.compute.end()
                          ? std::string(" – |")
                          : std::format(" {:.0f} / {:.0f} |", it->second.p50_us, it->second.p99_us);
            }
            md += "\n";
        }
    }

    bool any_dnf = false;
    for (const VariantSummary& s : summaries) {
        for (const auto& [reason, count] : s.dnf_reasons) {
            if (!any_dnf) md += "\nRuns that did not finish:\n\n";
            any_dnf = true;
            md += std::format("- {}: {} ({} run{})\n", s.variant, reason, count,
                              count == 1 ? "" : "s");
        }
    }
    return md;
}

}  // namespace apex::runner
