#include "apex/runner/summary.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace apex::runner {
namespace {

using stack::MissionState;

SweepRun sweep_run(std::string variant, std::string track, std::uint64_t seed) {
    SweepRun run;
    run.variant = std::move(variant);
    run.track = std::move(track);
    run.seed = seed;
    return run;
}

RunResult finished(std::vector<double> laps) {
    RunResult r;
    r.lap_times = std::move(laps);
    r.completed = true;
    r.stopped = true;
    return r;
}

TEST(Summarize, RaceNumbersAndGapToOracle) {
    const std::vector<SweepRun> runs = {
        sweep_run("mpc", "t1", 1),    sweep_run("mpc", "t1", 2),    sweep_run("mpc", "t2", 1),
        sweep_run("oracle", "t1", 1), sweep_run("oracle", "t1", 2), sweep_run("oracle", "t2", 1),
    };
    std::vector<RunResult> results = {
        finished({40.0, 22.0, 24.0}),  // racing mean 23
        finished({42.0, 21.0, 21.0}),  // 21
        RunResult{},                   // filled below: a DNF on lap 1
        finished({38.0, 20.0, 20.0}),  // 20
        finished({38.0, 19.0, 19.0}),  // 19
        finished({30.0, 18.0, 18.0}),  // the pair of a DNF: no gap
    };
    results[0].cones_hit = 2;
    results[0].penalty_seconds = 4.0;
    results[1].off_course = 1;
    results[1].penalty_seconds = 10.0;
    results[2].dnf_reason = "off the track by 5.2 m";
    results[2].lap_times = {};

    const std::vector<VariantSummary> s = summarize(runs, results, "oracle");
    ASSERT_EQ(s.size(), 2U);
    const VariantSummary& mpc = s[0];
    EXPECT_EQ(mpc.variant, "mpc");
    EXPECT_EQ(mpc.runs, 3U);
    EXPECT_EQ(mpc.completed, 2U);
    EXPECT_EQ(mpc.stopped, 2U);
    EXPECT_DOUBLE_EQ(*mpc.lap1, 41.0);
    EXPECT_DOUBLE_EQ(*mpc.racing_mean, 22.0);
    EXPECT_DOUBLE_EQ(*mpc.racing_best, 21.0);
    EXPECT_DOUBLE_EQ(mpc.cones_hit, 2.0 / 3.0);
    EXPECT_DOUBLE_EQ(mpc.off_course, 1.0 / 3.0);
    EXPECT_DOUBLE_EQ(mpc.penalty_seconds, 14.0 / 3.0);
    // (23 - 20) and (21 - 19); the t2 pair has no mpc racing laps.
    EXPECT_EQ(mpc.gap_pairs, 2U);
    EXPECT_DOUBLE_EQ(*mpc.gap_to_oracle, 2.5);
    EXPECT_EQ(mpc.dnf_reasons.at("off the track by 5.2 m"), 1U);

    const VariantSummary& oracle = s[1];
    EXPECT_FALSE(oracle.gap_to_oracle.has_value());
    EXPECT_EQ(oracle.gap_pairs, 0U);
    EXPECT_TRUE(oracle.dnf_reasons.empty());
}

TEST(Summarize, EstimationAndCompute) {
    const std::vector<SweepRun> runs = {sweep_run("a", "t", 1), sweep_run("a", "t", 2),
                                        sweep_run("a", "t", 3)};
    std::vector<RunResult> results(3);
    results[0].map.cones = 100;
    results[0].map.matched = 90;
    results[0].map.coverage = 0.9;
    results[0].map.position = {90, 0.2, 0.5};
    results[1].map.cones = 100;
    results[1].map.matched = 100;
    results[1].map.coverage = 1.0;
    results[1].map.position = {100, 0.1, 0.3};
    results[0].phases[stack::index_of(MissionState::Racing)].position = {10, 0.3, 0.6};
    results[0].phases[stack::index_of(MissionState::Racing)].lateral = {10, 0.2, 0.4};
    results[1].phases[stack::index_of(MissionState::Exploring)].position = {10, 0.1, 0.2};
    results[0].max_lateral_true = 1.1;
    results[1].max_lateral_true = 1.4;
    results[0].association = {100, 80, 78, 90};
    results[1].association = {100, 20, 18, 30};
    results[0].compute = {{"controller", {{"count", 5}, {"p50_us", 100.0}, {"p99_us", 300.0}}},
                          {"realtime_factor", 12.0}};
    results[1].compute = {{"controller", {{"count", 5}, {"p50_us", 200.0}, {"p99_us", 250.0}}},
                          {"perception", {{"count", 0}, {"p50_us", 0.0}, {"p99_us", 0.0}}}};
    results[2].error = "track: unknown layout";

    const std::vector<VariantSummary> s = summarize(runs, results, "");
    ASSERT_EQ(s.size(), 1U);
    const VariantSummary& a = s[0];
    EXPECT_EQ(a.errors, 1U);
    EXPECT_EQ(a.dnf_reasons.at("error: track: unknown layout"), 1U);
    EXPECT_DOUBLE_EQ(*a.map_rms, 0.15);
    EXPECT_DOUBLE_EQ(*a.map_coverage, 0.95);
    EXPECT_DOUBLE_EQ(*a.localization_racing, 0.3);
    EXPECT_DOUBLE_EQ(*a.localization_lap1, 0.1);
    EXPECT_DOUBLE_EQ(*a.tracking_racing, 0.2);
    EXPECT_DOUBLE_EQ(a.max_lateral_true, 1.4);
    EXPECT_DOUBLE_EQ(a.association_precision, 96.0 / 100.0);
    EXPECT_DOUBLE_EQ(a.association_recall, 96.0 / 120.0);
    ASSERT_EQ(a.compute.size(), 1U);  // modules that never ran are left out
    EXPECT_DOUBLE_EQ(a.compute.at("controller").p50_us, 150.0);
    EXPECT_DOUBLE_EQ(a.compute.at("controller").p99_us, 300.0);
    EXPECT_FALSE(a.lap1.has_value());
}

TEST(SummaryMarkdown, TablesAndFailures) {
    const std::vector<SweepRun> runs = {sweep_run("mpc", "t", 1), sweep_run("stanley", "t", 1)};
    std::vector<RunResult> results = {finished({40.0, 21.5}), RunResult{}};
    results[0].compute = {{"controller", {{"count", 5}, {"p50_us", 120.0}, {"p99_us", 480.0}}}};
    results[1].dnf_reason = "stalled for 10 s";
    const std::string md = summary_markdown(summarize(runs, results, ""), "Controllers");
    EXPECT_NE(md.find("## Controllers"), std::string::npos);
    EXPECT_NE(md.find("| mpc | 1/1 | 40.00 | 21.50 | 21.50 | – |"), std::string::npos) << md;
    EXPECT_NE(md.find("| stanley | 0/1 | – |"), std::string::npos) << md;
    EXPECT_NE(md.find("| controller | 120 / 480 | – |"), std::string::npos) << md;
    EXPECT_NE(md.find("- stanley: stalled for 10 s (1 run)"), std::string::npos) << md;

    const nlohmann::json j = summarize(runs, results, "")[0];
    EXPECT_EQ(j["completed"], 1);
    EXPECT_TRUE(j["gap_to_oracle"].is_null());
    EXPECT_DOUBLE_EQ(j["compute"]["controller"]["p99_us"].get<double>(), 480.0);
}

}  // namespace
}  // namespace apex::runner
