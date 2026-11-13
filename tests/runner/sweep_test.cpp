#include "apex/runner/sweep.hpp"

#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/rng.hpp"

namespace apex::runner {
namespace {

using nlohmann::json;

SweepConfig two_by_two_by_three() {
    return json{
        {"name", "test"},
        {"base", {{"vehicle", {{"mass", 250.0}}}, {"stack", {{"mission", {{"laps", 3}}}}}}},
        {"tracks", {{{"seed", 5}}, {{"layout", "technical"}}}},
        {"seeds", {1, 2, 3}},
        {"variants",
         {{{"name", "a"}},
          {{"name", "b"},
           {"patch", {{"vehicle", {{"mass", 260.0}}}, {"stack", {{"mission", {{"laps", 2}}}}}}}}}}}
        .get<SweepConfig>();
}

// A stand-in for a closed-loop run: some arithmetic whose result depends only on the config.
RunResult fake_run(const RunConfig& config) {
    RunResult r;
    r.name = config.name;
    r.seed = config.seed;
    Rng rng = Rng(config.seed).split(config.track.seed);
    double lap = 20.0;
    for (int i = 0; i < 20000; ++i) lap += 1e-5 * rng.normal();
    for (int i = 0; i < config.stack.mission.laps; ++i) r.lap_times.push_back(lap + i);
    r.completed = true;
    return r;
}

TEST(SweepConfig, JsonRoundTrip) {
    const SweepConfig config = two_by_two_by_three();
    EXPECT_EQ(config.name, "test");
    ASSERT_EQ(config.variants.size(), 2U);
    EXPECT_TRUE(config.variants[0].patch.empty());
    const SweepConfig back = json(config).get<SweepConfig>();
    EXPECT_EQ(json(back), json(config));
}

TEST(SweepConfig, RejectsMalformedInput) {
    EXPECT_THROW((void)json({{"runs", 3}}).get<SweepConfig>(), std::invalid_argument);
    EXPECT_THROW((void)json({{"base", 3}}).get<SweepConfig>(), std::invalid_argument);
    EXPECT_THROW((void)json({{"variants", {{{"name", "a"}, {"patch", 1}}}}}).get<SweepConfig>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json({{"variants", {{{"name", "a"}, {"overrides", json::object()}}}}})
                     .get<SweepConfig>(),
                 std::invalid_argument);
}

TEST(ExpandSweep, VariantsThenTracksThenSeeds) {
    const std::vector<SweepRun> runs = expand_sweep(two_by_two_by_three());
    ASSERT_EQ(runs.size(), 12U);
    EXPECT_EQ(runs[0].config.name, "a/gen5/s1");
    EXPECT_EQ(runs[2].config.name, "a/gen5/s3");
    EXPECT_EQ(runs[3].config.name, "a/technical/s1");
    EXPECT_EQ(runs[6].config.name, "b/gen5/s1");
    EXPECT_EQ(runs[11].variant, "b");
    EXPECT_EQ(runs[11].track, "technical");
    EXPECT_EQ(runs[11].seed, 3U);
    EXPECT_EQ(runs[11].config.seed, 3U);
    EXPECT_EQ(runs[11].config.track.source, TrackSpec::Source::Layout);
    std::set<std::string> names;
    for (const SweepRun& run : runs) names.insert(run.config.name);
    EXPECT_EQ(names.size(), runs.size());
}

TEST(ExpandSweep, PatchesTheBase) {
    const std::vector<SweepRun> runs = expand_sweep(two_by_two_by_three());
    // Variant a is the base: three laps, a 250 kg car that the software knows.
    EXPECT_EQ(runs[0].config.stack.mission.laps, 3);
    EXPECT_DOUBLE_EQ(runs[0].config.vehicle.mass, 250.0);
    EXPECT_DOUBLE_EQ(runs[0].config.nominal.mass, 250.0);
    // Variant b changes the laps and the car; the nominal model still follows the true car,
    // since neither the base nor the patch sets it.
    EXPECT_EQ(runs[6].config.stack.mission.laps, 2);
    EXPECT_DOUBLE_EQ(runs[6].config.vehicle.mass, 260.0);
    EXPECT_DOUBLE_EQ(runs[6].config.nominal.mass, 260.0);
}

TEST(ExpandSweep, DefaultsToTheBaseAlone) {
    const SweepConfig config =
        json{{"base", {{"seed", 9}, {"track", {{"seed", 4}}}}}}.get<SweepConfig>();
    const std::vector<SweepRun> runs = expand_sweep(config);
    ASSERT_EQ(runs.size(), 1U);
    EXPECT_EQ(runs[0].config.name, "base/gen4/s9");
}

TEST(ExpandSweep, ErrorsNameTheCulprit) {
    SweepConfig config = two_by_two_by_three();
    config.variants[1].patch = {{"stack", {{"mission", {{"lap", 2}}}}}};
    try {
        (void)expand_sweep(config);
        FAIL() << "expected a throw";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("'b'"), std::string::npos) << e.what();
    }

    config = two_by_two_by_three();
    config.variants[1].patch = {{"latency", {{"lidar", 0.0505}}}};
    EXPECT_THROW((void)expand_sweep(config), std::invalid_argument);

    config = two_by_two_by_three();
    config.variants[1].name = "a";
    EXPECT_THROW((void)expand_sweep(config), std::invalid_argument);

    config = two_by_two_by_three();
    config.oracle_variant = "oracle";
    EXPECT_THROW((void)expand_sweep(config), std::invalid_argument);
}

TEST(TrackLabel, EachSource) {
    EXPECT_EQ(track_label(json({{"seed", 12}}).get<TrackSpec>()), "gen12");
    EXPECT_EQ(track_label(json({{"layout", "mixed"}}).get<TrackSpec>()), "mixed");
    EXPECT_EQ(track_label(json({{"file", "tracks/fsg.json"}}).get<TrackSpec>()), "fsg");
    EXPECT_EQ(track_label(TrackSpec{}), "oval");
}

TEST(RunParallel, ResultsIndependentOfThreadCount) {
    const std::vector<SweepRun> runs = expand_sweep(two_by_two_by_three());
    const std::vector<RunResult> serial = run_parallel(runs, fake_run, 1);
    ASSERT_EQ(serial.size(), runs.size());
    for (const unsigned threads : {2U, 5U, 64U}) {
        const std::vector<RunResult> parallel = run_parallel(runs, fake_run, threads);
        ASSERT_EQ(parallel.size(), runs.size());
        for (std::size_t i = 0; i < runs.size(); ++i) {
            EXPECT_EQ(parallel[i].name, runs[i].config.name);
            EXPECT_EQ(deterministic_json(parallel[i]), deterministic_json(serial[i]));
        }
    }
}

TEST(RunParallel, CallbackOncePerRunAndErrorsAreResults) {
    const std::vector<SweepRun> runs = expand_sweep(two_by_two_by_three());
    const RunFunction flaky = [](const RunConfig& config) {
        if (config.seed == 2) throw std::runtime_error("boom");
        return fake_run(config);
    };
    std::vector<int> calls(runs.size(), 0);
    const std::vector<RunResult> results =
        run_parallel(runs, flaky, 4, [&](std::size_t index, const RunResult&) { ++calls[index]; });
    for (std::size_t i = 0; i < runs.size(); ++i) {
        EXPECT_EQ(calls[i], 1);
        EXPECT_EQ(results[i].name, runs[i].config.name);
        if (runs[i].seed == 2) {
            EXPECT_EQ(results[i].error, "boom");
            EXPECT_FALSE(results[i].completed);
        } else {
            EXPECT_TRUE(results[i].error.empty());
        }
    }
}

TEST(RunParallel, NoRuns) {
    EXPECT_TRUE(run_parallel({}, fake_run, 4).empty());
}

}  // namespace
}  // namespace apex::runner
