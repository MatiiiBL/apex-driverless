#include "apex/runner/result.hpp"

#include <gtest/gtest.h>

namespace apex::runner {
namespace {

TEST(RunResult, LapSummaries) {
    RunResult r;
    EXPECT_FALSE(r.lap1_time().has_value());
    EXPECT_FALSE(r.best_racing_lap().has_value());
    r.lap_times = {40.0};
    EXPECT_DOUBLE_EQ(*r.lap1_time(), 40.0);
    EXPECT_FALSE(r.mean_racing_lap().has_value());
    r.lap_times = {40.0, 22.0, 21.0, 23.0};
    r.penalty_seconds = 4.0;
    EXPECT_DOUBLE_EQ(*r.best_racing_lap(), 21.0);
    EXPECT_DOUBLE_EQ(*r.mean_racing_lap(), 22.0);
    EXPECT_DOUBLE_EQ(r.total_time(), 106.0);
    EXPECT_DOUBLE_EQ(r.score(), 110.0);
}

TEST(RunResult, Json) {
    RunResult r;
    r.name = "oval";
    r.lap_times = {40.0, 22.0};
    r.final_state = stack::MissionState::Finished;
    r.transitions.push_back(
        {1.0, stack::MissionState::Staged, stack::MissionState::Exploring, "go"});
    r.phases[stack::index_of(stack::MissionState::Racing)].duration = 22.0;
    r.association.matched = 4;
    r.association.correct = 3;
    r.wall_time = 1.5;
    r.compute["controller"] = {{"p99_us", 100.0}};
    const nlohmann::json j = r;
    EXPECT_EQ(j["final_state"], "finished");
    EXPECT_EQ(j["transitions"][0]["to"], "exploring");
    EXPECT_DOUBLE_EQ(j["lap1"].get<double>(), 40.0);
    EXPECT_DOUBLE_EQ(j["racing_best"].get<double>(), 22.0);
    EXPECT_TRUE(j["phases"].contains("racing"));
    EXPECT_FALSE(j["phases"].contains("exploring"));
    EXPECT_DOUBLE_EQ(j["association"]["precision"].get<double>(), 0.75);

    const nlohmann::json d = deterministic_json(r);
    EXPECT_FALSE(d.contains("wall_time"));
    EXPECT_FALSE(d.contains("compute"));
    EXPECT_EQ(d["laps"], j["laps"]);
}

TEST(RunResult, MissingLapsAreNull) {
    const nlohmann::json j = RunResult{};
    EXPECT_TRUE(j["lap1"].is_null());
    EXPECT_TRUE(j["racing_mean"].is_null());
}

}  // namespace
}  // namespace apex::runner
