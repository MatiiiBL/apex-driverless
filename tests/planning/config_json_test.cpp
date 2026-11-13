#include <stdexcept>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "apex/planning/planning.hpp"

namespace apex::planning {
namespace {

using nlohmann::json;

TEST(PlanningConfigJson, SpeedProfileRoundTripAndPartialOverride) {
    SpeedProfileConfig config;
    config.grip_factor = 0.7;
    const json j = config;
    EXPECT_DOUBLE_EQ(j.get<SpeedProfileConfig>().grip_factor, 0.7);

    const auto partial = json::parse(R"({"max_speed": 15.0})").get<SpeedProfileConfig>();
    EXPECT_DOUBLE_EQ(partial.max_speed, 15.0);
    EXPECT_DOUBLE_EQ(partial.grip_factor, SpeedProfileConfig{}.grip_factor);
    EXPECT_THROW(json::parse(R"({"grip": 0.7})").get<SpeedProfileConfig>(), std::invalid_argument);
}

TEST(PlanningConfigJson, ExplorationRoundTripAndPartialOverride) {
    ExplorationConfig config;
    config.beam_width = 20;
    config.speed.max_speed = 9.0;
    const auto back = json(config).get<ExplorationConfig>();
    EXPECT_EQ(back.beam_width, 20U);
    EXPECT_DOUBLE_EQ(back.speed.max_speed, 9.0);

    const auto partial = json::parse(R"({"color_weight": 8.0})").get<ExplorationConfig>();
    EXPECT_DOUBLE_EQ(partial.color_weight, 8.0);
    EXPECT_DOUBLE_EQ(partial.speed.grip_factor, ExplorationConfig{}.speed.grip_factor);
    EXPECT_THROW(json::parse(R"({"beam": 3})").get<ExplorationConfig>(), std::invalid_argument);
}

TEST(PlanningConfigJson, RacelineRoundTripWithNestedSpeedProfile) {
    RacelineConfig config;
    config.iterations = 5;
    config.speed.grip_factor = 0.9;
    const auto back = json(config).get<RacelineConfig>();
    EXPECT_EQ(back.iterations, 5U);
    EXPECT_DOUBLE_EQ(back.speed.grip_factor, 0.9);

    const auto partial = json::parse(R"({"speed": {"max_speed": 20.0}})").get<RacelineConfig>();
    EXPECT_DOUBLE_EQ(partial.speed.max_speed, 20.0);
    EXPECT_DOUBLE_EQ(partial.speed.grip_factor, RacelineConfig{}.speed.grip_factor);
    EXPECT_DOUBLE_EQ(partial.step, RacelineConfig{}.step);
    EXPECT_THROW(json::parse(R"({"speed": {"grip": 1.0}})").get<RacelineConfig>(),
                 std::invalid_argument);
}

TEST(PlanningConfigJson, TrackModelRoundTripAndPartialOverride) {
    TrackModelConfig config;
    config.max_gap = 20.0;
    config.passes = 3;
    const json j = config;
    const auto back = j.get<TrackModelConfig>();
    EXPECT_DOUBLE_EQ(back.max_gap, 20.0);
    EXPECT_EQ(back.passes, 3U);

    const auto partial = json::parse(R"({"fill_spacing": 1.0})").get<TrackModelConfig>();
    EXPECT_DOUBLE_EQ(partial.fill_spacing, 1.0);
    EXPECT_DOUBLE_EQ(partial.max_lateral, TrackModelConfig{}.max_lateral);
    EXPECT_THROW(json::parse(R"({"gap": 1.0})").get<TrackModelConfig>(), std::invalid_argument);
}

}  // namespace
}  // namespace apex::planning
