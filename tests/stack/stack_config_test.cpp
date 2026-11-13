#include "apex/stack/stack_config.hpp"

#include <stdexcept>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace apex::stack {
namespace {

using nlohmann::json;

TEST(StackConfig, JsonRoundTripKeepsEveryModule) {
    StackConfig config;
    config.perception.ground.threshold = 0.05;
    config.mission.laps = 4;
    config.lap_counter.min_lap_distance = 40.0;
    config.known_pose_mapper.gate = 0.6;
    config.stop_decel = 4.0;
    const StackConfig back = json(config).get<StackConfig>();
    EXPECT_DOUBLE_EQ(back.perception.ground.threshold, 0.05);
    EXPECT_EQ(back.mission.laps, 4);
    EXPECT_DOUBLE_EQ(back.lap_counter.min_lap_distance, 40.0);
    EXPECT_DOUBLE_EQ(back.known_pose_mapper.gate, 0.6);
    EXPECT_DOUBLE_EQ(back.stop_decel, 4.0);
}

TEST(StackConfig, MissingKeysKeepDefaultsAndUnknownKeysThrow) {
    const StackConfig config =
        json{{"perception", {{"ground", {{"threshold", 0.05}}}}}}.get<StackConfig>();
    EXPECT_DOUBLE_EQ(config.perception.ground.threshold, 0.05);
    EXPECT_DOUBLE_EQ(config.perception.ground.seed_cell_size,
                     perception::GroundConfig{}.seed_cell_size);
    EXPECT_EQ(config.mission.laps, MissionConfig{}.laps);
    EXPECT_THROW((void)json({{"perception", {{"radar", 1}}}}).get<StackConfig>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json({{"planner", 1}}).get<StackConfig>(), std::invalid_argument);
}

}  // namespace
}  // namespace apex::stack
