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

}  // namespace
}  // namespace apex::planning
