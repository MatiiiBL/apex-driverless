#include "apex/core/json.hpp"

#include <gtest/gtest.h>

namespace apex {
namespace {

TEST(Json, VehicleParamsRoundTripAndPartialOverride) {
    VehicleParams params;
    params.mass = 300.0;
    params.front_tire.mu = 1.2;
    const nlohmann::json j = params;
    const VehicleParams back = j.get<VehicleParams>();
    EXPECT_DOUBLE_EQ(back.mass, 300.0);
    EXPECT_DOUBLE_EQ(back.front_tire.mu, 1.2);

    // Missing keys keep their defaults.
    const auto partial = nlohmann::json::parse(R"({"max_power": 50000})").get<VehicleParams>();
    EXPECT_DOUBLE_EQ(partial.max_power, 50000.0);
    EXPECT_DOUBLE_EQ(partial.mass, VehicleParams{}.mass);
}

TEST(Json, UnknownKeysAreRejected) {
    EXPECT_THROW(nlohmann::json::parse(R"({"mas": 200})").get<VehicleParams>(),
                 std::invalid_argument);
    EXPECT_THROW(nlohmann::json::parse(R"({"x": 1, "z": 2})").get<Pose2>(), std::invalid_argument);
}

TEST(Json, ConeColorsAsNames) {
    EXPECT_EQ(nlohmann::json(ConeColor::BigOrange).get<std::string>(), "big_orange");
    EXPECT_EQ(nlohmann::json("yellow").get<ConeColor>(), ConeColor::Yellow);
    EXPECT_THROW(nlohmann::json("teal").get<ConeColor>(), std::invalid_argument);
}

TEST(Json, ConeMapSerializes) {
    ConeMap map;
    map.revision = 3;
    MapCone cone;
    cone.id = 5;
    cone.position = {1.0, 2.0};
    cone.color = ConeColor::Blue;
    map.cones.push_back(cone);
    const nlohmann::json j = map;
    EXPECT_EQ(j["revision"], 3);
    EXPECT_EQ(j["cones"][0]["color"], "blue");
    EXPECT_DOUBLE_EQ(j["cones"][0]["position"][1].get<double>(), 2.0);
}

}  // namespace
}  // namespace apex
