#include <stdexcept>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "apex/control/controller.hpp"

namespace apex::control {
namespace {

using nlohmann::json;

TEST(ControllerConfig, RoundTripsThroughJson) {
    ControllerConfig config;
    config.kind = ControllerKind::Stanley;
    config.pure_pursuit.lookahead_time = 0.55;
    config.pure_pursuit.speed.kp = 2.5;
    config.stanley.gain = 3.0;
    config.stanley.slip_feedforward = false;
    config.mpc.horizon = 42;
    config.mpc.weight_lateral = 9.0;
    config.mpc.compensate_lag = false;

    const json j = config;
    EXPECT_EQ(j.at("kind"), "stanley");
    const ControllerConfig back = j.get<ControllerConfig>();
    EXPECT_EQ(json(back), j);
    EXPECT_EQ(back.mpc.horizon, 42);
    EXPECT_FALSE(back.stanley.slip_feedforward);
}

TEST(ControllerConfig, MissingKeysKeepTheirDefaults) {
    const ControllerConfig config = json::parse(R"({"kind": "pure_pursuit",
        "pure_pursuit": {"lookahead_min": 4.0}})")
                                        .get<ControllerConfig>();
    EXPECT_EQ(config.kind, ControllerKind::PurePursuit);
    EXPECT_EQ(config.pure_pursuit.lookahead_min, 4.0);
    EXPECT_EQ(config.pure_pursuit.lookahead_max, PurePursuitConfig{}.lookahead_max);
    EXPECT_EQ(config.mpc.horizon, MpcConfig{}.horizon);
}

TEST(ControllerConfig, RejectsUnknownKeysAndKinds) {
    EXPECT_THROW((void)json::parse(R"({"mpc": {"horizn": 20}})").get<ControllerConfig>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json::parse(R"({"stanley": {"speed": {"kd": 1}}})").get<ControllerConfig>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json::parse(R"({"kind": "lqr"})").get<ControllerConfig>(),
                 std::invalid_argument);
}

TEST(ControllerConfig, InvalidValuesThrowAtConstruction) {
    const VehicleParams params;
    ControllerConfig config;
    config.kind = ControllerKind::PurePursuit;
    config.pure_pursuit.lookahead_min = 0.0;
    EXPECT_THROW((void)make_controller(config, params), std::invalid_argument);

    config = {};
    config.kind = ControllerKind::Stanley;
    config.stanley.speed.kp = -1.0;
    EXPECT_THROW((void)make_controller(config, params), std::invalid_argument);

    config = {};
    config.kind = ControllerKind::Stanley;
    VehicleParams broken;
    broken.mass = 0.0;
    EXPECT_THROW((void)make_controller(config, broken), std::invalid_argument);
}

TEST(ControllerConfig, NamesItsKinds) {
    EXPECT_EQ(to_string(ControllerKind::PurePursuit), "pure_pursuit");
    EXPECT_EQ(to_string(ControllerKind::Stanley), "stanley");
    EXPECT_EQ(to_string(ControllerKind::Mpc), "mpc");
    ControllerConfig config;
    config.kind = ControllerKind::Stanley;
    EXPECT_EQ(make_controller(config, {})->name(), "stanley");
    config.kind = ControllerKind::PurePursuit;
    EXPECT_EQ(make_controller(config, {})->name(), "pure_pursuit");
}

}  // namespace
}  // namespace apex::control
