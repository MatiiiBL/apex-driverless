#include "apex/runner/config.hpp"

#include <stdexcept>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace apex::runner {
namespace {

using nlohmann::json;

TEST(RunConfig, DefaultsMatchTheDesign) {
    const RunConfig config = json::object().get<RunConfig>();
    EXPECT_EQ(config.track.source, TrackSpec::Source::Oval);
    EXPECT_DOUBLE_EQ(config.latency.camera, 0.040);
    EXPECT_DOUBLE_EQ(config.latency.lidar, 0.050);
    EXPECT_DOUBLE_EQ(config.control_period, 0.020);
    EXPECT_EQ(config.stack.mission.laps, 10);
    EXPECT_DOUBLE_EQ(config.dnf.stall_time, 10.0);
    EXPECT_DOUBLE_EQ(config.dnf.max_off_track, 5.0);
    EXPECT_FALSE(config.oracles.perception);
    EXPECT_FALSE(config.oracles.state);
    EXPECT_NO_THROW(validate(config));
}

TEST(RunConfig, JsonRoundTrip) {
    RunConfig config;
    config.name = "test";
    config.track.source = TrackSpec::Source::Generated;
    config.track.seed = 17;
    config.vehicle.mass = 250.0;
    config.nominal.mass = 245.0;
    config.friction_scale = 0.9;
    config.latency.lidar = 0.08;
    config.seed = 99;
    config.oracles.state = true;
    config.stack.mission.laps = 3;
    config.dnf.time_limit = 120.0;
    const RunConfig back = json(config).get<RunConfig>();
    EXPECT_EQ(back.name, "test");
    EXPECT_EQ(back.track.source, TrackSpec::Source::Generated);
    EXPECT_EQ(back.track.seed, 17U);
    EXPECT_DOUBLE_EQ(back.vehicle.mass, 250.0);
    EXPECT_DOUBLE_EQ(back.nominal.mass, 245.0);
    EXPECT_DOUBLE_EQ(back.friction_scale, 0.9);
    EXPECT_DOUBLE_EQ(back.latency.lidar, 0.08);
    EXPECT_EQ(back.seed, 99U);
    EXPECT_TRUE(back.oracles.state);
    EXPECT_EQ(back.stack.mission.laps, 3);
    EXPECT_DOUBLE_EQ(back.dnf.time_limit, 120.0);
}

TEST(RunConfig, NominalStartsFromTheTrueCar) {
    const RunConfig config =
        json{{"vehicle", {{"mass", 260.0}}}, {"nominal", {{"drag_area", 1.6}}}}.get<RunConfig>();
    EXPECT_DOUBLE_EQ(config.vehicle.mass, 260.0);
    EXPECT_DOUBLE_EQ(config.nominal.mass, 260.0);
    EXPECT_DOUBLE_EQ(config.nominal.drag_area, 1.6);
    EXPECT_DOUBLE_EQ(config.vehicle.drag_area, VehicleParams{}.drag_area);
}

TEST(RunConfig, NoiseModelsFromJson) {
    const RunConfig config =
        json{{"noise", {{"imu", {{"gyro_stddev", 0.01}}}}}, {"sensor_noise_scale", 0.5}}
            .get<RunConfig>();
    EXPECT_DOUBLE_EQ(config.noise.imu.gyro_stddev, 0.01);
    EXPECT_DOUBLE_EQ(config.noise.imu.accel_stddev, sim::ImuNoise{}.accel_stddev);
    EXPECT_DOUBLE_EQ(config.sensor_noise_scale, 0.5);
    EXPECT_NO_THROW(validate(config));

    RunConfig negative;
    negative.noise.wheels.stddev = -0.1;
    EXPECT_THROW(validate(negative), std::invalid_argument);
    EXPECT_THROW((void)json({{"noise", {{"radar", {}}}}}).get<RunConfig>(), std::invalid_argument);
}

TEST(RunConfig, RejectsUnknownKeysAtEveryLevel) {
    EXPECT_THROW((void)json({{"lap", 3}}).get<RunConfig>(), std::invalid_argument);
    EXPECT_THROW((void)json({{"latency", {{"radar", 0.1}}}}).get<RunConfig>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json({{"oracles", {{"map", true}}}}).get<RunConfig>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json({{"stack", {{"mission", {{"lap", 2}}}}}}).get<RunConfig>(),
                 std::invalid_argument);
}

TEST(TrackSpec, EachForm) {
    EXPECT_EQ(json({{"seed", 3}}).get<TrackSpec>().seed, 3U);
    EXPECT_EQ(json({{"layout", "hairpins"}}).get<TrackSpec>().layout, "hairpins");
    EXPECT_EQ(json({{"file", "a.json"}}).get<TrackSpec>().source, TrackSpec::Source::File);
    const TrackSpec oval = json({{"oval", {{"radius", 15.0}}}}).get<TrackSpec>();
    EXPECT_EQ(oval.source, TrackSpec::Source::Oval);
    EXPECT_DOUBLE_EQ(oval.oval_radius, 15.0);
    EXPECT_DOUBLE_EQ(oval.oval_straight, 40.0);
    for (const TrackSpec& spec : {json({{"seed", 3}}).get<TrackSpec>(), oval}) {
        EXPECT_EQ(json(spec), json(json(spec).get<TrackSpec>()));
    }
}

TEST(TrackSpec, ExactlyOneSource) {
    EXPECT_THROW((void)json({{"seed", 3}, {"layout", "x"}}).get<TrackSpec>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json::object().get<TrackSpec>(), std::invalid_argument);
    EXPECT_THROW((void)json({{"generator", 3}}).get<TrackSpec>(), std::invalid_argument);
}

TEST(RunConfig, ValidateNamesTheProblem) {
    RunConfig config;
    config.latency.lidar = 0.0505;
    try {
        validate(config);
        FAIL() << "expected a throw";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("latency.lidar"), std::string::npos);
    }
    config = RunConfig{};
    config.control_period = 0.0;
    EXPECT_THROW(validate(config), std::invalid_argument);
    config = RunConfig{};
    config.friction_scale = 0.0;
    EXPECT_THROW(validate(config), std::invalid_argument);
    config = RunConfig{};
    config.dnf.time_limit = -1.0;
    EXPECT_THROW(validate(config), std::invalid_argument);
}

TEST(ToTicks, WholeMilliseconds) {
    EXPECT_EQ(to_ticks(0.0, "x"), 0);
    EXPECT_EQ(to_ticks(0.05, "x"), 50);
    EXPECT_EQ(to_ticks(0.02, "x"), 20);
    EXPECT_EQ(to_ticks(1.234, "x"), 1234);
    EXPECT_THROW((void)to_ticks(-0.001, "x"), std::invalid_argument);
    EXPECT_THROW((void)to_ticks(0.0015, "x"), std::invalid_argument);
}

}  // namespace
}  // namespace apex::runner
