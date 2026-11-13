#include "apex/sim/config_json.hpp"

#include <stdexcept>

#include <gtest/gtest.h>

namespace apex::sim {
namespace {

using nlohmann::json;

TEST(ConfigJson, SensorNoiseRoundTripsEveryField) {
    // Every field off its default, so a field missing from the JSON form would show.
    const SensorNoise noisy = scale_noise(SensorNoise{}, 1.7);
    SensorNoise changed = noisy;
    changed.lidar.dropout = 0.05;
    changed.camera.min_range = 2.5;
    changed.camera.detection_probability = 0.9;
    changed.camera.detection_half_range = 18.0;
    changed.camera.detection_falloff = 2.0;
    changed.camera.confusion_near = 0.02;
    changed.camera.confusion_far = 0.2;
    changed.camera.false_positives = 1.0;
    changed.wheels.slip_stiffness = 15.0;
    const SensorNoise back = json(changed).get<SensorNoise>();
    EXPECT_EQ(json(back), json(changed));
    EXPECT_NE(json(back), json(SensorNoise{}));
    EXPECT_EQ(back.camera.false_positives, 1.0);
    EXPECT_EQ(back.imu.gyro_bias_walk, changed.imu.gyro_bias_walk);
}

TEST(ConfigJson, MissingKeysKeepDefaults) {
    const json partial = {{"imu", {{"gyro_bias", 0.01}}}, {"lidar", json::object()}};
    const SensorNoise noise = partial.get<SensorNoise>();
    EXPECT_EQ(noise.imu.gyro_bias, 0.01);
    EXPECT_EQ(noise.imu.accel_stddev, ImuNoise{}.accel_stddev);
    EXPECT_EQ(noise.lidar.range_stddev, LidarNoise{}.range_stddev);
    EXPECT_EQ(noise.camera.false_positives, CameraNoise{}.false_positives);
}

TEST(ConfigJson, UnknownKeysAreRejected) {
    EXPECT_THROW((void)json({{"lidr", json::object()}}).get<SensorNoise>(), std::invalid_argument);
    EXPECT_THROW((void)json({{"camera", {{"fov", 1.0}}}}).get<SensorNoise>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json({{"rules", {{"min_lenght", 100.0}}}}).get<TrackGenOptions>(),
                 std::invalid_argument);
    EXPECT_THROW((void)json(3.0).get<LidarNoise>(), std::invalid_argument);
}

TEST(ConfigJson, TrackGenOptionsRoundTrip) {
    TrackGenOptions options;
    options.rules.max_length = 420.0;
    options.rules.min_clearance = 5.0;
    options.hairpin_weight = 0.5;
    options.max_attempts = 77;
    const TrackGenOptions back = json(options).get<TrackGenOptions>();
    EXPECT_EQ(back.rules.max_length, 420.0);
    EXPECT_EQ(back.rules.min_clearance, 5.0);
    EXPECT_EQ(back.rules.min_radius, TrackRules{}.min_radius);
    EXPECT_EQ(back.hairpin_weight, 0.5);
    EXPECT_EQ(back.corner_weight, TrackGenOptions{}.corner_weight);
    EXPECT_EQ(back.max_attempts, 77);
    EXPECT_EQ(json(back), json(options));
}

}  // namespace
}  // namespace apex::sim
