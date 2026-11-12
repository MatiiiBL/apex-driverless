#include "apex/core/types.hpp"

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/vehicle.hpp"

namespace apex {
namespace {

TEST(Angles, WrapIntoHalfOpenInterval) {
    EXPECT_DOUBLE_EQ(wrap_angle(0.0), 0.0);
    EXPECT_DOUBLE_EQ(wrap_angle(kPi), kPi);
    EXPECT_DOUBLE_EQ(wrap_angle(-kPi), kPi);
    EXPECT_NEAR(wrap_angle(3.0 * kPi), kPi, 1e-12);
    EXPECT_NEAR(wrap_angle(kTwoPi + 0.25), 0.25, 1e-12);
    EXPECT_NEAR(angle_diff(0.1, kTwoPi - 0.1), 0.2, 1e-12);
    EXPECT_NEAR(angle_diff(-3.0, 3.0), kTwoPi - 6.0, 1e-12);
}

TEST(Pose2, TransformsRoundTrip) {
    const Pose2 pose{2.0, -1.0, 0.7};
    const Vec2 local{0.5, 3.0};
    const Vec2 world = pose.to_parent(local);
    EXPECT_TRUE(pose.to_local(world).isApprox(local, 1e-12));
    // A point straight ahead lands along the heading.
    const Vec2 ahead = pose.to_parent({1.0, 0.0});
    EXPECT_NEAR(ahead.x(), 2.0 + std::cos(0.7), 1e-12);
    EXPECT_NEAR(ahead.y(), -1.0 + std::sin(0.7), 1e-12);
}

TEST(Pose2, ComposeInverseBetween) {
    const Pose2 a{1.0, 2.0, 0.3};
    const Pose2 b{-4.0, 0.5, -2.9};
    const Pose2 identity = a.compose(a.inverse());
    EXPECT_NEAR(identity.x, 0.0, 1e-12);
    EXPECT_NEAR(identity.y, 0.0, 1e-12);
    EXPECT_NEAR(identity.yaw, 0.0, 1e-12);
    const Pose2 delta = a.between(b);
    const Pose2 back = a.compose(delta);
    EXPECT_NEAR(back.x, b.x, 1e-12);
    EXPECT_NEAR(back.y, b.y, 1e-12);
    EXPECT_NEAR(angle_diff(back.yaw, b.yaw), 0.0, 1e-12);
}

TEST(ConeColor, StringRoundTrip) {
    for (std::size_t i = 0; i < kConeColorCount; ++i) {
        const auto color = static_cast<ConeColor>(i);
        EXPECT_EQ(cone_color_from_string(to_string(color)), color);
    }
    EXPECT_FALSE(cone_color_from_string("purple").has_value());
    EXPECT_GT(cone_shape(ConeColor::BigOrange).height, cone_shape(ConeColor::Blue).height);
}

TEST(Messages, StampOfEveryAlternative) {
    EXPECT_EQ(stamp_of(ImuSample{1.5}), 1.5);
    EXPECT_EQ(stamp_of(LidarScan{2.5, {}}), 2.5);
    EXPECT_EQ(stamp_of(CameraFrame{3.5, {}}), 3.5);
}

TEST(SensorRig, DefaultLidarChannels) {
    const LidarGeometry lidar;
    ASSERT_EQ(lidar.elevations.size(), 64U);
    EXPECT_NEAR(rad_to_deg(lidar.elevations.front()), -12.0, 1e-9);
    EXPECT_NEAR(rad_to_deg(lidar.elevations.back()), 4.0, 1e-9);
    // Every rate divides the 1 kHz physics rate.
    const SensorRig rig;
    for (const double rate : {rig.lidar.rate_hz, rig.camera.rate_hz, rig.imu_rate_hz,
                              rig.wheel_rate_hz, rig.steering_rate_hz}) {
        EXPECT_DOUBLE_EQ(std::fmod(1000.0, rate), 0.0) << rate;
    }
}

TEST(VehicleParams, AeroForcesScaleWithSpeedSquared) {
    const VehicleParams params;
    EXPECT_NEAR(params.downforce(20.0), 4.0 * params.downforce(10.0), 1e-9);
    EXPECT_NEAR(params.drag(10.0), 0.5 * 1.2 * 1.4 * 100.0, 1e-9);
    EXPECT_DOUBLE_EQ(params.wheelbase(), params.lf + params.lr);
}

}  // namespace
}  // namespace apex
