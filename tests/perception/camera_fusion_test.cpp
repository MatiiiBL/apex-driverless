#include "apex/perception/camera_fusion.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"

#include "scene.hpp"

namespace apex::perception {
namespace {

CameraFrame frame_at(Time stamp, std::size_t detections = 0) {
    CameraFrame frame;
    frame.stamp = stamp;
    frame.detections.resize(detections);
    return frame;
}

CameraDetection detection_of(const Vec2& body, ConeColor color, double confidence = 0.8,
                             std::int32_t id = -1) {
    return {std::atan2(body.y(), body.x()), body.norm(), color, confidence, id};
}

std::vector<Time> stamps(const CameraBuffer& buffer) {
    std::vector<Time> out;
    for (const CameraFrame& f : buffer.frames()) out.push_back(f.stamp);
    return out;
}

TEST(CameraBuffer, KeepsTheNewestFramesSortedWhateverTheArrivalOrder) {
    CameraBuffer buffer(3);
    EXPECT_EQ(buffer.closest(1.0), nullptr);
    EXPECT_EQ(buffer.push(frame_at(0.10)), CameraBuffer::Insert::Added);
    EXPECT_EQ(buffer.push(frame_at(0.00)), CameraBuffer::Insert::Added);
    EXPECT_EQ(buffer.push(frame_at(0.20)), CameraBuffer::Insert::Added);
    EXPECT_EQ(stamps(buffer), (std::vector<Time>{0.00, 0.10, 0.20}));
    // Full: a frame older than all of them is dropped, a newer one evicts the oldest.
    EXPECT_EQ(buffer.push(frame_at(-0.05)), CameraBuffer::Insert::Dropped);
    EXPECT_EQ(buffer.push(frame_at(0.15)), CameraBuffer::Insert::Added);
    EXPECT_EQ(stamps(buffer), (std::vector<Time>{0.10, 0.15, 0.20}));
    // Same stamp: replaced, not duplicated.
    EXPECT_EQ(buffer.push(frame_at(0.15, 4)), CameraBuffer::Insert::Replaced);
    EXPECT_EQ(buffer.size(), 3U);
    EXPECT_EQ(buffer.closest(0.15)->detections.size(), 4U);
    EXPECT_EQ(buffer.push(frame_at(std::numeric_limits<double>::quiet_NaN())),
              CameraBuffer::Insert::Dropped);
    EXPECT_THROW(CameraBuffer(0), std::invalid_argument);
}

TEST(CameraBuffer, ClosestPrefersTheEarlierOnATie) {
    CameraBuffer buffer(8);
    for (const Time t : {0.30, 0.10, 0.20}) (void)buffer.push(frame_at(t));
    EXPECT_DOUBLE_EQ(buffer.closest(0.0)->stamp, 0.10);
    EXPECT_DOUBLE_EQ(buffer.closest(0.16)->stamp, 0.20);
    EXPECT_DOUBLE_EQ(buffer.closest(0.25)->stamp, 0.20);
    EXPECT_DOUBLE_EQ(buffer.closest(9.0)->stamp, 0.30);
    EXPECT_DOUBLE_EQ(buffer.closest(0.15)->stamp, 0.10);
}

TEST(CameraFusion, ColorsEachConeFromItsDetection) {
    CameraFusion fusion(CameraFusionConfig{}, CameraGeometry{});
    const std::vector<Vec2> cones = {{4.0, 1.75}, {4.0, -1.75}, {9.0, 1.8}, {9.0, -1.7}};
    CameraFrame frame = frame_at(1.0);
    // Listed in another order than the cones, with a false positive far off.
    frame.detections = {
        detection_of(cones[3], ConeColor::Yellow), detection_of(cones[0], ConeColor::Blue, 0.9),
        detection_of({20.0, -6.0}, ConeColor::Orange), detection_of(cones[2], ConeColor::Blue),
        detection_of(cones[1], ConeColor::Yellow)};
    (void)fusion.add_frame(frame);
    std::vector<ColorMatch> colors;
    ASSERT_TRUE(fusion.colorize(cones, 1.0, 0.0, colors));
    ASSERT_EQ(colors.size(), cones.size());
    EXPECT_EQ(colors[0].color, ConeColor::Blue);
    EXPECT_EQ(colors[1].color, ConeColor::Yellow);
    EXPECT_EQ(colors[2].color, ConeColor::Blue);
    EXPECT_EQ(colors[3].color, ConeColor::Yellow);
    EXPECT_EQ(colors[0].detection, 1);
    EXPECT_NEAR(colors[0].confidence, 0.9, 1e-9);  // perfect match: the detector's confidence
    for (const ColorMatch& c : colors) {
        EXPECT_GE(c.confidence, 0.0);
        EXPECT_LE(c.confidence, 1.0);
    }
}

TEST(CameraFusion, CompensatesTheRotationBetweenStamps) {
    const CameraGeometry geometry;
    std::vector<Cone> cones;
    std::vector<Vec2> positions;
    for (int i = 0; i < 5; ++i) {
        const double bearing = deg_to_rad(-40.0 + 20.0 * i);
        const double range = 6.0 + 3.0 * i;
        positions.emplace_back(range * std::cos(bearing), range * std::sin(bearing));
        cones.push_back({i, positions.back(), i % 2 == 0 ? ConeColor::Blue : ConeColor::Yellow});
    }
    // The camera frame is 50 ms after the scan; the car turns at 2 rad/s meanwhile.
    const double yaw_rate = 2.0;
    const Time scan = 3.0;
    const Time camera = scan + 0.05;
    Rng rng(1);
    const CameraFrame frame = testing::detect_cones(
        geometry, Pose2{0.0, 0.0, yaw_rate * (camera - scan)}, cones, {}, rng, camera);
    ASSERT_EQ(frame.detections.size(), cones.size());

    CameraFusion fusion(CameraFusionConfig{}, geometry);
    (void)fusion.add_frame(frame);
    std::vector<ColorMatch> colors;
    ASSERT_TRUE(fusion.colorize(positions, scan, yaw_rate, colors));
    for (std::size_t i = 0; i < cones.size(); ++i) {
        ASSERT_GE(colors[i].detection, 0) << "cone " << i;
        EXPECT_EQ(frame.detections[static_cast<std::size_t>(colors[i].detection)].truth_id,
                  cones[i].id);
        EXPECT_EQ(colors[i].color, cones[i].color);
    }
    // Ignoring the rotation, the 0.1 rad bearing shift is outside every gate.
    ASSERT_TRUE(fusion.colorize(positions, scan, 0.0, colors));
    for (const ColorMatch& c : colors) EXPECT_EQ(c.color, ConeColor::Unknown);
}

TEST(CameraFusion, SeparatesNearAndFarConesAtTheSameBearing) {
    CameraFusion fusion(CameraFusionConfig{}, CameraGeometry{});
    const std::vector<Vec2> cones = {{5.0, 0.0}, {15.0, 0.25}};  // bearings 0 and 0.95 degrees
    CameraFrame frame = frame_at(0.0);
    frame.detections = {detection_of(cones[1], ConeColor::Yellow),
                        detection_of(cones[0], ConeColor::Blue)};
    (void)fusion.add_frame(frame);
    std::vector<ColorMatch> colors;
    ASSERT_TRUE(fusion.colorize(cones, 0.0, 0.0, colors));
    EXPECT_EQ(colors[0].color, ConeColor::Blue);
    EXPECT_EQ(colors[1].color, ConeColor::Yellow);

    // The camera saw only the far one: the near cone must not take its color.
    frame.detections = {detection_of(cones[1], ConeColor::Yellow)};
    (void)fusion.add_frame(frame);
    ASSERT_TRUE(fusion.colorize(cones, 0.0, 0.0, colors));
    EXPECT_EQ(colors[0].color, ConeColor::Unknown);
    EXPECT_EQ(colors[0].confidence, 0.0);
    EXPECT_EQ(colors[1].color, ConeColor::Yellow);
}

TEST(CameraFusion, LeavesConesUnknownWithoutAUsableFrame) {
    const CameraFusionConfig config;
    CameraFusion fusion(config, CameraGeometry{});
    const std::vector<Vec2> cones = {{5.0, 1.0}};
    std::vector<ColorMatch> colors;
    EXPECT_FALSE(fusion.colorize(cones, 1.0, 0.0, colors));  // nothing buffered
    ASSERT_EQ(colors.size(), 1U);
    EXPECT_EQ(colors[0].color, ConeColor::Unknown);

    CameraFrame frame = frame_at(1.0);
    frame.detections = {detection_of(cones[0], ConeColor::Blue)};
    (void)fusion.add_frame(frame);
    EXPECT_FALSE(fusion.colorize(cones, 1.0 + 1.5 * config.max_time_gap, 0.0, colors));
    EXPECT_EQ(colors[0].color, ConeColor::Unknown);
    EXPECT_FALSE(fusion.colorize(cones, 1.0, std::numeric_limits<double>::quiet_NaN(), colors));
    EXPECT_TRUE(fusion.colorize(cones, 1.0 + 0.5 * config.max_time_gap, 0.0, colors));
    EXPECT_EQ(colors[0].color, ConeColor::Blue);
}

TEST(CameraFusion, IgnoresDetectionsItCannotTrust) {
    CameraFusion fusion(CameraFusionConfig{}, CameraGeometry{});
    // Outside the 100 degree field of view: never matched, even to a detection right on it.
    const std::vector<Vec2> cones = {{1.0, 5.0}, {6.0, 0.5}, {8.0, -2.0}};
    CameraFrame frame = frame_at(0.0);
    frame.detections = {detection_of(cones[0], ConeColor::Blue),
                        detection_of(cones[1], ConeColor::Unknown),
                        {std::numeric_limits<double>::quiet_NaN(), 8.2, ConeColor::Blue, 0.9, -1}};
    (void)fusion.add_frame(frame);
    std::vector<ColorMatch> colors;
    ASSERT_TRUE(fusion.colorize(cones, 0.0, 0.0, colors));
    EXPECT_EQ(colors[0].detection, -1);
    EXPECT_EQ(colors[0].color, ConeColor::Unknown);
    EXPECT_EQ(colors[1].detection, 1);  // matched, but the detector did not know the color
    EXPECT_EQ(colors[1].color, ConeColor::Unknown);
    EXPECT_EQ(colors[1].confidence, 0.0);
    EXPECT_EQ(colors[2].detection, -1);
}

TEST(CameraFusion, ConfidenceFallsWithMatchQuality) {
    CameraFusion fusion(CameraFusionConfig{}, CameraGeometry{});
    const std::vector<Vec2> cones = {{10.0, 0.0}};
    CameraFrame frame = frame_at(0.0);
    frame.detections = {{0.02, 10.0, ConeColor::Yellow, 1.0, -1}};  // 2 sigma off in bearing
    (void)fusion.add_frame(frame);
    std::vector<ColorMatch> colors;
    ASSERT_TRUE(fusion.colorize(cones, 0.0, 0.0, colors));
    EXPECT_EQ(colors[0].color, ConeColor::Yellow);
    EXPECT_NEAR(colors[0].confidence, std::exp(-0.5 * 4.0), 1e-6);
}

TEST(CameraFusion, RejectsBadConfig) {
    CameraFusionConfig config;
    config.bearing_sigma = 0.0;
    EXPECT_THROW(CameraFusion(config, CameraGeometry{}), std::invalid_argument);
    config = CameraFusionConfig{};
    config.buffer_frames = 0;
    EXPECT_THROW(CameraFusion(config, CameraGeometry{}), std::invalid_argument);
}

}  // namespace
}  // namespace apex::perception
