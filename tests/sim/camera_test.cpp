#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"
#include "apex/sim/sensors.hpp"

namespace apex::sim {
namespace {

// A detector that finds every candidate, exactly, in its true color, with no false positives.
CameraNoise perfect() {
    CameraNoise noise = scale_noise(SensorNoise{}, 0.0).camera;
    noise.detection_probability = 1.0;
    noise.detection_half_range = 1e6;
    noise.confusion_near = 0.0;
    noise.confusion_far = 0.0;
    noise.false_positives = 0.0;
    return noise;
}

// The car at the origin heading +x; the camera at the CoG, 1.1 m up.
const VehicleState kOrigin;

std::vector<std::int32_t> ids(const CameraFrame& frame) {
    std::vector<std::int32_t> out;
    for (const CameraDetection& d : frame.detections) out.push_back(d.truth_id);
    return out;
}

TEST(Camera, ReportsVisibleConesWithinFieldOfViewAndRange) {
    const CameraGeometry geometry;
    const std::vector<Cone> cones = {
        {0, {10.0, 3.0}, ConeColor::Blue},    // in view
        {1, {8.0, -4.0}, ConeColor::Yellow},  // in view
        {2, {-5.0, 0.0}, ConeColor::Blue},    // behind
        {3, {2.0, 5.0}, ConeColor::Blue},     // 68 degrees left: outside the 100 degree view
        {4, {35.0, 0.0}, ConeColor::Orange},  // beyond 30 m
        {5, {1.5, 0.0}, ConeColor::Yellow},   // under the nose
        {6, {20.0, -1.0}, ConeColor::BigOrange},
    };
    Rng rng(1);
    const CameraFrame frame = simulate_camera(geometry, perfect(), kOrigin, cones, 2.0, rng);
    EXPECT_EQ(frame.stamp, 2.0);
    // Sorted by bearing: right (negative) to left.
    EXPECT_EQ(ids(frame), (std::vector<std::int32_t>{1, 6, 0}));
    for (const CameraDetection& d : frame.detections) {
        const Cone& cone = cones[static_cast<std::size_t>(d.truth_id)];
        EXPECT_NEAR(d.bearing, std::atan2(cone.position.y(), cone.position.x()), 1e-12);
        EXPECT_NEAR(d.range, cone.position.norm(), 1e-12);
        EXPECT_EQ(d.color, cone.color);
        EXPECT_GT(d.confidence, 0.9);
    }
}

// The camera looks down from 1.1 m: a cone hides another only when nearly in line with it and
// close in front of it.
TEST(Camera, ConesHideConesBehindThem) {
    const CameraGeometry geometry;
    Rng rng(2);
    const std::vector<Cone> in_line = {{0, {20.0, 0.0}, ConeColor::Blue},
                                       {1, {18.0, 0.0}, ConeColor::Blue}};
    EXPECT_EQ(ids(simulate_camera(geometry, perfect(), kOrigin, in_line, 0.0, rng)),
              (std::vector<std::int32_t>{1}));
    const std::vector<Cone> far_apart = {{0, {20.0, 0.0}, ConeColor::Blue},
                                         {1, {10.0, 0.0}, ConeColor::Blue}};
    EXPECT_EQ(simulate_camera(geometry, perfect(), kOrigin, far_apart, 0.0, rng).detections.size(),
              2U);
    const std::vector<Cone> side_by_side = {{0, {20.0, 0.0}, ConeColor::Blue},
                                            {1, {18.0, 0.5}, ConeColor::Blue}};
    EXPECT_EQ(
        simulate_camera(geometry, perfect(), kOrigin, side_by_side, 0.0, rng).detections.size(),
        2U);
}

struct RangeStats {
    double detected = 0.0;  // fraction of frames
    double range_stddev = 0.0;
    double bearing_stddev = 0.0;
    double confused = 0.0;  // fraction of detections
};

RangeStats statistics(const CameraNoise& noise, double range, int frames) {
    const CameraGeometry geometry;
    const std::vector<Cone> cones = {{0, {range, 0.0}, ConeColor::Yellow}};
    Rng rng(static_cast<std::uint64_t>(range * 10.0));
    int detected = 0;
    int confused = 0;
    double range_squares = 0.0;
    double bearing_squares = 0.0;
    for (int i = 0; i < frames; ++i) {
        for (const CameraDetection& d :
             simulate_camera(geometry, noise, kOrigin, cones, 0.0, rng).detections) {
            if (d.truth_id != 0) continue;
            ++detected;
            confused += d.color != ConeColor::Yellow ? 1 : 0;
            range_squares += (d.range - range) * (d.range - range);
            bearing_squares += d.bearing * d.bearing;
        }
    }
    const double n = std::max(1, detected);
    return {detected / static_cast<double>(frames), std::sqrt(range_squares / n),
            std::sqrt(bearing_squares / n), confused / n};
}

// Detection probability falls logistically with range, range noise grows as range^2, color
// confusion grows linearly: checked against the formulas of CameraNoise over 4000 frames.
TEST(Camera, NoiseGrowsWithRangeAsConfigured) {
    CameraNoise noise;
    noise.false_positives = 0.0;
    const double max_range = CameraGeometry{}.max_range;
    for (const double range : {5.0, 15.0, 22.0, 27.0}) {
        SCOPED_TRACE(range);
        const RangeStats s = statistics(noise, range, 4000);
        const double p =
            noise.detection_probability /
            (1.0 + std::exp((range - noise.detection_half_range) / noise.detection_falloff));
        EXPECT_NEAR(s.detected, p, 0.025);
        const double sigma = noise.range_stddev + noise.range_stddev_quadratic * range * range;
        EXPECT_NEAR(s.range_stddev, sigma, 0.08 * sigma);
        EXPECT_NEAR(s.bearing_stddev, noise.bearing_stddev, 0.08 * noise.bearing_stddev);
        const double confusion =
            noise.confusion_near + (noise.confusion_far - noise.confusion_near) * range / max_range;
        EXPECT_NEAR(s.confused, confusion, 0.025);
    }
}

TEST(Camera, FalsePositivesArePoissonAndUntagged) {
    const CameraGeometry geometry;
    CameraNoise noise;
    noise.false_positives = 0.5;
    Rng rng(9);
    const std::vector<Cone> none;
    int total = 0;
    for (int i = 0; i < 4000; ++i) {
        const CameraFrame frame = simulate_camera(geometry, noise, kOrigin, none, 0.0, rng);
        for (const CameraDetection& d : frame.detections) {
            ++total;
            EXPECT_EQ(d.truth_id, -1);
            EXPECT_LE(std::abs(d.bearing), 0.5 * geometry.horizontal_fov);
            EXPECT_GE(d.range, noise.min_range);
            EXPECT_LE(d.range, geometry.max_range);
            EXPECT_LT(d.confidence, 0.6);
        }
    }
    EXPECT_NEAR(total / 4000.0, 0.5, 0.04);
}

// Measurements follow the car: the same cone seen from a moved and turned car has the bearing
// and range of its position relative to the camera mount.
TEST(Camera, MeasuresRelativeToTheCameraMount) {
    CameraGeometry geometry;
    geometry.mount = Vec3(0.5, 0.0, 1.1);
    VehicleState state;
    state.pose = {3.0, 1.0, deg_to_rad(30.0)};
    const std::vector<Cone> cones = {{0, {15.0, 9.0}, ConeColor::Blue}};
    Rng rng(4);
    const CameraFrame frame = simulate_camera(geometry, perfect(), state, cones, 0.0, rng);
    ASSERT_EQ(frame.detections.size(), 1U);
    const Vec2 relative = state.pose.to_local(cones[0].position) - Vec2(0.5, 0.0);
    EXPECT_NEAR(frame.detections[0].range, relative.norm(), 1e-12);
    EXPECT_NEAR(frame.detections[0].bearing, std::atan2(relative.y(), relative.x()), 1e-12);
}

}  // namespace
}  // namespace apex::sim
