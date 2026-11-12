#include "apex/core/track.hpp"

#include <cmath>
#include <filesystem>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

namespace apex {
namespace {

TEST(Track, OvalIsValidWithColorsOnTheRightSides) {
    const Track track = make_oval_track(40.0, 12.0);
    EXPECT_NO_THROW(validate_track(track));
    const double expected_length = 2.0 * 40.0 + kTwoPi * 12.0;
    EXPECT_NEAR(track.centerline.length(), expected_length, 0.01 * expected_length);
    int big = 0;
    for (const Cone& cone : track.cones) {
        const double lateral = track.centerline.project(cone.position).lateral;
        EXPECT_NEAR(std::abs(lateral), 1.75, 0.02);
        if (cone.color == ConeColor::Blue) {
            EXPECT_GT(lateral, 0.0);
        }
        if (cone.color == ConeColor::Yellow) {
            EXPECT_LT(lateral, 0.0);
        }
        if (cone.color == ConeColor::BigOrange) ++big;
    }
    EXPECT_EQ(big, 4);
}

TEST(Track, StagedBehindTheTimingLine) {
    const Track track = make_oval_track();
    const Trajectory::Projection start = track.centerline.project(track.start_pose.position());
    EXPECT_NEAR(track.centerline.delta_s(start.s, 0.0), 6.0, 0.05);
    EXPECT_NEAR(start.lateral, 0.0, 1e-6);
    const Vec2 line_mid = 0.5 * (track.timing_line_left + track.timing_line_right);
    EXPECT_NEAR(track.centerline.project(line_mid).s, 0.0, 1e-6);
}

TEST(Track, ConesDenserInCorners) {
    const Track track = make_oval_track(40.0, 12.0);
    // Inner (left) edge of a 12 m corner has a 10.25 m radius: spacing sqrt(8 * 0.15 * 10.25).
    double straight_gap = 0.0;
    double corner_gap = 1e9;
    std::vector<Vec2> blue;
    for (const Cone& cone : track.cones) {
        if (cone.color == ConeColor::Blue) blue.push_back(cone.position);
    }
    for (std::size_t i = 0; i + 1 < blue.size(); ++i) {
        const double gap = (blue[i + 1] - blue[i]).norm();
        const double kappa = track.centerline.at(track.centerline.project(blue[i]).s).curvature;
        if (std::abs(kappa) < 1e-3) straight_gap = std::max(straight_gap, gap);
        if (std::abs(kappa) > 0.07) corner_gap = std::min(corner_gap, gap);
    }
    EXPECT_GT(straight_gap, 4.0);
    EXPECT_LE(straight_gap, 5.0 + 1e-6);
    EXPECT_LT(corner_gap, 3.8);
}

TEST(Track, RejectsCornersTighterThanTheWidth) {
    std::vector<Vec2> tight;
    for (int i = 0; i < 16; ++i) tight.push_back(1.5 * heading_vector(i * kTwoPi / 16.0));
    EXPECT_THROW((void)track_from_centerline(tight), std::invalid_argument);
}

TEST(Track, JsonRoundTrip) {
    const Track track = make_oval_track(30.0, 10.0);
    const auto path = std::filesystem::temp_directory_path() / "apex_track_roundtrip.json";
    save_track(track, path);
    const Track loaded = load_track(path);
    std::filesystem::remove(path);
    EXPECT_EQ(loaded.name, track.name);
    ASSERT_EQ(loaded.cones.size(), track.cones.size());
    EXPECT_TRUE(loaded.cones[7].position.isApprox(track.cones[7].position));
    EXPECT_EQ(loaded.cones[7].color, track.cones[7].color);
    EXPECT_NEAR(loaded.centerline.length(), track.centerline.length(), 1e-9);
    EXPECT_NEAR(loaded.start_pose.yaw, track.start_pose.yaw, 1e-15);
    EXPECT_NO_THROW(validate_track(loaded));
}

}  // namespace
}  // namespace apex
