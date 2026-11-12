#include "apex/stack/lap_counter.hpp"

#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "apex/core/geometry.hpp"
#include "apex/core/track.hpp"

namespace apex::stack {
namespace {

// A perfect map of every cone of the track.
ConeMap perfect_map(const Track& track) {
    ConeMap map;
    for (const Cone& cone : track.cones) {
        MapCone landmark;
        landmark.id = cone.id;
        landmark.position = cone.position;
        landmark.color = cone.color;
        landmark.truth_id = cone.id;
        map.cones.push_back(landmark);
    }
    map.revision = 1;
    return map;
}

// Drives the counter along the centerline from the staging position for `distance` meters.
void drive(LapCounter& counter, const Track& track, double distance, double step = 0.4) {
    const Trajectory& center = track.centerline;
    const double s0 = center.project(track.start_pose.position()).s;
    for (double d = 0.0; d <= distance; d += step) {
        (void)counter.update(center.at(s0 + d).position);
    }
}

TEST(TimingLine, FromTheBigOrangeCones) {
    const Track track = make_oval_track();
    const auto line = timing_line_from_map(perfect_map(track), track.start_pose, 15.0);
    ASSERT_TRUE(line.has_value());
    EXPECT_TRUE(line->mapped);
    EXPECT_LT((line->left - track.timing_line_left).norm(), 0.01);
    EXPECT_LT((line->right - track.timing_line_right).norm(), 0.01);
}

TEST(TimingLine, NeedsABigOrangeConeOnEachSideNearTheStart) {
    const Track track = make_oval_track();
    ConeMap map = perfect_map(track);
    for (MapCone& cone : map.cones) {
        const bool left = track.start_pose.to_local(cone.position).y() > 0.0;
        if (cone.color == ConeColor::BigOrange && left) cone.color = ConeColor::Unknown;
    }
    EXPECT_FALSE(timing_line_from_map(map, track.start_pose, 15.0).has_value());
    // Big orange cones beyond the search radius are someone else's.
    EXPECT_FALSE(timing_line_from_map(perfect_map(track), track.start_pose, 3.0).has_value());
    EXPECT_FALSE(timing_line_from_map(ConeMap{}, track.start_pose, 15.0).has_value());
}

TEST(LapCounter, CountsEveryForwardCrossing) {
    const Track track = make_oval_track();
    LapCounter counter(track.start_pose, LapCounterConfig{});
    counter.update_line(perfect_map(track));
    ASSERT_TRUE(counter.line().mapped);
    drive(counter, track, 5.5);
    EXPECT_EQ(counter.crossings(), 0);
    drive(counter, track, 7.0);
    // Restarting the drive from staging adds a jump back; that is not a forward crossing.
    EXPECT_EQ(counter.crossings(), 1);
    EXPECT_EQ(counter.laps_completed(), 0);

    LapCounter laps(track.start_pose, LapCounterConfig{});
    laps.update_line(perfect_map(track));
    drive(laps, track, 6.0 + 3.0 * track.centerline.length() + 1.0);
    EXPECT_EQ(laps.crossings(), 4);
    EXPECT_EQ(laps.laps_completed(), 3);
    EXPECT_NEAR(laps.distance_driven(), 6.0 + 3.0 * track.centerline.length() + 1.0, 1.0);
}

TEST(LapCounter, FallbackLineBeforeAnyBigOrangeConeIsMapped) {
    const Track track = make_oval_track();
    LapCounter counter(track.start_pose, LapCounterConfig{});
    counter.update_line(ConeMap{});
    EXPECT_FALSE(counter.line().mapped);
    drive(counter, track, 5.8);
    EXPECT_EQ(counter.crossings(), 0);
    LapCounter again(track.start_pose, LapCounterConfig{});
    drive(again, track, 7.0);
    EXPECT_EQ(again.crossings(), 1);
}

TEST(LapCounter, IgnoresBackwardAndOutsideCrossings) {
    const Track track = make_oval_track();
    const Vec2 middle = 0.5 * (track.timing_line_left + track.timing_line_right);
    const Vec2 forward = heading_vector(track.centerline.at(0.0).yaw);
    const Vec2 left = left_normal(forward);

    LapCounter backward(track.start_pose, LapCounterConfig{});
    backward.update_line(perfect_map(track));
    (void)backward.update(middle + forward);
    EXPECT_FALSE(backward.update(middle - forward));
    EXPECT_EQ(backward.crossings(), 0);

    // Passing 1.75 + 1.5 + 0.5 m to the left of the middle: beyond the line and its margin.
    LapCounter outside(track.start_pose, LapCounterConfig{});
    outside.update_line(perfect_map(track));
    (void)outside.update(middle - forward + 3.75 * left);
    EXPECT_FALSE(outside.update(middle + forward + 3.75 * left));
    // Within the margin it counts.
    (void)outside.update(middle - forward + 2.75 * left);
    EXPECT_TRUE(outside.update(middle + forward + 2.75 * left));
}

TEST(LapCounter, NeedsAMinimumDistanceBetweenCrossings) {
    const Track track = make_oval_track();
    const Vec2 middle = 0.5 * (track.timing_line_left + track.timing_line_right);
    const Vec2 forward = heading_vector(track.centerline.at(0.0).yaw);
    LapCounter counter(track.start_pose, LapCounterConfig{});
    counter.update_line(perfect_map(track));
    (void)counter.update(middle - forward);
    EXPECT_TRUE(counter.update(middle + forward));
    // Wobbling back over the line and forward again within 30 m is not a lap.
    (void)counter.update(middle - forward);
    EXPECT_FALSE(counter.update(middle + forward));
    EXPECT_EQ(counter.crossings(), 1);
}

TEST(LapCounter, RejectsInvalidConfigs) {
    LapCounterConfig config;
    config.min_lap_distance = 0.0;
    EXPECT_THROW(LapCounter(Pose2{}, config), std::invalid_argument);
}

}  // namespace
}  // namespace apex::stack
