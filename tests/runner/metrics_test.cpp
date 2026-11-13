#include "apex/runner/metrics.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "apex/core/track.hpp"

namespace apex::runner {
namespace {

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
    return map;
}

TEST(ErrorAccumulator, RmsAndMax) {
    ErrorAccumulator acc;
    EXPECT_EQ(acc.stats().samples, 0U);
    EXPECT_EQ(acc.stats().rms, 0.0);
    acc.add(3.0);
    acc.add(-4.0);
    const ErrorStats s = acc.stats();
    EXPECT_EQ(s.samples, 2U);
    EXPECT_DOUBLE_EQ(s.rms, std::sqrt(12.5));
    EXPECT_DOUBLE_EQ(s.max, 4.0);
}

TEST(EvaluateMap, PerfectMap) {
    const Track track = make_oval_track();
    const MapError e = evaluate_map(perfect_map(track), track, 30.0, true);
    EXPECT_EQ(e.cones, track.cones.size());
    EXPECT_EQ(e.landmarks, track.cones.size());
    EXPECT_EQ(e.matched, track.cones.size());
    EXPECT_EQ(e.false_landmarks, 0U);
    EXPECT_EQ(e.duplicates, 0U);
    EXPECT_DOUBLE_EQ(e.coverage, 1.0);
    EXPECT_DOUBLE_EQ(e.color_accuracy, 1.0);
    EXPECT_DOUBLE_EQ(e.position.rms, 0.0);
    EXPECT_TRUE(e.finalized);
    EXPECT_DOUBLE_EQ(e.time, 30.0);
}

TEST(EvaluateMap, CountsEveryKindOfMistake) {
    const Track track = make_oval_track();
    ConeMap map = perfect_map(track);
    for (MapCone& landmark : map.cones) landmark.position.x() += 0.1;  // a shifted map
    map.cones[0].color = ConeColor::Unknown;
    map.cones.pop_back();  // a cone never mapped
    MapCone duplicate = map.cones[3];
    duplicate.position.y() += 0.3;
    map.cones.push_back(duplicate);
    MapCone clutter;
    clutter.truth_id = -1;
    map.cones.push_back(clutter);
    MapCone out_of_range;
    out_of_range.truth_id = 100000;
    map.cones.push_back(out_of_range);

    const MapError e = evaluate_map(map, track, 0.0, false);
    const std::size_t n = track.cones.size();
    EXPECT_EQ(e.landmarks, n + 2);
    EXPECT_EQ(e.matched, n);
    EXPECT_EQ(e.false_landmarks, 2U);
    EXPECT_EQ(e.duplicates, 1U);
    EXPECT_DOUBLE_EQ(e.coverage, static_cast<double>(n - 1) / static_cast<double>(n));
    EXPECT_DOUBLE_EQ(e.color_accuracy, static_cast<double>(n - 1) / static_cast<double>(n));
    EXPECT_NEAR(e.position.max, std::hypot(0.1, 0.3), 1e-12);
    const double expected_rms =
        std::sqrt((static_cast<double>(n - 1) * 0.01 + (0.01 + 0.09)) / static_cast<double>(n));
    EXPECT_NEAR(e.position.rms, expected_rms, 1e-12);
}

TEST(OffTrackDistance, ZeroOnTheTrackThenGrows) {
    const Track track = make_oval_track();
    EXPECT_EQ(off_track_distance(track, 10.0, 0.0), 0.0);
    EXPECT_EQ(off_track_distance(track, 10.0, 1.7), 0.0);
    EXPECT_NEAR(off_track_distance(track, 10.0, 3.0), 1.25, 1e-9);
    EXPECT_NEAR(off_track_distance(track, 10.0, -7.75), 6.0, 1e-9);
}

TEST(DnfMonitor, Stalled) {
    DnfRules rules;
    rules.stall_time = 10.0;
    DnfMonitor monitor(rules);
    EXPECT_FALSE(monitor.update(0.0, 0.0, 0.0).has_value());
    EXPECT_FALSE(monitor.update(5.0, 0.2, 0.0).has_value());
    // Moving again resets the timer.
    EXPECT_FALSE(monitor.update(6.0, 3.0, 0.0).has_value());
    EXPECT_FALSE(monitor.update(7.0, 0.0, 0.0).has_value());
    EXPECT_FALSE(monitor.update(16.9, 0.0, 0.0).has_value());
    const auto reason = monitor.update(17.0, 0.0, 0.0);
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("stalled"), std::string::npos);
}

TEST(DnfMonitor, OffTrackAndTimeLimit) {
    DnfRules rules;
    rules.time_limit = 100.0;
    DnfMonitor off(rules);
    EXPECT_FALSE(off.update(1.0, 10.0, 4.9).has_value());
    const auto reason = off.update(1.001, 10.0, 5.1);
    ASSERT_TRUE(reason.has_value());
    EXPECT_NE(reason->find("off the track"), std::string::npos);

    DnfMonitor late(rules);
    EXPECT_FALSE(late.update(100.0, 10.0, 0.0).has_value());
    const auto timeout = late.update(100.001, 10.0, 0.0);
    ASSERT_TRUE(timeout.has_value());
    EXPECT_NE(timeout->find("time limit"), std::string::npos);
}

}  // namespace
}  // namespace apex::runner
