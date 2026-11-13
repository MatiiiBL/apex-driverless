#include "apex/sim/track_gen.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/timing.hpp"
#include "apex/sim/track_rules.hpp"

namespace apex::sim {
namespace {

// The default rules with the length limit lowered to admit the 155 m test oval.
TrackRules oval_rules() {
    TrackRules rules;
    rules.min_length = 100.0;
    return rules;
}

TEST(TrackRules, MeasuresTheOval) {
    const Track track = make_oval_track(40.0, 12.0);
    const TrackStats stats = analyze_track(track);
    EXPECT_NEAR(stats.length, 80.0 + kTwoPi * 12.0, 1.0);
    // The oval joins its straights to the half circles without transitions, and the spline
    // through its waypoints overshoots the curvature step by ~12% (radius 10.5 m) and rings a
    // little into the straights. The generator's clothoids avoid exactly this.
    EXPECT_GT(stats.min_radius, 10.0);
    EXPECT_LT(stats.min_radius, 12.0);
    EXPECT_GT(stats.longest_straight, 35.0);
    EXPECT_LT(stats.longest_straight, 41.0);
    // The closest non-adjacent points are 20 m apart around a half circle: a 17.8 m chord,
    // minus 1.75 m to each cone line.
    const double chord = 2.0 * 12.0 * std::sin(20.0 / (2.0 * 12.0));
    EXPECT_NEAR(stats.min_clearance, chord - 3.5, 0.5);
    EXPECT_EQ(stats.hairpins, 0);
    EXPECT_EQ(stats.corners, 2);  // 12 m is too wide for a hairpin and too tight for a sweeper
    EXPECT_FALSE(check_track_rules(track, oval_rules()).has_value());
    EXPECT_TRUE(check_track_rules(track).has_value());  // too short for the default rules
}

TEST(TrackRules, ClassifiesHairpinsAndSweepers) {
    EXPECT_EQ(analyze_track(make_oval_track(60.0, 8.0)).hairpins, 2);
    EXPECT_EQ(analyze_track(make_oval_track(20.0, 20.0)).sweepers, 2);
}

TEST(TrackRules, ReportsTheBrokenRule) {
    auto broken = [](const Track& track, const TrackRules& rules) {
        return check_track_rules(track, rules).value_or("");
    };
    const Track oval = make_oval_track(40.0, 12.0);
    TrackRules rules = oval_rules();
    rules.max_straight = 30.0;
    EXPECT_NE(broken(oval, rules).find("straight"), std::string::npos);
    rules = oval_rules();
    rules.min_radius = 15.0;
    EXPECT_NE(broken(oval, rules).find("radius"), std::string::npos);
    rules = oval_rules();
    rules.max_length = 150.0;
    EXPECT_NE(broken(oval, rules).find("length"), std::string::npos);
    // Straights 8 m apart leave 4.5 m between their cone lines.
    rules = oval_rules();
    rules.min_radius = 3.0;
    rules.min_clearance = 5.0;
    rules.adjacency_length = 13.0;
    EXPECT_NE(broken(make_oval_track(60.0, 4.0), rules).find("between"), std::string::npos);
}

TEST(TrackGen, ReferenceLayoutsMeetTheRules) {
    for (const std::string& name : reference_track_names()) {
        SCOPED_TRACE(name);
        const Track track = reference_track(name);
        EXPECT_EQ(track.name, name);
        EXPECT_NO_THROW(validate_track(track));
        const std::optional<std::string> broken = check_track_rules(track);
        EXPECT_FALSE(broken.has_value()) << *broken;
        const TrackStats stats = analyze_track(track);
        std::cout << name << ": " << stats.length << " m, radius " << stats.min_radius
                  << " m, straight " << stats.longest_straight << " m, clearance "
                  << stats.min_clearance << " m, " << stats.hairpins << " hairpins, "
                  << stats.chicanes << " chicanes, " << stats.sweepers << " sweepers, "
                  << stats.corners << " corners\n";
    }
    EXPECT_THROW((void)reference_track("no-such-layout"), std::invalid_argument);
}

TEST(TrackGen, SameSeedSameTrack) {
    const Track a = generate_track(42);
    const Track b = generate_track(42);
    ASSERT_EQ(a.cones.size(), b.cones.size());
    for (std::size_t i = 0; i < a.cones.size(); ++i) {
        EXPECT_EQ(a.cones[i].position, b.cones[i].position);
        EXPECT_EQ(a.cones[i].color, b.cones[i].color);
    }
    EXPECT_EQ(a.centerline.length(), b.centerline.length());
    EXPECT_EQ(a.seed, 42U);
    const Track c = generate_track(43);
    EXPECT_NE(a.centerline.length(), c.centerline.length());
}

TEST(TrackGen, RejectsInconsistentOptions) {
    TrackGenOptions options;
    options.rules.max_length = options.rules.min_length;
    EXPECT_THROW((void)generate_track(1, options), std::invalid_argument);
    options = {};
    options.corner_weight = -1.0;
    EXPECT_THROW((void)generate_track(1, options), std::invalid_argument);
    options = {};
    options.max_attempts = 0;
    EXPECT_THROW((void)generate_track(1, options), std::invalid_argument);
}

// Seeds 1-200 in four shards of 50, which ctest runs in parallel. Every track passes
// validate_track and every apex rule, and has at least one hairpin, chicane and sweeper.
class TrackGenSeeds : public testing::TestWithParam<int> {};

TEST_P(TrackGenSeeds, MeetEveryRule) {
    constexpr int kShardSize = 50;
    LatencyStats time;
    int attempts = 0;
    TrackStats total;
    const auto first = static_cast<std::uint64_t>(GetParam() * kShardSize + 1);
    for (std::uint64_t seed = first; seed < first + kShardSize; ++seed) {
        SCOPED_TRACE(seed);
        Stopwatch watch;
        const GeneratedTrack generated = generate_track_with_stats(seed);
        time.add(watch.elapsed_seconds());
        attempts += generated.attempts;
        const Track& track = generated.track;
        EXPECT_NO_THROW(validate_track(track));
        const std::optional<std::string> broken = check_track_rules(track);
        EXPECT_FALSE(broken.has_value()) << *broken;
        const TrackStats stats = analyze_track(track);
        EXPECT_GE(stats.hairpins, 1);
        EXPECT_GE(stats.chicanes, 1);
        EXPECT_GE(stats.sweepers, 1);
        total.length += stats.length;
        total.hairpins += stats.hairpins;
        total.chicanes += stats.chicanes;
        total.sweepers += stats.sweepers;
        total.corners += stats.corners;
    }
    std::cout << "seeds " << first << "-" << first + kShardSize - 1 << ": mean length "
              << total.length / kShardSize << " m, mean attempts "
              << static_cast<double>(attempts) / kShardSize << ", generation p50 "
              << time.percentile(0.5) * 1e3 << " ms, max " << time.max() * 1e3
              << " ms; features: " << total.hairpins << " hairpins, " << total.chicanes
              << " chicanes, " << total.sweepers << " sweepers, " << total.corners << " corners\n";
}

INSTANTIATE_TEST_SUITE_P(Shards, TrackGenSeeds, testing::Range(0, 4));

}  // namespace
}  // namespace apex::sim
