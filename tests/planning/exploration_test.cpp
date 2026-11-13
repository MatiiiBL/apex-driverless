#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/track.hpp"
#include "apex/planning/planning.hpp"
#include "apex/sim/track_gen.hpp"

#include "cone_maps.hpp"
#include "exploration_eval.hpp"

namespace apex::planning {
namespace {

using test_support::MapErrors;

StateEstimate state_at(const Pose2& pose, double speed) {
    StateEstimate state;
    state.pose = pose;
    state.vx = speed;
    state.valid = true;
    return state;
}

// A straight corridor along +x: blue cones at y = 1.75, yellow at y = -1.75, every 4 m.
ConeMap corridor(double length) {
    ConeMap map;
    for (double x = 0.0; x <= length; x += 4.0) {
        for (const auto& [y, color] :
             {std::pair{1.75, ConeColor::Blue}, std::pair{-1.75, ConeColor::Yellow}}) {
            MapCone cone;
            cone.id = static_cast<std::int32_t>(map.cones.size());
            cone.position = {x, y};
            cone.color = color;
            cone.color_votes[index_of(color)] = 5;
            map.cones.push_back(cone);
        }
    }
    return map;
}

TEST(ExplorationPlanner, FollowsAStraightCorridorAndStopsBeforeItsEnd) {
    ExplorationPlanner planner({}, VehicleParams{});
    const ExplorationConfig config;
    const std::optional<Trajectory> path = planner.plan(corridor(24.0), state_at({}, 5.0));
    ASSERT_TRUE(path.has_value());
    EXPECT_FALSE(path->closed());
    EXPECT_EQ((*path)[0].position, Vec2(0.0, 0.0));
    EXPECT_GT(path->length(), 18.0);
    EXPECT_LE(path->length(), 24.5);
    // It stops with stop_reserve of the path to spare, in case the next plan's path is shorter.
    const double stop_s = (1.0 - config.stop_reserve) * path->length();
    for (const TrajectoryPoint& p : path->points()) {
        EXPECT_LT(std::abs(p.position.y()), 0.1);
        EXPECT_NEAR(p.width_left, 1.75, 0.15);
        EXPECT_NEAR(p.width_right, 1.75, 0.15);
        if (p.s >= stop_s) {
            EXPECT_EQ(p.speed, 0.0);
        }
    }
    EXPECT_GT(path->at(stop_s - 0.6).speed, 0.0);
    EXPECT_NEAR((*path)[0].speed, 5.0, 1e-9);
    EXPECT_GT(path->at(0.25 * path->length()).speed, 5.0);
    EXPECT_EQ(planner.stats().plans, 1U);
    EXPECT_EQ(planner.stats().failures, 0U);
}

// A lap of plans (test_support::evaluate_exploration), summarized.
struct Summary {
    std::size_t plans = 0;
    std::size_t found = 0;
    std::size_t inside = 0;  // paths between the true cone lines over their whole length
    double mean_deviation = 0.0;
};

Summary summarize(const Track& track, const MapErrors& errors, std::uint64_t seed) {
    const std::vector<test_support::PlanRecord> records = test_support::evaluate_exploration(
        track, {.errors = errors, .seed = seed, .plan_every = 10.0});
    Summary summary;
    summary.plans = records.size();
    for (const test_support::PlanRecord& r : records) {
        if (!r.found) continue;
        ++summary.found;
        if (r.inside) ++summary.inside;
        summary.mean_deviation += r.mean_deviation;
    }
    summary.mean_deviation /= static_cast<double>(std::max<std::size_t>(1, summary.found));
    return summary;
}

TEST(ExplorationPlanner, FollowsGeneratedTracksFromImperfectMaps) {
    for (const std::string_view name : {"technical", "mixed"}) {
        const Summary s = summarize(sim::reference_track(name), MapErrors{}, 1);
        EXPECT_GE(s.found, s.plans - s.plans / 20) << name;
        EXPECT_GE(static_cast<double>(s.inside), 0.95 * static_cast<double>(s.found)) << name;
        EXPECT_LT(s.mean_deviation, 0.3) << name;
    }
}

TEST(ExplorationPlanner, CopesWithTheWorstMaps) {
    const Summary s =
        summarize(sim::reference_track("technical"), test_support::worst_case_errors(), 2);
    EXPECT_GE(s.found, s.plans - s.plans / 10);
    EXPECT_GE(static_cast<double>(s.inside), 0.85 * static_cast<double>(s.found));
}

TEST(ExplorationPlanner, NoPathWithoutConesAheadOrAValidState) {
    ExplorationPlanner planner({}, VehicleParams{});
    EXPECT_FALSE(planner.plan(ConeMap{}, state_at({}, 0.0)).has_value());
    // At x = 40 facing +x, every cone of the corridor is behind the car.
    EXPECT_FALSE(planner.plan(corridor(24.0), state_at({40.0, 0.0, 0.0}, 0.0)).has_value());
    StateEstimate invalid = state_at({}, 0.0);
    invalid.valid = false;
    EXPECT_FALSE(planner.plan(corridor(24.0), invalid).has_value());
    EXPECT_EQ(planner.stats().plans, 3U);
    EXPECT_EQ(planner.stats().failures, 3U);
    EXPECT_EQ(planner.stats().plan_time.count(), 3U);
}

TEST(ExplorationPlanner, IgnoresTruthIds) {
    const Track track = make_oval_track();
    Rng rng(9);
    ConeMap map = test_support::make_cone_map(track, MapErrors{}, rng);
    const StateEstimate state = state_at(test_support::centerline_pose(track, 30.0), 6.0);
    ExplorationPlanner planner({}, VehicleParams{});
    const std::optional<Trajectory> before = planner.plan(map, state);
    for (std::size_t i = 0; i < map.cones.size(); ++i) {
        map.cones[i].truth_id = static_cast<std::int32_t>((i * 7919) % 1000);
    }
    const std::optional<Trajectory> after = planner.plan(map, state);
    ASSERT_TRUE(before.has_value());
    ASSERT_TRUE(after.has_value());
    ASSERT_EQ(before->size(), after->size());
    for (std::size_t i = 0; i < before->size(); ++i) {
        EXPECT_EQ((*before)[i].position, (*after)[i].position);
        EXPECT_EQ((*before)[i].speed, (*after)[i].speed);
    }
}

// A map of uncolored landmarks, as perception leaves many at high noise.
ConeMap uncolored(std::initializer_list<Vec2> positions) {
    ConeMap map;
    for (const Vec2& p : positions) {
        MapCone cone;
        cone.id = static_cast<std::int32_t>(map.cones.size());
        cone.position = p;
        map.cones.push_back(cone);
    }
    return map;
}

double nearest_cone(const Vec2& p, const ConeMap& map) {
    double nearest = 1e9;
    for (const MapCone& cone : map.cones) nearest = std::min(nearest, (cone.position - p).norm());
    return nearest;
}

// When no edge is within start_radius (6 m), the first edge is the nearest usable one further
// out, with the path running straight to it from the car.
TEST(ExplorationPlanner, StartsAtTheNearestEdgeWhenNoneIsClose) {
    ConeMap far = corridor(36.0);
    std::erase_if(far.cones, [](const MapCone& c) { return c.position.x() < 11.0; });
    ExplorationPlanner planner({}, VehicleParams{});
    const std::optional<Trajectory> path = planner.plan(far, state_at({}, 0.0));
    ASSERT_TRUE(path.has_value());
    EXPECT_EQ((*path)[0].position, Vec2(0.0, 0.0));
    EXPECT_GT(path->length(), 20.0);
    for (const TrajectoryPoint& p : path->points()) EXPECT_LT(std::abs(p.position.y()), 0.1);

    // A map recorded in closed loop at four times the sensor noise, with the car staged at
    // (-6, 0) facing +x: the big orange cones and the first yellow cone were never mapped, the
    // nearest track edge is 13.9 m away, and a hull edge out to another part of the track, at
    // 12.5 m, is nearer but no start (too wide, too sharp a turn). The run failed with no path.
    const ConeMap recorded =
        uncolored({{10.27, 1.80},  {3.46, 16.04},  {14.15, -0.90}, {6.28, 19.61},  {17.21, 0.95},
                   {12.14, 15.88}, {17.33, 9.62},  {19.45, 3.68},  {20.67, 6.99},  {19.69, 13.89},
                   {-3.66, 19.26}, {13.31, 2.55},  {1.33, 19.48},  {16.51, 12.35}, {10.42, -1.68},
                   {8.34, 16.20},  {14.71, 14.58}, {14.61, 18.69}, {-1.49, 15.84}, {15.61, 4.30},
                   {5.43, 1.75}});
    const std::optional<Trajectory> staged =
        planner.plan(recorded, state_at({-6.0, 0.0, 0.0}, 0.0));
    ASSERT_TRUE(staged.has_value());
    EXPECT_GT(staged->length(), 15.0);
    for (const TrajectoryPoint& p : staged->points()) {
        if (p.position.x() < 12.0) {
            EXPECT_LT(std::abs(p.position.y()), 0.8) << "x = " << p.position.x();
        }
    }
}

// A map recorded in closed loop at four times the sensor noise, the car entering a right-hand
// hairpin, colors removed. Without them, a path straight across the hairpin's infield, between
// cones of its inner line, scored as well as the one around it; the car followed it into a cone.
// The planner now keeps its path a meter from every mapped cone, which rules the shortcut out.
TEST(ExplorationPlanner, KeepsClearOfConesItDoesNotCross) {
    const ConeMap map = uncolored(
        {{15.34, -1.76},  {15.46, 1.76},   {7.16, -21.63},  {20.24, -1.78},  {11.52, -19.28},
         {20.41, 1.72},   {16.15, -17.52}, {10.21, -23.83}, {0.27, -28.65},  {10.38, -1.73},
         {10.45, 1.75},   {3.35, -24.77},  {6.25, -26.81},  {24.10, 0.94},   {14.73, -21.78},
         {20.84, -15.89}, {25.53, -4.20},  {23.33, -2.52},  {26.87, -6.67},  {19.43, -20.11},
         {24.37, -14.27}, {27.14, -0.89},  {27.18, -9.52},  {26.26, -12.19}, {29.38, -3.60},
         {24.09, -18.42}, {30.56, -10.36}, {30.51, -6.88},  {29.41, -13.68}, {27.26, -16.44}});
    ExplorationPlanner planner({}, VehicleParams{});
    const std::optional<Trajectory> path =
        planner.plan(map, state_at({23.147, -0.929, -0.388}, 6.4));
    ASSERT_TRUE(path.has_value());
    EXPECT_GT(path->length(), 20.0);  // around the hairpin, not 12.8 m across it
    for (const TrajectoryPoint& p : path->points()) {
        EXPECT_GT(nearest_cone(p.position, map), 0.9) << "s = " << p.s;
    }
}

TEST(ExplorationPlanner, RejectsInvalidConfig) {
    ExplorationConfig config;
    config.beam_width = 0;
    EXPECT_THROW(ExplorationPlanner(config, VehicleParams{}), std::invalid_argument);
    config = {};
    config.min_path_length = config.stop_margin;
    EXPECT_THROW(ExplorationPlanner(config, VehicleParams{}), std::invalid_argument);
}

}  // namespace
}  // namespace apex::planning
