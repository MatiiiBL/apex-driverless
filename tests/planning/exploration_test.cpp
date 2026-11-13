#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
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
    for (const TrajectoryPoint& p : path->points()) {
        EXPECT_LT(std::abs(p.position.y()), 0.1);
        EXPECT_NEAR(p.width_left, 1.75, 0.15);
        EXPECT_NEAR(p.width_right, 1.75, 0.15);
        if (p.s >= path->length() - config.stop_margin) {
            EXPECT_EQ(p.speed, 0.0);
        }
    }
    EXPECT_NEAR((*path)[0].speed, 5.0, 1e-9);
    EXPECT_GT(path->at(0.5 * path->length()).speed, 5.0);
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
    for (const std::string& name : {"technical", "mixed"}) {
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
