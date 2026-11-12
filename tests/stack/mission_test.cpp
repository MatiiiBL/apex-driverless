#include "apex/stack/mission.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace apex::stack {
namespace {

// A healthy car at time t that has a fresh path.
MissionInputs healthy(Time t, int laps = 0, double speed = 5.0) {
    MissionInputs in;
    in.now = t;
    in.estimate_usable = true;
    in.speed = speed;
    in.laps_completed = laps;
    in.last_path_time = t;
    return in;
}

TEST(Mission, GoesAfterTheStartDelayOnceTheEstimateIsUsable) {
    MissionConfig config;
    config.start_delay = 0.5;
    Mission mission(config);
    EXPECT_EQ(mission.update(healthy(0.48)), MissionState::Staged);

    MissionInputs unusable = healthy(0.5);
    unusable.estimate_usable = false;
    EXPECT_EQ(mission.update(unusable), MissionState::Staged);
    EXPECT_EQ(mission.update(healthy(0.52)), MissionState::Exploring);
    ASSERT_EQ(mission.transitions().size(), 1U);
    EXPECT_DOUBLE_EQ(mission.transitions()[0].time, 0.52);
    EXPECT_EQ(mission.transitions()[0].from, MissionState::Staged);
    EXPECT_EQ(mission.transitions()[0].to, MissionState::Exploring);
}

TEST(Mission, FullTrackdriveSequence) {
    MissionConfig config;
    config.laps = 3;
    config.start_delay = 0.0;
    config.raceline_delay = 1.0;
    Mission mission(config);
    EXPECT_EQ(mission.update(healthy(0.0)), MissionState::Exploring);
    // Closing lap 1 does not finalize by itself: the stack reports a built track model.
    EXPECT_EQ(mission.update(healthy(30.0, 1)), MissionState::Exploring);
    mission.map_finalized(30.0);
    EXPECT_EQ(mission.state(), MissionState::MapFinalized);
    EXPECT_EQ(mission.update(healthy(30.98, 1)), MissionState::MapFinalized);
    EXPECT_EQ(mission.update(healthy(31.0, 1)), MissionState::Racing);
    EXPECT_EQ(mission.update(healthy(50.0, 2)), MissionState::Racing);
    // Racing needs no fresh exploration path.
    MissionInputs racing = healthy(60.0, 2);
    racing.last_path_time = 29.0;
    EXPECT_EQ(mission.update(racing), MissionState::Racing);
    EXPECT_EQ(mission.update(healthy(70.0, 3, 15.0)), MissionState::Stopping);
    EXPECT_EQ(mission.update(healthy(72.0, 3, 0.5)), MissionState::Stopping);
    EXPECT_EQ(mission.update(healthy(73.0, 3, 0.05)), MissionState::Finished);
    // Terminal states stick, whatever happens next.
    MissionInputs broken = healthy(74.0, 3);
    broken.estimate_finite = false;
    EXPECT_EQ(mission.update(broken), MissionState::Finished);
    EXPECT_EQ(mission.transitions().size(), 5U);
}

TEST(Mission, MapFinalizedOnlyFromExploring) {
    Mission mission(MissionConfig{});
    mission.map_finalized(0.1);
    EXPECT_EQ(mission.state(), MissionState::Staged);
    EXPECT_TRUE(mission.transitions().empty());
}

TEST(Mission, StopsAfterTheLastLapEvenWithoutARaceline) {
    MissionConfig config;
    config.laps = 2;
    config.start_delay = 0.0;
    Mission mission(config);
    (void)mission.update(healthy(0.0));
    EXPECT_EQ(mission.update(healthy(40.0, 1)), MissionState::Exploring);
    EXPECT_EQ(mission.update(healthy(80.0, 2)), MissionState::Stopping);
}

TEST(Mission, SingleLapStopsRightAfterFinalizing) {
    MissionConfig config;
    config.laps = 1;
    config.start_delay = 0.0;
    Mission mission(config);
    (void)mission.update(healthy(0.0));
    mission.map_finalized(40.0);
    EXPECT_EQ(mission.update(healthy(40.02, 1)), MissionState::Stopping);
}

TEST(Mission, FailsAtOnceOnANonFiniteEstimate) {
    Mission mission(MissionConfig{});
    MissionInputs in = healthy(0.1);
    in.estimate_finite = false;
    EXPECT_EQ(mission.update(in), MissionState::Failed);
    EXPECT_NE(mission.transitions().back().reason.find("non-finite"), std::string::npos);
}

TEST(Mission, FailsWhenLocalizationStaysLost) {
    MissionConfig config;
    config.start_delay = 0.0;
    config.max_lost_time = 0.5;
    Mission mission(config);
    (void)mission.update(healthy(0.0));
    MissionInputs lost = healthy(1.0);
    lost.estimate_usable = false;
    EXPECT_EQ(mission.update(lost), MissionState::Exploring);
    // Recovering within the allowance resets the timer.
    EXPECT_EQ(mission.update(healthy(1.4)), MissionState::Exploring);
    lost.now = 1.6;
    EXPECT_EQ(mission.update(lost), MissionState::Exploring);
    lost.now = 2.05;
    EXPECT_EQ(mission.update(lost), MissionState::Exploring);
    lost.now = 2.12;
    EXPECT_EQ(mission.update(lost), MissionState::Failed);
    EXPECT_NE(mission.transitions().back().reason.find("localization"), std::string::npos);
}

TEST(Mission, FailsWithoutAnExplorationPathForTooLong) {
    MissionConfig config;
    config.start_delay = 0.0;
    config.max_no_path_time = 3.0;
    Mission mission(config);
    MissionInputs in = healthy(0.0);
    in.last_path_time = -1.0;
    (void)mission.update(in);
    // The first path may take max_no_path_time from the start of exploration.
    in.now = 2.9;
    EXPECT_EQ(mission.update(in), MissionState::Exploring);
    in.last_path_time = 2.9;
    in.now = 5.8;
    EXPECT_EQ(mission.update(in), MissionState::Exploring);
    in.now = 5.92;
    EXPECT_EQ(mission.update(in), MissionState::Failed);
    EXPECT_NE(mission.transitions().back().reason.find("no exploration path"), std::string::npos);
}

TEST(Mission, ExplicitFailureFromAnyRunningState) {
    Mission mission(MissionConfig{});
    mission.fail(0.2, "test");
    EXPECT_EQ(mission.state(), MissionState::Failed);
    EXPECT_EQ(mission.transitions().back().reason, "test");
    mission.fail(0.3, "again");
    EXPECT_EQ(mission.transitions().size(), 1U);
}

TEST(Mission, RejectsInvalidConfigs) {
    MissionConfig no_laps;
    no_laps.laps = 0;
    EXPECT_THROW(Mission{no_laps}, std::invalid_argument);
    MissionConfig negative;
    negative.raceline_delay = -1.0;
    EXPECT_THROW(Mission{negative}, std::invalid_argument);
    MissionConfig nan;
    nan.max_no_path_time = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(Mission{nan}, std::invalid_argument);
}

TEST(Mission, ConfigJson) {
    MissionConfig config;
    config.laps = 4;
    config.raceline_delay = 2.5;
    const nlohmann::json j = config;
    const auto back = j.get<MissionConfig>();
    EXPECT_EQ(back.laps, 4);
    EXPECT_DOUBLE_EQ(back.raceline_delay, 2.5);
    EXPECT_DOUBLE_EQ(nlohmann::json::object().get<MissionConfig>().start_delay,
                     MissionConfig{}.start_delay);
    EXPECT_THROW((void)nlohmann::json({{"lap", 3}}).get<MissionConfig>(), std::invalid_argument);
}

TEST(MissionState, Names) {
    EXPECT_EQ(to_string(MissionState::MapFinalized), "map_finalized");
    EXPECT_TRUE(is_terminal(MissionState::Failed));
    EXPECT_FALSE(is_terminal(MissionState::Stopping));
}

TEST(AssociationCounts, PrecisionAndRecall) {
    AssociationCounts counts;
    EXPECT_EQ(counts.precision(), 0.0);
    EXPECT_EQ(counts.recall(), 0.0);
    counts.matched = 10;
    counts.correct = 9;
    counts.matchable = 12;
    EXPECT_DOUBLE_EQ(counts.precision(), 0.9);
    EXPECT_DOUBLE_EQ(counts.recall(), 0.75);
}

}  // namespace
}  // namespace apex::stack
