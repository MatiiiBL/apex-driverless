// Closed-loop runs of the whole car: perception, estimation, planning and control behind the
// mission, in the simulator (docs/design.md, section 7).

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "apex/runner/run_stack.hpp"
#include "apex/runner/sweep.hpp"
#include "apex/stack/stack.hpp"

#include "shuffled_truth_ids.hpp"

namespace apex::runner {
namespace {

using stack::MissionState;

// Stanley follows the exploration planner's paths, replanned at every scan, most closely of the
// three controllers (docs/evaluation.md), so it drives these tests.
RunConfig mission(TrackSpec track, int laps) {
    RunConfig config;
    config.track = std::move(track);
    config.stack.mission.laps = laps;
    config.stack.controller.kind = control::ControllerKind::Stanley;
    config.dnf.time_limit = 150.0;
    return config;
}

TrackSpec oval() {
    return TrackSpec{};
}

TrackSpec generated(std::uint64_t seed) {
    TrackSpec spec;
    spec.source = TrackSpec::Source::Generated;
    spec.seed = seed;
    return spec;
}

class CommandLog final : public RunObserver {
public:
    void on_start(const Track& /*track*/) override {}
    void on_control(const TickView& tick) override { commands.push_back(tick.command); }
    std::vector<ControlCommand> commands;
};

std::vector<MissionState> states_entered(const RunResult& r) {
    std::vector<MissionState> states;
    for (const stack::MissionTransition& t : r.transitions) states.push_back(t.to);
    return states;
}

bool entered(const RunResult& r, MissionState state) {
    const std::vector<MissionState> states = states_entered(r);
    return std::find(states.begin(), states.end(), state) != states.end();
}

// Finished every lap with no penalty and came to rest. The run ends once the car is at rest,
// which can be a control step before the stack reports Finished.
void expect_clean_finish(const RunResult& r, std::size_t laps) {
    EXPECT_TRUE(r.error.empty()) << r.error;
    EXPECT_TRUE(r.completed) << r.dnf_reason;
    EXPECT_TRUE(r.stopped);
    EXPECT_EQ(r.cones_hit, 0U);
    EXPECT_EQ(r.off_course, 0U);
    EXPECT_FALSE(entered(r, MissionState::Failed));
    EXPECT_EQ(r.lap_times.size(), laps);

    EXPECT_GT(r.map.coverage, 0.95);
    EXPECT_EQ(r.map.false_landmarks, 0U);
    EXPECT_LT(r.map.position.rms, 0.05);
    EXPECT_GT(r.association.precision(), 0.99);
    EXPECT_GT(r.association.recall(), 0.9);
    const PhaseMetrics& exploring = r.phases[stack::index_of(MissionState::Exploring)];
    EXPECT_LT(exploring.position.rms, 0.15);
    EXPECT_LT(exploring.lateral.rms, 0.1);
}

// The whole mission in two laps: explore and map, race on the raceline, stop.
void expect_clean_mission(const RunResult& r) {
    expect_clean_finish(r, 2);
    const std::vector<MissionState> states = states_entered(r);
    ASSERT_GE(states.size(), 4U);
    EXPECT_EQ(std::vector<MissionState>(states.begin(), states.begin() + 4),
              (std::vector<MissionState>{MissionState::Exploring, MissionState::MapFinalized,
                                         MissionState::Racing, MissionState::Stopping}));
    EXPECT_TRUE(r.map.finalized);
    ASSERT_EQ(r.lap_times.size(), 2U);
    EXPECT_LT(r.lap_times[1], 0.9 * r.lap_times[0]);  // racing beats exploring
    const PhaseMetrics& racing = r.phases[stack::index_of(MissionState::Racing)];
    EXPECT_LT(racing.position.rms, 0.15);
}

TEST(FullStack, FinishesTheOvalCleanly) {
    expect_clean_mission(run_stack(mission(oval(), 2)));
}

TEST(FullStack, FinishesAGeneratedTrackCleanly) {
    expect_clean_mission(run_stack(mission(generated(3), 2)));
}

TEST(FullStack, FinishesCleanlyWithTheOracles) {
    const std::vector<std::pair<bool, bool>> oracles = {{true, false}, {false, true}, {true, true}};
    for (const auto& [perception, state] : oracles) {
        RunConfig config = mission(oval(), 2);
        config.oracles.perception = perception;
        config.oracles.state = state;
        SCOPED_TRACE(::testing::Message() << "perception " << perception << ", state " << state);
        const RunResult r = run_stack(config);
        expect_clean_mission(r);
        if (state) {
            EXPECT_DOUBLE_EQ(r.phases[stack::index_of(MissionState::Racing)].position.rms, 0.0);
        }
    }
}

// A one-lap mission stops straight from the exploration path, with no raceline.
TEST(FullStack, ExploresAndStopsCleanly) {
    const RunResult r = run_stack(mission(oval(), 1));
    expect_clean_finish(r, 1);
    const std::vector<MissionState> states = states_entered(r);
    ASSERT_GE(states.size(), 2U);
    EXPECT_EQ(states[0], MissionState::Exploring);
    EXPECT_EQ(states[1], MissionState::Stopping);
    EXPECT_FALSE(entered(r, MissionState::Racing));
}

// docs/design.md, section 4.3: truth ids are for the evaluation only. Relabelling every one of
// them must not change a single command, over a mission that maps, races and stops.
TEST(FullStack, TruthIdsNeverChangeACommand) {
    const RunConfig config = mission(oval(), 2);
    const Track track = make_track(config.track);
    const auto commands = [&](bool shuffle) {
        stack::Stack car(config.stack, config.nominal, make_world_config(config).rig,
                         track.start_pose, config.oracles);
        test_drivers::ShuffledTruthIds relabelled(car, track.cones.size(), 2024);
        CommandLog log;
        const RunResult r = shuffle ? run_closed_loop(config, track, relabelled, &log)
                                    : run_closed_loop(config, track, car, &log);
        EXPECT_TRUE(entered(r, MissionState::Racing));
        EXPECT_TRUE(entered(r, MissionState::Stopping));
        return log.commands;
    };
    const std::vector<ControlCommand> plain = commands(false);
    const std::vector<ControlCommand> shuffled = commands(true);
    ASSERT_EQ(plain.size(), shuffled.size());
    for (std::size_t i = 0; i < plain.size(); ++i) {
        ASSERT_TRUE(plain[i].steer == shuffled[i].steer && plain[i].accel == shuffled[i].accel)
            << "the commands first differ at t = " << plain[i].stamp << " s";
    }
}

// Runs share nothing, so a sweep's results cannot depend on how many run at once. Under the
// tsan preset this is also the data-race check of the thread pool with real runs.
TEST(FullStack, SweepResultsIndependentOfThreadCount) {
    SweepConfig sweep;
    RunConfig base;
    base.dnf.time_limit = 3.0;  // staging and the first seconds of exploration
    sweep.base = base;
    sweep.seeds = {1, 2, 3};
    const std::vector<SweepRun> runs = expand_sweep(sweep);
    const RunFunction run = [](const RunConfig& c) { return run_stack(c); };
    const std::vector<RunResult> serial = run_parallel(runs, run, 1);
    const std::vector<RunResult> parallel = run_parallel(runs, run, 3);
    ASSERT_EQ(serial.size(), 3U);
    for (std::size_t i = 0; i < runs.size(); ++i) {
        EXPECT_TRUE(serial[i].error.empty()) << serial[i].error;
        EXPECT_EQ(deterministic_json(serial[i]), deterministic_json(parallel[i]));
    }
    // Different seeds give different noise, so different runs.
    EXPECT_NE(deterministic_json(serial[0])["phases"], deterministic_json(serial[1])["phases"]);
}

}  // namespace
}  // namespace apex::runner
