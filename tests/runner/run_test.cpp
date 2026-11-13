#include "apex/runner/run.hpp"

#include <cmath>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/track.hpp"
#include "apex/runner/recorders.hpp"

#include "centerline_driver.hpp"

namespace apex::runner {
namespace {

using stack::MissionState;
using test_drivers::CenterlineDriver;
using test_drivers::CenterlineDriverOptions;
using test_drivers::Delivery;
using test_drivers::Fault;

// A short oval: 103 m a lap, about 13 s at 8 m/s.
const Track& small_oval() {
    static const Track track = make_oval_track(20.0, 10.0);
    return track;
}

// The centerline driver drives on the true state, so every config turns that oracle on.
RunConfig config_for(int laps) {
    RunConfig config;
    config.oracles.state = true;
    config.stack.mission.laps = laps;
    config.dnf.time_limit = 120.0;
    return config;
}

// What the race officials said at the last control step.
class OfficialsRecord final : public RunObserver {
public:
    void on_start(const Track& /*track*/) override {}
    void on_control(const TickView& tick) override {
        laps.clear();
        for (const sim::LapRecord& lap : tick.race.laps()) laps.push_back(lap.time());
        penalty_seconds = tick.race.penalty_seconds();
        ++controls;
    }
    std::vector<double> laps;
    double penalty_seconds = -1.0;
    int controls = 0;
};

RunResult drive(const RunConfig& config, const CenterlineDriverOptions& options,
                RunObserver* observer = nullptr) {
    CenterlineDriver driver(small_oval(), config.nominal, options);
    return run_closed_loop(config, small_oval(), driver, observer);
}

std::vector<Delivery> deliveries(const RunConfig& config, const CenterlineDriverOptions& options) {
    CenterlineDriver driver(small_oval(), config.nominal, options);
    (void)run_closed_loop(config, small_oval(), driver);
    return driver.deliveries();
}

TEST(RunClosedLoop, CompletesLapsScoredByTheOfficials) {
    CenterlineDriverOptions options;
    options.laps = 2;
    OfficialsRecord officials;
    const RunResult r = drive(config_for(2), options, &officials);

    EXPECT_TRUE(r.completed) << r.dnf_reason;
    EXPECT_TRUE(r.stopped);
    EXPECT_TRUE(r.dnf_reason.empty());
    ASSERT_EQ(r.lap_times.size(), 2U);
    ASSERT_EQ(officials.laps.size(), 2U);
    const double nominal_lap = small_oval().centerline.length() / options.speed;
    for (std::size_t i = 0; i < 2; ++i) {
        EXPECT_DOUBLE_EQ(r.lap_times[i], officials.laps[i]);
        EXPECT_NEAR(r.lap_times[i], nominal_lap, 0.15 * nominal_lap);
    }
    EXPECT_EQ(r.cones_hit, 0U);
    EXPECT_EQ(r.off_course, 0U);
    EXPECT_DOUBLE_EQ(r.penalty_seconds, officials.penalty_seconds);
    EXPECT_DOUBLE_EQ(r.score(), r.total_time());
    EXPECT_EQ(r.laps_target, 2);
    EXPECT_DOUBLE_EQ(r.track_length, small_oval().centerline.length());

    // The map was scored when lap 1 closed; the driver's map is the truth.
    EXPECT_TRUE(r.map.finalized);
    EXPECT_DOUBLE_EQ(r.map.coverage, 1.0);
    EXPECT_DOUBLE_EQ(r.map.position.rms, 0.0);

    const PhaseMetrics& racing = r.phases[stack::index_of(MissionState::Racing)];
    EXPECT_GT(racing.duration, 0.5 * nominal_lap);
    EXPECT_GT(racing.position.samples, 0U);
    EXPECT_DOUBLE_EQ(racing.position.rms, 0.0);  // the estimate is the truth
    EXPECT_LT(racing.lateral.rms, 0.3);
    double total = 0.0;
    for (const PhaseMetrics& phase : r.phases) total += phase.duration;
    EXPECT_NEAR(total, r.sim_time, 1e-6);
    EXPECT_GT(r.max_lateral_true, 0.0);
    EXPECT_LT(r.max_lateral_true, 1.0);
    EXPECT_TRUE(r.compute.contains("sim_tick"));
}

TEST(RunClosedLoop, SensorMessagesArriveAfterTheirLatency) {
    RunConfig config = config_for(1);
    config.dnf.time_limit = 1.0;
    config.latency.imu = 0.003;
    config.latency.lidar = 0.080;
    CenterlineDriverOptions options;
    options.fault = Fault::Stall;
    const std::vector<Delivery> received = deliveries(config, options);

    std::map<std::string, double> latency = {
        {"imu", 0.003}, {"wheels", 0.0}, {"steering", 0.0}, {"camera", 0.040}, {"lidar", 0.080}};
    std::map<std::string, int> counts;
    double previous_arrival = 0.0;
    for (const Delivery& d : received) {
        ASSERT_TRUE(latency.contains(d.kind)) << d.kind;
        EXPECT_NEAR(d.arrival - d.stamp, latency[d.kind], 1e-9) << d.kind << " @ " << d.stamp;
        EXPECT_GE(d.arrival, previous_arrival);
        previous_arrival = d.arrival;
        ++counts[d.kind];
    }
    // Stamps up to 1 s, delivered by 1 s: IMU at 200 Hz, wheels and steering at 100 Hz, the
    // camera at 20 Hz and the LiDAR at 10 Hz.
    EXPECT_EQ(counts["imu"], 199);
    EXPECT_EQ(counts["wheels"], 100);
    EXPECT_EQ(counts["steering"], 100);
    EXPECT_EQ(counts["camera"], 19);
    EXPECT_EQ(counts["lidar"], 9);
}

TEST(RunClosedLoop, PerfectPerceptionReplacesScansAndFrames) {
    RunConfig config = config_for(1);
    config.dnf.time_limit = 1.0;
    config.oracles.perception = true;
    CenterlineDriverOptions options;
    options.fault = Fault::Stall;
    int observations = 0;
    for (const Delivery& d : deliveries(config, options)) {
        EXPECT_NE(d.kind, "lidar");
        EXPECT_NE(d.kind, "camera");
        if (d.kind != "oracle_observations") continue;
        ++observations;
        EXPECT_NEAR(d.arrival - d.stamp, 0.050, 1e-9);
        EXPECT_NEAR(std::remainder(d.stamp, 0.1), 0.0, 1e-9);
        EXPECT_GT(d.cones, 0U);  // the start straight is lined with cones
    }
    EXPECT_EQ(observations, 9);
}

struct DnfCase {
    const char* name;
    Fault fault;
    const char* reason;  // a substring of the DNF reason
};

class RunClosedLoopDnf : public ::testing::TestWithParam<DnfCase> {};

TEST_P(RunClosedLoopDnf, EndsTheRunWithAReason) {
    RunConfig config = config_for(1);
    config.dnf.stall_time = 2.0;
    config.dnf.max_off_track = 1.0;
    config.dnf.time_limit = 30.0;
    CenterlineDriverOptions options;
    options.fault = GetParam().fault;
    options.fault_time = 1.5;
    const RunResult r = drive(config, options);
    EXPECT_FALSE(r.completed);
    EXPECT_NE(r.dnf_reason.find(GetParam().reason), std::string::npos) << r.dnf_reason;
    EXPECT_LT(r.sim_time, 10.0);
}

INSTANTIATE_TEST_SUITE_P(
    EveryRule, RunClosedLoopDnf,
    ::testing::Values(DnfCase{"Stalled", Fault::Stall, "stalled for 2 s"},
                      DnfCase{"OffTrack", Fault::Veer, "off the track"},
                      DnfCase{"MissionFailed", Fault::Fail, "mission failed: test fault"},
                      DnfCase{"NonFinite", Fault::NonFinite, "non-finite control command"},
                      DnfCase{"FinishedEarly", Fault::FinishEarly,
                              "the mission ended after 0 of 1 laps"}),
    [](const ::testing::TestParamInfo<DnfCase>& info) { return std::string(info.param.name); });

TEST(RunClosedLoop, TimeLimit) {
    RunConfig config = config_for(1);
    config.dnf.time_limit = 1.5;
    const RunResult r = drive(config, CenterlineDriverOptions{});
    EXPECT_FALSE(r.completed);
    EXPECT_NE(r.dnf_reason.find("time limit"), std::string::npos) << r.dnf_reason;
    EXPECT_NEAR(r.sim_time, 1.501, 1e-9);
}

TEST(RunClosedLoop, ACarThatNeverStopsIsTimedButNotStopped) {
    RunConfig config = config_for(1);
    config.dnf.stop_timeout = 2.0;
    CenterlineDriverOptions options;
    options.fault = Fault::NeverStop;
    options.fault_time = 1e9;
    const RunResult r = drive(config, options);
    EXPECT_TRUE(r.completed) << r.dnf_reason;
    EXPECT_FALSE(r.stopped);
    ASSERT_EQ(r.lap_times.size(), 1U);
    EXPECT_GT(r.sim_time, r.lap_times[0] + 2.0);
}

TEST(RunClosedLoop, InvalidConfigThrows) {
    RunConfig config = config_for(1);
    config.latency.lidar = 0.0505;
    CenterlineDriver driver(small_oval(), config.nominal, {});
    EXPECT_THROW((void)run_closed_loop(config, small_oval(), driver), std::invalid_argument);
}

TEST(RunClosedLoop, DeterministicAndRecorded) {
    const RunConfig config = config_for(1);
    std::ostringstream csv;
    CsvLog log(csv, 0.1);
    ReplayRecorder replay;
    const RunResult first = drive(config, CenterlineDriverOptions{}, &log);
    const RunResult second = drive(config, CenterlineDriverOptions{}, &replay);
    EXPECT_EQ(deterministic_json(first), deterministic_json(second));
    ASSERT_TRUE(first.completed) << first.dnf_reason;

    // One CSV row per 0.1 s, the first at the first control step.
    EXPECT_NEAR(static_cast<double>(log.rows()), first.sim_time / 0.1, 1.5);
    std::istringstream lines(csv.str());
    std::string header;
    std::getline(lines, header);
    EXPECT_EQ(header.substr(0, 8), "t,state,");
    std::size_t rows = 0;
    for (std::string line; std::getline(lines, line);) ++rows;
    EXPECT_EQ(rows, log.rows());

    const nlohmann::json& j = replay.replay();
    EXPECT_EQ(j["track"]["cones"].size(), small_oval().cones.size());
    EXPECT_NEAR(static_cast<double>(j["frames"].size()), second.sim_time / 0.05, 1.5);
    EXPECT_GE(j["maps"].size(), 1U);
    ASSERT_GE(j["references"].size(), 1U);
    EXPECT_TRUE(j["references"][0]["closed"].get<bool>());
}

}  // namespace
}  // namespace apex::runner
