#include "apex/runner/run.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <string>
#include <variant>
#include <vector>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/timing.hpp"
#include "apex/runner/delay_queue.hpp"
#include "apex/runner/oracle.hpp"

namespace apex::runner {

namespace {

// The runner's own random stream (the oracle's noise), split from the run seed apart from the
// world's streams.
constexpr std::uint64_t kOracleStream = 0x0AC1E;
// Slower than this after the last lap, the car has come to rest.
constexpr double kRestSpeed = 0.1;  // m/s
// Window for projecting onto a closed reference around the previous projection.
constexpr double kProjectionWindow = 15.0;  // m

bool finite(const ControlCommand& c) {
    return std::isfinite(c.steer) && std::isfinite(c.accel);
}

// Whatever reaches the driver through the delay line.
using InFlight = std::variant<SensorMessage, ConeObservations>;

// Latency of each SensorMessage alternative, in ticks, indexed by variant index.
std::array<std::int64_t, std::variant_size_v<SensorMessage>> latency_ticks(const Latencies& l) {
    std::array<std::int64_t, std::variant_size_v<SensorMessage>> ticks{};
    ticks[SensorMessage(ImuSample{}).index()] = to_ticks(l.imu, "latency.imu");
    ticks[SensorMessage(WheelSpeeds{}).index()] = to_ticks(l.wheels, "latency.wheels");
    ticks[SensorMessage(SteeringSample{}).index()] = to_ticks(l.steering, "latency.steering");
    ticks[SensorMessage(LidarScan{}).index()] = to_ticks(l.lidar, "latency.lidar");
    ticks[SensorMessage(CameraFrame{}).index()] = to_ticks(l.camera, "latency.camera");
    return ticks;
}

// Localization and tracking errors, accumulated per mission state at every control step.
class PhaseAccumulator {
public:
    void add(stack::MissionState state, const StateEstimate& estimate, const VehicleState& truth,
             const Trajectory* reference) {
        if (!estimate.valid) return;
        Accumulators& a = by_state_[stack::index_of(state)];
        a.position.add((estimate.pose.position() - truth.pose.position()).norm());
        a.yaw.add(std::abs(angle_diff(estimate.pose.yaw, truth.pose.yaw)));
        if (reference == nullptr || reference->empty()) return;
        const Vec2 p = estimate.pose.position();
        Trajectory::Projection projection;
        if (reference->closed()) {
            projection = reference->project(p, hint_, kProjectionWindow);
            if (projection.distance > 0.5 * kProjectionWindow) projection = reference->project(p);
            hint_ = projection.s;
        } else {
            projection = reference->project(p);
        }
        a.lateral.add(projection.distance);
    }

    void fill(RunResult& result) const {
        for (std::size_t i = 0; i < by_state_.size(); ++i) {
            result.phases[i].position = by_state_[i].position.stats();
            result.phases[i].yaw = by_state_[i].yaw.stats();
            result.phases[i].lateral = by_state_[i].lateral.stats();
        }
    }

private:
    struct Accumulators {
        ErrorAccumulator position;
        ErrorAccumulator yaw;
        ErrorAccumulator lateral;
    };
    std::array<Accumulators, stack::kMissionStateCount> by_state_{};
    double hint_ = 0.0;
};

}  // namespace

RunResult run_closed_loop(const RunConfig& config, const Track& track, stack::Driver& driver,
                          RunObserver* observer) {
    validate(config);
    const Stopwatch wall;
    sim::World world(track, make_world_config(config));
    const Track& truth_track = world.track();
    const LidarGeometry& lidar = world.config().rig.lidar;
    const auto latency = latency_ticks(config.latency);
    const std::int64_t lidar_latency = to_ticks(config.latency.lidar, "latency.lidar");
    const std::int64_t control_ticks = to_ticks(config.control_period, "control_period");
    const int laps = config.stack.mission.laps;

    RunResult result;
    result.name = config.name;
    result.track_name = truth_track.name;
    result.track_seed = truth_track.seed;
    result.track_length = truth_track.centerline.length();
    result.seed = config.seed;
    result.laps_target = laps;

    DelayQueue<InFlight> in_flight;
    std::vector<InFlight> due;
    Rng oracle_rng = Rng(config.seed).split(kOracleStream);
    DnfMonitor dnf(config.dnf);
    PhaseAccumulator phases;
    std::size_t transitions_seen = 0;
    bool map_evaluated = false;
    bool laps_done = false;
    Time laps_done_time = 0.0;
    double sim_seconds = 0.0;
    std::int64_t ticks = 0;

    if (observer != nullptr) observer->on_start(truth_track);
    while (true) {
        const Stopwatch tick_watch;
        std::vector<SensorMessage> messages = world.step();
        sim_seconds += tick_watch.elapsed_seconds();
        ++ticks;
        const std::int64_t tick = world.tick();
        const Time now = world.time();
        const VehicleState& truth = world.truth();

        for (SensorMessage& message : messages) {
            const bool lidar_scan = std::holds_alternative<LidarScan>(message);
            if (config.oracles.perception &&
                (lidar_scan || std::holds_alternative<CameraFrame>(message))) {
                if (lidar_scan) {
                    in_flight.push(oracle_observations(truth_track.cones, truth, lidar,
                                                       config.perception_oracle, stamp_of(message),
                                                       oracle_rng),
                                   tick + lidar_latency);
                }
                continue;
            }
            const std::size_t kind = message.index();
            in_flight.push(std::move(message), tick + latency[kind]);
        }
        if (config.oracles.state) driver.on_oracle_state(oracle_state(truth, now));
        due.clear();
        in_flight.pop_due(tick, due);
        for (const InFlight& item : due) {
            if (const auto* sensor = std::get_if<SensorMessage>(&item)) {
                driver.on_sensor(*sensor);
            } else {
                driver.on_oracle_observations(std::get<ConeObservations>(item));
            }
        }

        if (tick % control_ticks == 0) {
            const ControlCommand command = driver.control(now);
            if (!finite(command)) {
                result.dnf_reason = "non-finite control command";
                break;
            }
            world.command(command);
            phases.add(driver.mission_state(), driver.estimate(), truth, driver.reference());
            if (observer != nullptr) {
                observer->on_control({now, truth, world.race(), driver, command});
            }
        }

        const stack::MissionState state = driver.mission_state();
        result.phases[stack::index_of(state)].duration += sim::World::kTickSeconds;
        const auto& transitions = driver.transitions();
        for (; transitions_seen < transitions.size(); ++transitions_seen) {
            if (transitions[transitions_seen].to == stack::MissionState::MapFinalized &&
                driver.map() != nullptr && !map_evaluated) {
                result.map = evaluate_map(*driver.map(), truth_track, now, true);
                map_evaluated = true;
            }
        }

        const sim::RaceMonitor& race = world.race();
        const double speed = std::hypot(truth.vx, truth.vy);
        if (!laps_done && race.laps_completed() >= laps) {
            laps_done = true;
            laps_done_time = now;
        }
        if (state == stack::MissionState::Failed) {
            result.dnf_reason = "mission failed: " +
                                (transitions.empty() ? std::string() : transitions.back().reason);
            break;
        }
        if (state == stack::MissionState::Finished) {
            if (!laps_done) {
                result.dnf_reason = std::format("the mission ended after {} of {} laps",
                                                race.laps_completed(), laps);
            }
            result.stopped = laps_done && speed < config.dnf.stall_speed;
            break;
        }
        if (laps_done) {
            if (speed < kRestSpeed) {
                result.stopped = true;
                break;
            }
            if (now - laps_done_time > config.dnf.stop_timeout) break;
        }
        const double off_track = off_track_distance(truth_track, race.s(), race.lateral());
        result.max_lateral_true = std::max(result.max_lateral_true, std::abs(race.lateral()));
        if (const auto reason = dnf.update(now, speed, off_track)) {
            result.dnf_reason = *reason;
            break;
        }
    }

    const sim::RaceMonitor& race = world.race();
    for (const sim::LapRecord& lap : race.laps()) {
        if (static_cast<int>(result.lap_times.size()) < laps)
            result.lap_times.push_back(lap.time());
    }
    result.cones_hit = race.cone_hits().size();
    result.off_course = race.off_course().size();
    result.penalty_seconds = race.penalty_seconds();
    result.completed = laps_done && result.dnf_reason.empty();
    result.final_state = driver.mission_state();
    result.transitions = driver.transitions();
    result.sim_time = world.time();
    if (!map_evaluated && driver.map() != nullptr) {
        result.map = evaluate_map(*driver.map(), truth_track, world.time(), false);
    }
    result.association = driver.association_counts();
    phases.fill(result);
    result.wall_time = wall.elapsed_seconds();
    result.compute = driver.compute_summary();
    result.compute["sim_tick"] = {
        {"count", ticks},
        {"mean_us", ticks > 0 ? 1e6 * sim_seconds / static_cast<double>(ticks) : 0.0}};
    result.compute["realtime_factor"] =
        result.wall_time > 0.0 ? result.sim_time / result.wall_time : 0.0;
    return result;
}

}  // namespace apex::runner
