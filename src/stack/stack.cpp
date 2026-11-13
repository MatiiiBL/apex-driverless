#include "apex/stack/stack.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <exception>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <utility>
#include <variant>

#include <nlohmann/json.hpp>

#include "apex/core/log.hpp"
#include "apex/core/timing.hpp"
#include "apex/stack/braking.hpp"

namespace apex::stack {

namespace {

// How much of the oracle's true-state stream to keep for looking up the pose at an
// observation's stamp: comfortably more than the LiDAR latency.
constexpr double kTruthHistorySeconds = 0.5;

bool finite(const StateEstimate& s) {
    return std::isfinite(s.pose.x) && std::isfinite(s.pose.y) && std::isfinite(s.pose.yaw) &&
           std::isfinite(s.vx) && std::isfinite(s.vy) && std::isfinite(s.yaw_rate) &&
           s.pose_covariance.allFinite();
}

bool finite(const ControlCommand& c) {
    return std::isfinite(c.steer) && std::isfinite(c.accel);
}

// Standard deviation along the major axis of the position uncertainty: the square root of the
// larger eigenvalue of the 2x2 position block, (a + c) / 2 + sqrt(((a - c) / 2)^2 + b^2).
double position_std(const StateEstimate& s) {
    const double a = s.pose_covariance(0, 0);
    const double b = s.pose_covariance(0, 1);
    const double c = s.pose_covariance(1, 1);
    const double half_diff = 0.5 * (a - c);
    return std::sqrt(std::max(0.5 * (a + c) + std::sqrt(half_diff * half_diff + b * b), 0.0));
}

// (landmark id, truth id) of every landmark, sorted by id.
std::vector<std::pair<std::int32_t, std::int32_t>> truth_by_id(const ConeMap& map) {
    std::vector<std::pair<std::int32_t, std::int32_t>> result;
    result.reserve(map.cones.size());
    for (const MapCone& cone : map.cones) result.emplace_back(cone.id, cone.truth_id);
    std::sort(result.begin(), result.end());
    return result;
}

// The truth ids that have a landmark, sorted.
std::vector<std::int32_t> mapped_truth_ids(const ConeMap& map) {
    std::vector<std::int32_t> result;
    result.reserve(map.cones.size());
    for (const MapCone& cone : map.cones) {
        if (cone.truth_id >= 0) result.push_back(cone.truth_id);
    }
    std::sort(result.begin(), result.end());
    return result;
}

const StackConfig& validated(const StackConfig& c) {
    if (!(c.hold_brake >= 0.0) || !(c.emergency_brake > 0.0) || !(c.driven_path_spacing > 0.0)) {
        throw std::invalid_argument("stack: decelerations and spacings must be positive");
    }
    return c;
}

}  // namespace

struct Stack::Impl {
    Impl(const StackConfig& c, const VehicleParams& nominal_params, const SensorRig& rig,
         const Pose2& start_pose, const Oracles& oracle_switches)
        : config(validated(c)),
          nominal(nominal_params),
          oracles(oracle_switches),
          perception(c.perception, rig),
          estimator(c.estimation, nominal_params, rig, start_pose),
          explorer(c.exploration, nominal_params),
          controller(control::make_controller(c.controller, nominal_params)),
          known_pose_mapper(c.known_pose_mapper),
          mission(c.mission),
          lap_counter(start_pose, c.lap_counter) {}

    [[nodiscard]] StateEstimate current_state() const {
        if (!oracles.state) return estimator.estimate();
        return truth_history.empty() ? StateEstimate{} : truth_history.back();
    }

    [[nodiscard]] const ConeMap& current_map() const {
        return oracles.state ? known_pose_mapper.map() : estimator.map();
    }

    [[nodiscard]] bool follows_exploration() const {
        const MissionState state = mission.state();
        return state == MissionState::Staged || state == MissionState::Exploring ||
               state == MissionState::MapFinalized;
    }

    [[nodiscard]] const Trajectory* active_reference() const {
        switch (mission.state()) {
            case MissionState::Staged:
            case MissionState::Exploring:
            case MissionState::MapFinalized:
                return exploration_path ? &*exploration_path : nullptr;
            case MissionState::Racing:
                return raceline ? &raceline->path : nullptr;
            case MissionState::Stopping:
            case MissionState::Finished:
                return braking ? &*braking : nullptr;
            case MissionState::Failed:
                return nullptr;
        }
        return nullptr;
    }

    // The oracle's true pose closest to `stamp`.
    [[nodiscard]] std::optional<Pose2> true_pose_at(Time stamp) const {
        if (truth_history.empty()) return std::nullopt;
        const auto it =
            std::lower_bound(truth_history.begin(), truth_history.end(), stamp,
                             [](const StateEstimate& s, Time t) { return s.stamp < t; });
        if (it == truth_history.end()) return truth_history.back().pose;
        if (it == truth_history.begin()) return it->pose;
        const auto before = std::prev(it);
        return (stamp - before->stamp < it->stamp - stamp) ? before->pose : it->pose;
    }

    void process_observations(const ConeObservations& observations, const Stopwatch& watch) {
        const ConeMap& map_before = current_map();
        const std::vector<std::int32_t> mapped_before = mapped_truth_ids(map_before);
        if (oracles.state) {
            const std::optional<Pose2> pose = true_pose_at(observations.stamp);
            if (!pose) return;
            known_pose_mapper.add(observations, *pose);
            count_associations(observations, known_pose_mapper.last_associations(), mapped_before);
        } else {
            estimator.on_cones(observations);
            count_associations(observations, estimator.last_associations(), mapped_before);
        }
        if (mapping) lap_counter.update_line(current_map());
        if (follows_exploration()) plan();
        scan_time.add(watch.elapsed_seconds());
    }

    // Evaluation only: compares truth ids, never feeds anything back.
    void count_associations(const ConeObservations& observations,
                            const std::vector<std::int32_t>& matches,
                            const std::vector<std::int32_t>& mapped_before) {
        if (matches.size() != observations.cones.size()) return;
        const auto truth = truth_by_id(current_map());
        for (std::size_t i = 0; i < matches.size(); ++i) {
            const std::int32_t truth_id = observations.cones[i].truth_id;
            if (truth_id >= 0) {
                ++associations.observations;
                if (std::binary_search(mapped_before.begin(), mapped_before.end(), truth_id)) {
                    ++associations.matchable;
                }
            }
            if (matches[i] < 0) continue;
            ++associations.matched;
            const auto landmark =
                std::lower_bound(truth.begin(), truth.end(), std::make_pair(matches[i], -1));
            if (truth_id >= 0 && landmark != truth.end() && landmark->first == matches[i] &&
                landmark->second == truth_id) {
                ++associations.correct;
            }
        }
    }

    void plan() {
        const StateEstimate state = current_state();
        if (!state.valid || !finite(state)) return;
        std::optional<Trajectory> path = explorer.plan(current_map(), state);
        if (!path) return;
        exploration_path = std::move(path);
        last_path_time = state.stamp;
    }

    void record(const Pose2& pose) {
        if (driven_path.empty() || (pose.position() - driven_path.back().position()).norm() >=
                                       config.driven_path_spacing) {
            driven_path.push_back(pose);
        }
    }

    void on_crossing(Time now, const Pose2& pose) {
        if (lap_counter.crossings() == 1) {
            recording = true;
            driven_path.clear();
            record(pose);
            return;
        }
        // A mission whose last lap was this one stops instead; it needs no raceline.
        if (mission.state() == MissionState::Exploring &&
            lap_counter.laps_completed() < config.mission.laps) {
            finalize(now, pose);
        }
    }

    // Lap 1 is closed: freeze the map, build the track model and the raceline. When the map
    // cannot be made into a track, the car explores another lap and tries again.
    void finalize(Time now, const Pose2& pose) {
        std::optional<planning::TrackModel> model =
            planning::build_track_model(current_map(), driven_path, config.track_model);
        std::optional<planning::Raceline> line;
        if (model) {
            try {
                line = planning::optimize_raceline(*model, nominal, config.raceline);
            } catch (const std::exception& e) {
                log(LogLevel::Warn, "t = {:.2f} s: raceline optimization failed: {}", now,
                    e.what());
            }
        }
        if (!model || !line || !line->path.closed() || line->path.size() < 3) {
            log(LogLevel::Warn, "t = {:.2f} s: lap {} did not give a track; exploring another lap",
                now, lap_counter.laps_completed());
            driven_path.clear();
            record(pose);
            return;
        }
        track_model = std::move(model);
        raceline = std::move(line);
        mapping = false;
        recording = false;
        if (oracles.state) {
            known_pose_mapper.set_mapping(false);
        } else {
            estimator.set_mapping(false);
        }
        mission.map_finalized(now);
    }

    void on_enter(MissionState state, const StateEstimate& estimate) {
        if (state == MissionState::Racing) {
            controller->reset();
        } else if (state == MissionState::Stopping) {
            const Trajectory* path =
                raceline ? &raceline->path : (exploration_path ? &*exploration_path : nullptr);
            if (path) {
                const double s = path->project(estimate.pose.position()).s;
                const double speed = std::hypot(estimate.vx, estimate.vy);
                braking = braking_reference(*path, s, speed, nominal, config.stop_profile);
            }
        }
    }

    ControlCommand command(const StateEstimate& state, bool usable, Time now) {
        ControlCommand command{now, last_command.steer, -config.hold_brake};
        switch (mission.state()) {
            case MissionState::Staged:
            case MissionState::Finished:
                command.steer = 0.0;
                break;
            case MissionState::Failed:
                command.accel = -config.emergency_brake;
                break;
            case MissionState::Exploring:
            case MissionState::MapFinalized:
            case MissionState::Racing:
            case MissionState::Stopping: {
                const Trajectory* reference = active_reference();
                if (!reference || !usable) break;
                const ControlCommand computed = controller->compute(state, *reference, now);
                if (finite(computed)) {
                    command = computed;
                } else {
                    mission.fail(now, "non-finite control command");
                    command.accel = -config.emergency_brake;
                }
                // Once the last lap is done the car only brakes: a controller that lost the car
                // (a spin) must not drive it anywhere to recover the reference.
                if (mission.state() == MissionState::Stopping) {
                    command.accel = std::min(command.accel, 0.0);
                }
                break;
            }
        }
        last_command = command;
        return command;
    }

    StackConfig config;
    VehicleParams nominal;
    Oracles oracles;
    perception::ConePerception perception;
    estimation::StateEstimator estimator;
    planning::ExplorationPlanner explorer;
    std::unique_ptr<control::Controller> controller;
    KnownPoseMapper known_pose_mapper;
    Mission mission;
    LapCounter lap_counter;

    std::deque<StateEstimate> truth_history;  // perfect-state oracle only
    bool mapping = true;
    std::optional<Trajectory> exploration_path;
    Time last_path_time = -1.0;
    std::optional<planning::TrackModel> track_model;
    std::optional<planning::Raceline> raceline;
    std::optional<Trajectory> braking;
    std::vector<Pose2> driven_path;
    bool recording = false;
    ControlCommand last_command;

    AssociationCounts associations;
    LatencyStats scan_time;     // a LiDAR scan through perception, estimation and planning
    LatencyStats control_time;  // one control() call
};

Stack::Stack(const StackConfig& config, const VehicleParams& nominal, const SensorRig& rig,
             const Pose2& start, const Oracles& oracles)
    : impl_(std::make_unique<Impl>(config, nominal, rig, start, oracles)) {}

Stack::~Stack() = default;

void Stack::on_sensor(const SensorMessage& message) {
    Impl& s = *impl_;
    if (const auto* imu = std::get_if<ImuSample>(&message)) {
        if (!s.oracles.state) s.estimator.on_imu(*imu);
    } else if (const auto* wheels = std::get_if<WheelSpeeds>(&message)) {
        if (!s.oracles.state) s.estimator.on_wheels(*wheels);
    } else if (const auto* steering = std::get_if<SteeringSample>(&message)) {
        if (!s.oracles.state) s.estimator.on_steering(*steering);
    } else if (const auto* camera = std::get_if<CameraFrame>(&message)) {
        if (!s.oracles.perception) s.perception.on_camera(*camera);
    } else if (const auto* scan = std::get_if<LidarScan>(&message)) {
        if (s.oracles.perception) return;
        const Stopwatch watch;
        const ConeObservations observations =
            s.perception.on_lidar(*scan, s.current_state().yaw_rate);
        s.process_observations(observations, watch);
    }
}

void Stack::on_oracle_observations(const ConeObservations& observations) {
    if (!impl_->oracles.perception) return;
    impl_->process_observations(observations, Stopwatch{});
}

void Stack::on_oracle_state(const StateEstimate& truth) {
    Impl& s = *impl_;
    if (!s.oracles.state) return;
    s.truth_history.push_back(truth);
    while (s.truth_history.front().stamp < truth.stamp - kTruthHistorySeconds) {
        s.truth_history.pop_front();
    }
}

ControlCommand Stack::control(Time now) {
    const Stopwatch watch;
    Impl& s = *impl_;
    const StateEstimate state = s.current_state();
    const bool is_finite = finite(state);
    const bool usable =
        is_finite && state.valid && position_std(state) <= s.config.mission.max_position_std;
    const MissionState before = s.mission.state();
    const bool driving = before != MissionState::Staged && !is_terminal(before);
    if (usable && driving) {
        if (s.lap_counter.update(state.pose.position())) s.on_crossing(now, state.pose);
        if (s.recording) s.record(state.pose);
    }

    MissionInputs inputs;
    inputs.now = now;
    inputs.estimate_finite = is_finite;
    inputs.estimate_usable = usable;
    inputs.speed = is_finite ? std::hypot(state.vx, state.vy) : 0.0;
    inputs.laps_completed = s.lap_counter.laps_completed();
    inputs.last_path_time = s.last_path_time;
    const MissionState middle = s.mission.state();  // finalize() may have moved it on
    const MissionState after = s.mission.update(inputs);
    if (after != middle) s.on_enter(after, state);

    const ControlCommand command = s.command(state, usable, now);
    s.control_time.add(watch.elapsed_seconds());
    return command;
}

MissionState Stack::mission_state() const {
    return impl_->mission.state();
}

const std::vector<MissionTransition>& Stack::transitions() const {
    return impl_->mission.transitions();
}

StateEstimate Stack::estimate() const {
    return impl_->current_state();
}

const ConeMap* Stack::map() const {
    return &impl_->current_map();
}

const Trajectory* Stack::reference() const {
    return impl_->active_reference();
}

AssociationCounts Stack::association_counts() const {
    return impl_->associations;
}

nlohmann::json Stack::compute_summary() const {
    const Impl& s = *impl_;
    nlohmann::json j = nlohmann::json::object();
    if (!s.oracles.perception) {
        j["perception"] = s.perception.stats().lidar_time.summary();
        j["perception"]["cones_reported"] = s.perception.stats().cones_reported;
    }
    if (!s.oracles.state) {
        const estimation::EstimatorStats& e = s.estimator.stats();
        j["estimation_predict"] = e.predict_time.summary();
        j["estimation_update"] = e.update_time.summary();
        j["estimation_update"]["observations"] = e.observations;
        j["estimation_update"]["associated"] = e.associated;
        j["estimation_update"]["new_landmarks"] = e.new_landmarks;
        j["estimation_update"]["rejected"] = e.rejected;
        j["estimation_update"]["tentative_matches"] = e.tentative_matches;
        j["estimation_update"]["unpaired_in_gate"] = e.unpaired_in_gate;
        j["estimation_update"]["ambiguous"] = e.ambiguous;
        j["estimation_update"]["too_far_to_map"] = e.too_far_to_map;
        j["estimation_update"]["too_close_to_map"] = e.too_close_to_map;
        j["estimation_update"]["tentatives_started"] = e.tentatives_started;
        j["estimation_update"]["tentatives_dropped"] = e.tentatives_dropped;
        j["estimation_update"]["late_dropped"] = e.late_dropped;
        j["estimation_update"]["invalid_inputs"] = e.invalid_inputs;
        j["estimation_update"]["failed_updates"] = e.failed_updates;
        j["estimation_update"]["recoveries"] = e.recoveries;
    }
    j["exploration"] = s.explorer.stats().plan_time.summary();
    j["exploration"]["failures"] = s.explorer.stats().failures;
    j["raceline"] = {{"optimize_s", s.raceline ? s.raceline->optimize_seconds : 0.0},
                     {"predicted_lap_s", s.raceline ? s.raceline->predicted_lap_time : 0.0}};
    j["controller"] = s.controller->stats().compute_time.summary();
    j["controller"]["solver_failures"] = s.controller->stats().solver_failures;
    j["stack_scan"] = s.scan_time.summary();
    j["stack_control"] = s.control_time.summary();
    return j;
}

int Stack::laps_completed() const {
    return impl_->lap_counter.laps_completed();
}

const TimingLine& Stack::timing_line() const {
    return impl_->lap_counter.line();
}

const std::optional<planning::TrackModel>& Stack::track_model() const {
    return impl_->track_model;
}

const std::optional<planning::Raceline>& Stack::raceline() const {
    return impl_->raceline;
}

std::span<const Pose2> Stack::driven_path() const {
    return impl_->driven_path;
}

}  // namespace apex::stack
