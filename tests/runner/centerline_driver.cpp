#include "centerline_driver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <variant>

namespace apex::runner::test_drivers {

namespace {

using stack::MissionState;

constexpr double kGravity = 9.81;
constexpr double kSpeedGain = 1.5;          // 1/s
constexpr double kStopDecel = 4.0;          // m/s^2
constexpr double kHoldBrake = 2.0;          // m/s^2
constexpr double kRestSpeed = 0.1;          // m/s
constexpr double kProjectionWindow = 10.0;  // m

std::string kind_of(const SensorMessage& message) {
    struct Visitor {
        std::string operator()(const ImuSample&) const { return "imu"; }
        std::string operator()(const WheelSpeeds&) const { return "wheels"; }
        std::string operator()(const SteeringSample&) const { return "steering"; }
        std::string operator()(const LidarScan&) const { return "lidar"; }
        std::string operator()(const CameraFrame&) const { return "camera"; }
    };
    return std::visit(Visitor{}, message);
}

std::size_t size_of(const SensorMessage& message) {
    if (const auto* scan = std::get_if<LidarScan>(&message)) return scan->points.size();
    if (const auto* frame = std::get_if<CameraFrame>(&message)) return frame->detections.size();
    return 0;
}

}  // namespace

CenterlineDriver::CenterlineDriver(const Track& track, const VehicleParams& vehicle,
                                   const CenterlineDriverOptions& options)
    : track_(track), vehicle_(vehicle), options_(options) {
    for (const Cone& cone : track.cones) {
        MapCone landmark;
        landmark.id = cone.id;
        landmark.position = cone.position;
        landmark.color = cone.color;
        landmark.truth_id = cone.id;
        landmark.observations = 1;
        map_.cones.push_back(landmark);
    }
    map_.revision = 1;
}

void CenterlineDriver::on_sensor(const SensorMessage& message) {
    deliveries_.push_back({kind_of(message), stamp_of(message), truth_.stamp, size_of(message)});
}

void CenterlineDriver::on_oracle_observations(const ConeObservations& observations) {
    deliveries_.push_back(
        {"oracle_observations", observations.stamp, truth_.stamp, observations.cones.size()});
}

void CenterlineDriver::on_oracle_state(const StateEstimate& truth) {
    truth_ = truth;
    count_laps();
}

void CenterlineDriver::count_laps() {
    const Trajectory& line = track_.centerline;
    const Vec2 p = truth_.pose.position();
    s_ = previous_s_ ? line.project(p, *previous_s_, kProjectionWindow).s : line.project(p).s;
    // A forward wrap of the centerline's arc length is a crossing of the timing line (s = 0).
    if (previous_s_ && *previous_s_ > 0.75 * line.length() && s_ < 0.25 * line.length()) {
        ++crossings_;
    }
    previous_s_ = s_;
}

void CenterlineDriver::transition(MissionState to, Time now, std::string reason) {
    transitions_.push_back({now, state_, to, std::move(reason)});
    state_ = to;
    state_since_ = now;
}

const Trajectory* CenterlineDriver::reference() const {
    return state_ == MissionState::Staged || stack::is_terminal(state_) ? nullptr
                                                                        : &track_.centerline;
}

ControlCommand CenterlineDriver::control(Time now) {
    const double speed = std::hypot(truth_.vx, truth_.vy);
    const bool faulty = options_.fault != Fault::None && now >= options_.fault_time;
    const bool stops = options_.fault != Fault::NeverStop;
    if (faulty && !stack::is_terminal(state_)) {
        if (options_.fault == Fault::Fail) transition(MissionState::Failed, now, "test fault");
        if (options_.fault == Fault::FinishEarly) {
            transition(MissionState::Finished, now, "test fault");
        }
    }
    switch (state_) {
        case MissionState::Staged:
            if (now >= options_.start_time && options_.fault != Fault::Stall) {
                transition(MissionState::Exploring, now, "go");
            }
            break;
        case MissionState::Exploring:
            if (laps_completed() >= options_.laps) {
                if (stops) transition(MissionState::Stopping, now, "laps done");
            } else if (laps_completed() >= 1) {
                transition(MissionState::MapFinalized, now, "lap 1 closed");
            }
            break;
        case MissionState::MapFinalized:
            if (now - state_since_ >= options_.raceline_delay) {
                transition(MissionState::Racing, now, "raceline ready");
            }
            break;
        case MissionState::Racing:
            if (laps_completed() >= options_.laps && stops) {
                transition(MissionState::Stopping, now, "laps done");
            }
            break;
        case MissionState::Stopping:
            if (speed < kRestSpeed) transition(MissionState::Finished, now, "at rest");
            break;
        case MissionState::Finished:
        case MissionState::Failed:
            break;
    }

    ControlCommand command{now, 0.0, -kHoldBrake};
    if (state_ == MissionState::Failed) command.accel = -9.0;
    if (state_ == MissionState::Staged || stack::is_terminal(state_)) return command;

    // Pure pursuit from the CoG to the centerline point one lookahead ahead (R. C. Coulter,
    // "Implementation of the Pure Pursuit Path Tracking Algorithm", CMU-RI-TR-92-01, 1992).
    const Vec2 target =
        truth_.pose.to_local(track_.centerline.at(s_ + options_.lookahead).position);
    const double alpha = std::atan2(target.y(), target.x());
    command.steer =
        std::atan(2.0 * vehicle_.wheelbase() * std::sin(alpha) / std::max(target.norm(), 1.0));
    command.steer = std::clamp(command.steer, -vehicle_.max_steer, vehicle_.max_steer);
    if (state_ == MissionState::Stopping) {
        command.accel = -kStopDecel;
    } else {
        const double resistance =
            (vehicle_.drag(speed) + vehicle_.rolling_resistance * vehicle_.mass * kGravity) /
            vehicle_.mass;
        command.accel = kSpeedGain * (options_.speed - speed) + resistance;
    }
    if (faulty && options_.fault == Fault::Veer) command.steer = vehicle_.max_steer;
    if (faulty && options_.fault == Fault::NonFinite) {
        command.steer = std::numeric_limits<double>::quiet_NaN();
    }
    return command;
}

}  // namespace apex::runner::test_drivers
