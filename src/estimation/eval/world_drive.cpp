#include "apex/estimation/eval/world_drive.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <stdexcept>
#include <utility>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"
#include "apex/perception/perception.hpp"

namespace apex::estimation::eval {

namespace {

constexpr std::int64_t kControlTicks = 20;  // 50 Hz, the stack's control rate
constexpr double kTimeLimit = 600.0;        // s

// The speed the car may drive at s: the profile's top speed and lateral acceleration limit
// everywhere ahead within braking distance, so it is never too fast for the next corner.
double reference_speed(const Trajectory& center, double s, double top_speed, double lateral,
                       double braking) {
    double v = top_speed;
    const double preview = top_speed * top_speed / (2.0 * braking);
    for (double d = 0.0; d <= preview; d += 1.0) {
        const double kappa = std::abs(center.at(center.wrap_s(s + d)).curvature);
        const double limit =
            kappa > 1e-9 ? std::min(top_speed, std::sqrt(lateral / kappa)) : top_speed;
        v = std::min(v, std::sqrt(limit * limit + 2.0 * braking * d));
    }
    return v;
}

}  // namespace

SyntheticRun make_world_run(const Track& track, const sim::WorldConfig& config,
                            const DriveProfile& profile, const SyntheticSensors& perception,
                            ConeSource source) {
    if (profile.laps < 1 || !(profile.exploration_speed > 0.0) || !(profile.racing_speed > 0.0) ||
        !(profile.max_accel > 0.0) || !(profile.max_brake > 0.0) ||
        !(profile.exploration_lateral_accel > 0.0) || !(profile.racing_lateral_accel > 0.0) ||
        profile.hold_at_start < 0.0 || !(perception.latency >= 0.0)) {
        throw std::invalid_argument("make_world_run: invalid drive profile");
    }
    sim::World world(track, config);
    const Trajectory& center = world.track().centerline;
    const VehicleParams& car = config.vehicle;
    Rng cone_rng = Rng(config.seed).split(0x0b5e7);
    apex::perception::ConePerception detector({}, config.rig);
    double last_gyro = 0.0;

    SyntheticRun run;
    run.mappable_since.assign(track.cones.size(), std::numeric_limits<Time>::infinity());
    std::vector<std::uint32_t> detections(track.cones.size(), 0);
    std::deque<std::pair<Time, ConeObservations>> in_flight;
    auto record_truth = [&] {
        const VehicleState& state = world.truth();
        TruthSample t;
        t.stamp = world.time();
        t.pose = state.pose;
        t.vx = state.vx;
        t.vy = state.vy;
        t.yaw_rate = state.yaw_rate;
        t.ax = state.ax;
        t.ay = state.ay;
        t.steer = state.steer;
        t.gyro_bias = std::numeric_limits<double>::quiet_NaN();
        t.distance = world.race().progress();
        t.lap = std::min(world.race().laps_completed() + 1, profile.laps);
        run.truth.push_back(t);
    };
    record_truth();

    double s_hint = center.project(track.start_pose.position()).s;
    while (world.race().laps_completed() < profile.laps) {
        if (world.time() > kTimeLimit) {
            throw std::runtime_error("make_world_run: the car did not finish");
        }
        if (world.tick() % kControlTicks == 0) {
            const VehicleState& state = world.truth();
            const Trajectory::Projection p = center.project(state.pose.position(), s_hint, 10.0);
            s_hint = p.s;
            const double speed = std::max(state.vx, 0.0);
            // Pure pursuit: the arc through a point one lookahead ahead on the centerline.
            const double lookahead = 2.5 + 0.25 * speed;
            const Vec2 target =
                state.pose.to_local(center.at(center.wrap_s(p.s + lookahead)).position);
            const double alpha = std::atan2(target.y(), target.x());
            ControlCommand command;
            command.stamp = world.time();
            command.steer = std::atan(2.0 * car.wheelbase() * std::sin(alpha) / lookahead);
            if (world.time() >= profile.hold_at_start) {
                const bool exploring = world.race().laps_completed() == 0;
                const double lateral =
                    exploring ? profile.exploration_lateral_accel : profile.racing_lateral_accel;
                const double v_ref = reference_speed(
                    center, p.s, exploring ? profile.exploration_speed : profile.racing_speed,
                    lateral, profile.max_brake);
                const double resistance =
                    (car.drag(speed) + car.rolling_resistance * car.mass * kGravity) / car.mass;
                // The friction circle: only the grip cornering leaves may drive, or the rear axle
                // saturates out of a hairpin and the car spins. Braking is planned to end before
                // the corner, so it keeps its full authority to make the plan.
                const double used = std::min(std::abs(state.ay) / lateral, 1.0);
                const double share = std::max(0.25, std::sqrt(1.0 - used * used));
                command.accel = std::clamp(3.0 * (v_ref - speed) + resistance, -profile.max_brake,
                                           share * profile.max_accel);
            }
            world.command(command);
        }
        const std::vector<SensorMessage> messages = world.step();
        const Time now = world.time();
        for (const SensorMessage& m : messages) {
            if (const auto* imu = std::get_if<ImuSample>(&m)) {
                run.messages.push_back({now, *imu});
                last_gyro = imu->yaw_rate;
            } else if (const auto* wheels = std::get_if<WheelSpeeds>(&m)) {
                run.messages.push_back({now, *wheels});
            } else if (const auto* steering = std::get_if<SteeringSample>(&m)) {
                run.messages.push_back({now, *steering});
            } else if (const auto* frame = std::get_if<CameraFrame>(&m)) {
                if (source == ConeSource::Perception) detector.on_camera(*frame);
            } else if (const auto* scan = std::get_if<LidarScan>(&m)) {
                ConeObservations cones = source == ConeSource::Perception
                                             ? detector.on_lidar(*scan, last_gyro)
                                             : observe_cones(track, world.truth().pose, scan->stamp,
                                                             perception, cone_rng);
                note_detections(cones, detections, run.mappable_since);
                in_flight.emplace_back(scan->stamp + perception.latency, std::move(cones));
            }
        }
        // Scans due this tick arrive after the tick's other messages, as a World delivers them.
        while (!in_flight.empty() &&
               in_flight.front().first <= now + 0.5 * sim::World::kTickSeconds) {
            run.messages.push_back({now, std::move(in_flight.front().second)});
            in_flight.pop_front();
        }
        record_truth();
        if (world.race().laps_completed() == 1 && !std::isfinite(run.lap1_end)) {
            run.lap1_end = now;
        }
    }
    return run;
}

}  // namespace apex::estimation::eval
