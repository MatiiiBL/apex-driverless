#include "path_follower.hpp"

#include <algorithm>

#include "apex/core/timing.hpp"

namespace apex::control {

PathFollower::PathFollower(const SpeedLoopConfig& speed, bool delay_compensation,
                           const VehicleParams& nominal)
    : model_(nominal),
      speed_config_(speed),
      speed_(speed, model_),
      delay_compensation_(delay_compensation) {}

ControlCommand PathFollower::compute(const StateEstimate& state, const Trajectory& reference,
                                     Time now) {
    const Stopwatch watch;
    ++stats_.calls;
    const double dt = clock_.step(now);
    ControlCommand command;
    if (!usable_inputs(state, reference)) {
        ++stats_.invalid_inputs;
        command =
            limit_command(last_ ? last_->steer : 0.0, -kSafeDecel, last_, dt, 0.0, model_, now);
    } else {
        const double speed = std::max(state.vx, 0.0);
        const double travel = speed * dt + 1.0;
        Pose2 pose = state.pose;
        if (delay_compensation_) {
            // Commands act after the dead time; the first-order lag adds about its time
            // constant of effective delay. The estimate may also be older than `now`.
            const VehicleParams& p = model_.params();
            const double horizon =
                std::max(now - state.stamp, 0.0) + p.steer_dead_time + p.steer_time_constant;
            pose = predict_pose(state, std::min(horizon, 0.2));
        }
        const double steering = steer(pose, state, reference, travel);
        const PathCoordinates here = cog_tracker_.locate(reference, state.pose.position(), travel);
        const TrajectoryPoint target =
            sample_reference(reference, here.s + speed * speed_config_.preview_time);
        const double accel = speed_.update(target.speed, target.accel, speed, dt);
        command = limit_command(steering, accel, last_, dt, speed, model_, now);
    }
    last_ = command;
    stats_.compute_time.add(watch.elapsed_seconds());
    return command;
}

void PathFollower::reset() {
    speed_.reset();
    cog_tracker_.reset();
    clock_.reset();
    last_.reset();
    reset_steering();
}

}  // namespace apex::control
