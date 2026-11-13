#include "stanley.hpp"

#include <algorithm>
#include <stdexcept>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

namespace apex::control {

StanleyController::StanleyController(const StanleyConfig& config, const VehicleParams& nominal)
    : PathFollower(config.speed, config.delay_compensation, nominal), config_(config) {
    if (!(config.gain > 0.0) || !(config.softening_speed > 0.0) || !(config.yaw_damping >= 0.0)) {
        throw std::invalid_argument(
            "StanleyConfig: need gain > 0, softening_speed > 0 and yaw_damping >= 0");
    }
}

double StanleyController::steer(const Pose2& pose, const StateEstimate& state,
                                const Trajectory& reference, double travel) {
    const VehicleParams& p = model().params();
    const double speed = std::max(state.vx, 0.0);
    const Vec2 front = pose.position() + p.lf * heading_vector(pose.yaw);
    const PathCoordinates at = front_tracker_.locate(reference, front, travel);
    const TrajectoryPoint path = sample_reference(reference, at.s);
    double delta = stanley_law(angle_diff(path.yaw, pose.yaw), at.lateral, speed, config_.gain,
                               config_.softening_speed);
    if (config_.slip_feedforward) {
        delta -= model().steady_state_slip(path.curvature, speed).front;
    }
    delta += config_.yaw_damping * (speed * path.curvature - state.yaw_rate);
    return delta;
}

}  // namespace apex::control
