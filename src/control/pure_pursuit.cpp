#include "pure_pursuit.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "apex/core/geometry.hpp"

namespace apex::control {

PurePursuitController::PurePursuitController(const PurePursuitConfig& config,
                                             const VehicleParams& nominal)
    : PathFollower(config.speed, config.delay_compensation, nominal), config_(config) {
    if (!(config.lookahead_min > 0.0) || !(config.lookahead_time >= 0.0) ||
        !(config.lookahead_max >= config.lookahead_min)) {
        throw std::invalid_argument(
            "PurePursuitConfig: need lookahead_min > 0, lookahead_time >= 0 and "
            "lookahead_max >= lookahead_min");
    }
}

double PurePursuitController::steer(const Pose2& pose, const StateEstimate& state,
                                    const Trajectory& reference, double travel) {
    const VehicleParams& p = model().params();
    const double speed = std::max(state.vx, 0.0);
    const Vec2 rear = pose.position() - p.lr * heading_vector(pose.yaw);
    const PathCoordinates at = rear_tracker_.locate(reference, rear, travel);
    const double lookahead = std::clamp(config_.lookahead_min + config_.lookahead_time * speed,
                                        config_.lookahead_min, config_.lookahead_max);
    const Vec2 goal = sample_reference(reference, at.s + lookahead).position;
    const Vec2 local = Pose2{rear.x(), rear.y(), pose.yaw}.to_local(goal);
    const double distance_squared = std::max(local.squaredNorm(), 1e-6);
    const double curvature = 2.0 * local.y() / distance_squared;
    double delta = std::atan(p.wheelbase() * curvature);
    if (config_.understeer_compensation) {
        const SlipAngles slip = model().steady_state_slip(curvature, speed);
        delta += slip.rear - slip.front;
    }
    return delta;
}

}  // namespace apex::control
