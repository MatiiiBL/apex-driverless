#pragma once

#include "apex/control/controller.hpp"
#include "apex/control/reference.hpp"

#include "path_follower.hpp"

namespace apex::control {

// Pure pursuit from the rear axle: the arc through the rear axle, tangent to its velocity,
// that reaches the reference point one lookahead distance ahead (measured along the path from
// the rear axle's projection) has curvature kappa = 2 y / d^2, with (x, y) the goal in the rear
// axle's frame and d its distance; the kinematic steering for it is atan(L kappa). Lookahead
// grows with speed, trading tracking precision for stability. Near the grip limit the tires
// slip, which the kinematic law leaves as a lateral error; with understeer compensation the
// nominal model's steady-state slip angles for the path's curvature correct it twice: the
// rear one turns the frame from the heading to the rear axle's velocity, and the difference
// alpha_r - alpha_f is added to the steering.
class PurePursuitController final : public PathFollower {
public:
    PurePursuitController(const PurePursuitConfig& config, const VehicleParams& nominal);
    [[nodiscard]] std::string_view name() const override { return "pure_pursuit"; }

private:
    [[nodiscard]] double steer(const Pose2& pose, const StateEstimate& state,
                               const Trajectory& reference, double travel) override;
    void reset_steering() override { rear_tracker_.reset(); }

    PurePursuitConfig config_;
    ReferenceTracker rear_tracker_;
};

}  // namespace apex::control
