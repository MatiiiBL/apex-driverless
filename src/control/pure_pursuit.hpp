#pragma once

#include "apex/control/controller.hpp"
#include "apex/control/reference.hpp"

#include "path_follower.hpp"

namespace apex::control {

// Pure pursuit from the rear axle: the arc through the rear axle, tangent to the car's
// heading, that reaches the reference point one lookahead distance ahead (measured along the
// path from the rear axle's projection) has curvature kappa = 2 y / d^2, with (x, y) the goal in
// the rear-axle frame and d its distance; the kinematic steering for it is atan(L kappa).
// Lookahead grows with speed, trading tracking precision for stability. With understeer
// compensation the nominal model's steady-state slip-angle difference for that curvature is
// added, which the kinematic law leaves as a growing lateral error near the grip limit.
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
