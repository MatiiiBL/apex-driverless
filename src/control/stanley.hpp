#pragma once

#include "apex/control/controller.hpp"
#include "apex/control/reference.hpp"

#include "path_follower.hpp"

namespace apex::control {

// Stanley (Hoffmann et al., ACC 2007) on the front axle's projection onto the reference:
//   delta = (psi_path - psi) + atan(-k e / (v_soft + v)) - alpha_f,ss + k_r (v kappa - r)
// The first two terms point the front wheels along the path and steer them back onto it, which
// for a kinematic car converges e exponentially at rate k; the softening speed keeps the gain
// bounded at standstill. The steady-state front slip alpha_f,ss of the nominal model (the
// paper's psi_ss term, for a car whose tires slip) lets the wheels point inward of the path by
// the slip they need in a curve, and the yaw-rate term damps the yaw mode that the heading term
// excites at speed. The paper's steering-servo damping term is not used: the estimate carries
// no measured steering angle; delay compensation covers the actuator instead.
class StanleyController final : public PathFollower {
public:
    StanleyController(const StanleyConfig& config, const VehicleParams& nominal);
    [[nodiscard]] std::string_view name() const override { return "stanley"; }

private:
    [[nodiscard]] double steer(const Pose2& pose, const StateEstimate& state,
                               const Trajectory& reference, double travel) override;
    void reset_steering() override { front_tracker_.reset(); }

    StanleyConfig config_;
    ReferenceTracker front_tracker_;
};

}  // namespace apex::control
