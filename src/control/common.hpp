#pragma once

#include <optional>

#include "apex/control/vehicle_model.hpp"
#include "apex/core/messages.hpp"
#include "apex/core/trajectory.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

// Pieces every controller of the module shares: input checks, the time step between calls,
// pose prediction through actuator delays, and the final guard on every command.
namespace apex::control {

// The stack calls control() every 20 ms (docs/design.md, section 4.4). The first call after a
// reset, or one after a gap, assumes this period.
inline constexpr double kNominalPeriod = 0.02;  // s
// Deceleration of the safe command used for unusable inputs (m/s^2).
inline constexpr double kSafeDecel = 4.0;

// A valid estimate with finite fields, and a non-empty reference.
[[nodiscard]] bool usable_inputs(const StateEstimate& state, const Trajectory& reference);

// The time since the previous call, or the nominal period on the first call and after a gap
// or a step backwards in time.
class CallClock {
public:
    [[nodiscard]] double step(Time now);
    void reset() { last_.reset(); }

private:
    std::optional<Time> last_;
};

// The pose after `horizon` seconds at the estimate's constant body velocities: an arc of
// curvature yaw_rate / speed, which is exact for constant (vx, vy, r).
[[nodiscard]] Pose2 predict_pose(const StateEstimate& state, double horizon);

// Stanley's steering law on front-axle errors: heading error plus atan(k e / (v_soft + v)),
// steering against a positive (leftward) lateral offset.
[[nodiscard]] double stanley_law(double heading_error, double lateral, double speed, double gain,
                                 double softening_speed);

// The command actually sent: steering within the angle limit and, after the first command,
// within the actuator's rate limit from the previous one; acceleration within the drive and
// brake limits at `speed`; anything non-finite replaced by the previous steering and braking.
[[nodiscard]] ControlCommand limit_command(double steer, double accel,
                                           const std::optional<ControlCommand>& previous, double dt,
                                           double speed, const NominalModel& model, Time now);

}  // namespace apex::control
