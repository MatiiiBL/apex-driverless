#pragma once

#include <optional>

#include "apex/control/controller.hpp"
#include "apex/control/reference.hpp"
#include "apex/control/speed_controller.hpp"
#include "apex/control/vehicle_model.hpp"

#include "common.hpp"

// The machinery the two geometric controllers share: input checks, prediction of the pose
// through the steering actuator's delay, the speed loop, the command guard and statistics. A
// subclass supplies only its steering law.
namespace apex::control {

class PathFollower : public Controller {
public:
    [[nodiscard]] ControlCommand compute(const StateEstimate& state, const Trajectory& reference,
                                         Time now) final;
    void reset() final;
    [[nodiscard]] const ControllerStats& stats() const final { return stats_; }

protected:
    PathFollower(const SpeedLoopConfig& speed, bool delay_compensation,
                 const VehicleParams& nominal);

    // The steering angle for the car at `pose` (already predicted through the actuator delay)
    // with the estimate's velocities; `travel` bounds how far the car moved since the last call.
    [[nodiscard]] virtual double steer(const Pose2& pose, const StateEstimate& state,
                                       const Trajectory& reference, double travel) = 0;
    virtual void reset_steering() = 0;

    [[nodiscard]] const NominalModel& model() const { return model_; }

private:
    NominalModel model_;
    SpeedLoopConfig speed_config_;
    SpeedController speed_;
    bool delay_compensation_;
    ReferenceTracker cog_tracker_;  // for the speed target
    CallClock clock_;
    std::optional<ControlCommand> last_;
    ControllerStats stats_;
};

}  // namespace apex::control
