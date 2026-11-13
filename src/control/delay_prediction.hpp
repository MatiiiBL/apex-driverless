#pragma once

#include <deque>

#include "apex/control/vehicle_model.hpp"
#include "apex/core/messages.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

// What the car will do before a new command can act. The steering responds 30 ms after a
// command and then lags it by a first-order 30 ms; the drive by 20 + 50 ms (docs/design.md,
// section 4.4). A command computed now therefore acts on the car as it will be 60-70 ms from
// now, and the commands already sent decide what happens until then. Each actuator is
// approximated as a pure delay (its dead time, plus its time constant when the lag is
// compensated too): its output at time t is the command sent at t - delay.
namespace apex::control {

// The commands sent over the last fraction of a second.
class CommandHistory {
public:
    void add(const ControlCommand& command);
    void clear() { commands_.clear(); }
    [[nodiscard]] bool empty() const { return commands_.empty(); }

    // The newest command sent at or before t; `fallback` when there is none.
    [[nodiscard]] ControlCommand at(Time t, const ControlCommand& fallback) const;

private:
    std::deque<ControlCommand> commands_;  // by stamp
};

// The car's state in the world frame.
struct WorldState {
    Pose2 pose;
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
};

struct ActuatorDelays {
    double steer = 0.0;  // s
    double drive = 0.0;  // s
};

// Integrates `state` from its stamp to `until` with the nominal model (RK4, steps of at most
// 5 ms), the steering and acceleration at each time being the commands sent one delay earlier
// (`before` for times older than the history). The car does not reverse.
[[nodiscard]] WorldState predict_through_delays(const NominalModel& model,
                                                const StateEstimate& state, Time until,
                                                const CommandHistory& history,
                                                const ActuatorDelays& delays,
                                                const ControlCommand& before);

}  // namespace apex::control
