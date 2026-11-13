#pragma once

#include "apex/core/track.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/runner/config.hpp"
#include "apex/runner/result.hpp"
#include "apex/sim/race.hpp"
#include "apex/sim/world.hpp"
#include "apex/stack/driver.hpp"

namespace apex::runner {

// One control step of a run, as observers see it.
struct TickView {
    Time time = 0.0;
    const VehicleState& truth;
    const sim::RaceMonitor& race;
    const stack::Driver& driver;
    const ControlCommand& command;
};

// Watches a run after every control step, for time-series logs and replays. Observers only
// read; nothing they do reaches the car or the simulator.
class RunObserver {
public:
    virtual ~RunObserver() = default;
    virtual void on_start(const Track& track) = 0;
    virtual void on_control(const TickView& tick) = 0;
};

// The track a spec names. Throws std::invalid_argument for an unknown layout and
// std::runtime_error for an unreadable file.
[[nodiscard]] Track make_track(const TrackSpec& spec);

// The simulator's side of a run: the true car, the friction and sensor-noise scales, the seed.
[[nodiscard]] sim::WorldConfig make_world_config(const RunConfig& config);

// One closed-loop run (docs/design.md, sections 4.4 and 5.6): a World and `driver` in
// lockstep, one 1 ms tick at a time. Each tick the world steps; its sensor messages enter a
// delay line and reach the driver `config.latency` after their stamps (under the
// perfect-perception oracle, each LiDAR scan is replaced by oracle observations with the LiDAR's
// latency and camera frames are dropped; under the perfect-state oracle the driver gets the
// true state every tick). Every control period the driver's command goes to the actuators.
//
// The run ends when the driver reports Finished or Failed, when the car comes to rest after the
// race officials have timed every lap (or stop_timeout passes), or on a DNF rule: stalled, off
// the track, out of time. Throws std::invalid_argument for an invalid config.
[[nodiscard]] RunResult run_closed_loop(const RunConfig& config, const Track& track,
                                        stack::Driver& driver, RunObserver* observer = nullptr);

}  // namespace apex::runner
