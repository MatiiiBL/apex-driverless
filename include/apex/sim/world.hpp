#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/track.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/sim/race.hpp"
#include "apex/sim/sensors.hpp"

namespace apex::sim {

struct WorldConfig {
    VehicleParams vehicle;        // the true car
    SensorRig rig;                // sensor geometry and rates
    SensorNoise noise;            // sensor noise models (sim/sensors.hpp)
    double friction_scale = 1.0;  // multiplies both axles' mu: a dusty or damp track
    std::uint64_t seed = 1;       // every random draw in the world derives from this
};

// The simulated world: one car on one track, its sensors, and the race officials.
//
// Time advances in fixed 1 ms ticks. Each sensor fires on the ticks its rate divides, so stamps
// are exact multiples of its period. Deterministic: the same track, config and command sequence
// give bit-identical messages and states on the same build.
class World {
public:
    static constexpr double kTickSeconds = 1e-3;

    // Places the car at track.start_pose, at rest. Throws std::invalid_argument for a sensor
    // rate that does not divide 1 kHz or an invalid vehicle parameter.
    World(Track track, const WorldConfig& config);
    ~World();
    World(World&&) noexcept;
    World& operator=(World&&) noexcept;

    // Queues a command for the actuators, which respond after their dead times. Values beyond
    // the actuator limits are clamped by the vehicle model, not rejected.
    void command(const ControlCommand& command);

    // Advances one tick and returns the sensor messages stamped at the new time, in a fixed
    // order: IMU, wheel speeds, steering, camera, LiDAR.
    [[nodiscard]] std::vector<SensorMessage> step();

    [[nodiscard]] std::int64_t tick() const;
    [[nodiscard]] Time time() const;
    [[nodiscard]] const VehicleState& truth() const;
    [[nodiscard]] const Track& track() const;
    [[nodiscard]] const RaceMonitor& race() const;
    [[nodiscard]] const WorldConfig& config() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex::sim
