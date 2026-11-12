#pragma once

#include <span>

#include "apex/core/messages.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

// Sensor models. Each simulate_* function is pure given its Rng, so tests of the car's modules
// can produce realistic measurements without running a World. The sim module owns the noise
// structs below and fills them with documented, realistic defaults (docs/sim.md).
namespace apex::sim {

struct LidarNoise {};
struct CameraNoise {};
struct ImuNoise {};
struct WheelNoise {};
struct SteeringNoise {};

struct SensorNoise {
    LidarNoise lidar;
    CameraNoise camera;
    ImuNoise imu;
    WheelNoise wheels;
    SteeringNoise steering;
};

// One LiDAR revolution against the cones (3D cone bodies over a flat ground plane at z = 0).
[[nodiscard]] LidarScan simulate_lidar(const LidarGeometry& geometry, const LidarNoise& noise,
                                       const VehicleState& state, std::span<const Cone> cones,
                                       Time stamp, Rng& rng);

// The cone detector's output for one camera frame.
[[nodiscard]] CameraFrame simulate_camera(const CameraGeometry& geometry, const CameraNoise& noise,
                                          const VehicleState& state, std::span<const Cone> cones,
                                          Time stamp, Rng& rng);

}  // namespace apex::sim
