#pragma once

#include <memory>
#include <string_view>

#include "apex/core/messages.hpp"
#include "apex/core/timing.hpp"
#include "apex/core/trajectory.hpp"
#include "apex/core/vehicle.hpp"

// Path-tracking controllers. The control module owns every *Config struct's fields and
// defaults (docs/control.md).
namespace apex::control {

enum class ControllerKind { PurePursuit, Stanley, Mpc };

struct PurePursuitConfig {};
struct StanleyConfig {};
struct MpcConfig {};

struct ControllerConfig {
    ControllerKind kind = ControllerKind::Mpc;
    PurePursuitConfig pure_pursuit;
    StanleyConfig stanley;
    MpcConfig mpc;
};

struct ControllerStats {
    LatencyStats compute_time;  // wall-clock time per compute()
    std::size_t calls = 0;
    std::size_t solver_failures = 0;  // MPC: QP not solved; a fallback command was used
};

// Tracks a reference trajectory (its path and its speed profile). The reference may change
// between calls: during lap 1 the exploration planner replans at every LiDAR scan, so a
// controller keeps its warm start by arc length, not by index. compute() never throws and never
// returns a non-finite command: on a solver failure it falls back and counts the failure.
class Controller {
public:
    virtual ~Controller() = default;
    [[nodiscard]] virtual std::string_view name() const = 0;
    [[nodiscard]] virtual ControlCommand compute(const StateEstimate& state,
                                                 const Trajectory& reference, Time now) = 0;
    // Drops internal state (warm starts, integrators), e.g. when switching to the raceline.
    virtual void reset() = 0;
    [[nodiscard]] virtual const ControllerStats& stats() const = 0;
};

[[nodiscard]] std::unique_ptr<Controller> make_controller(const ControllerConfig& config,
                                                          const VehicleParams& nominal);

}  // namespace apex::control
