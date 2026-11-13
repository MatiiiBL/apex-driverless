#pragma once

#include <cstddef>
#include <memory>
#include <string_view>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/timing.hpp"
#include "apex/core/trajectory.hpp"
#include "apex/core/vehicle.hpp"

// Path-tracking controllers. The control module owns every *Config struct's fields and
// defaults (docs/control.md).
namespace apex::control {

enum class ControllerKind { PurePursuit, Stanley, Mpc };

// The longitudinal loop shared by pure pursuit and Stanley: feed-forward of the reference
// acceleration plus the nominal drag and rolling resistance, and a PI term on the speed error
// with conditional integration and a clamped integral (anti-windup).
struct SpeedLoopConfig {
    double kp = 1.5;              // (m/s^2) per (m/s)
    double ki = 0.6;              // (m/s^2) per m of accumulated speed error
    double integral_limit = 2.0;  // bound on the integral term (m/s^2)
    // The target speed is read this far ahead of the car (s), which covers the drive's 20 ms
    // dead time and 50 ms lag.
    double preview_time = 0.07;
    double standstill_speed = 0.3;  // m/s: below this with a zero target, hold the brake
    double hold_brake = 1.5;        // m/s^2
};

// Pure pursuit (R. C. Coulter, "Implementation of the pure pursuit path tracking algorithm",
// CMU-RI-TR-92-01, 1992) from the rear axle, with lookahead lookahead_min + lookahead_time * v
// clamped to [lookahead_min, lookahead_max]. Short, because the law cuts corners by about a
// lookahead's worth of path, but not below 2 m, where the car weaves at walking pace in tight
// corners: docs/control.md has the sweeps these values come from.
struct PurePursuitConfig {
    double lookahead_min = 2.0;    // m
    double lookahead_time = 0.15;  // s
    double lookahead_max = 10.0;   // m
    // Corrects the kinematic law with the nominal model's steady-state slip angles: the frame
    // turned to the rear axle's velocity, plus the slip-angle difference (understeer).
    bool understeer_compensation = true;
    // Predicts the pose through the steering actuator's dead time and lag before steering.
    bool delay_compensation = true;
    SpeedLoopConfig speed;
};

// Stanley (G. M. Hoffmann, C. J. Tomlin, M. Montemerlo, S. Thrun, "Autonomous automobile
// trajectory tracking for off-road driving: controller design, experimental validation and
// racing", American Control Conference, 2007): front-axle heading and cross-track terms, the
// steady-state front slip as feed-forward and yaw-rate damping.
struct StanleyConfig {
    double gain = 2.0;             // k (1/s) in atan(k e / (softening_speed + v))
    double softening_speed = 1.0;  // m/s
    double yaw_damping = 0.05;     // s: steering per unit of yaw-rate error to the path's
    bool slip_feedforward = true;
    bool delay_compensation = true;
    SpeedLoopConfig speed;
};

// Linear time-varying MPC on the dynamic single-track error model (docs/control.md, "MPC").
struct MpcConfig {
    int horizon = 30;    // stages
    double step = 0.05;  // s per stage
    // Stage cost weights, on the scales of the errors they weigh.
    double weight_lateral = 6.0;     // 1/m^2, lateral error
    double weight_course = 4.0;      // 1/rad^2, heading error plus body sideslip
    double weight_speed = 1.0;       // 1/(m/s)^2
    double weight_steer_rate = 0.5;  // 1/(rad/s)^2
    double weight_accel = 0.02;      // 1/(m/s^2)^2, deviation from the feed-forward
    double terminal_scale = 3.0;     // multiplies the state weights at the last stage
    // Lateral bounds: the reference's free widths minus half the body width minus this margin,
    // soft with an exact penalty (linear) plus a quadratic one on the slack.
    double lateral_margin = 0.15;  // m
    double slack_linear = 200.0;
    double slack_quadratic = 2000.0;
    // Whether open references (lap 1's exploration paths) are bounded by their widths too. Off:
    // those widths come from a partial map, and near the car a quarter of them say the path
    // itself is narrower than the car (docs/control.md, "Lap 1").
    bool bound_open_references = false;
    // Acceleration bounds per stage: each axle's longitudinal force within this share of its
    // grip after the lateral force the plan needs (NominalModel::traction_limits).
    double grip_fraction = 0.9;
    // Dead-time compensation: the initial state is predicted through the commands in flight,
    // over the steering's dead time, plus its first-order lag time constant when compensate_lag.
    bool compensate_dead_time = true;
    bool compensate_lag = true;
    // Below kinematic_below the model is kinematic, above dynamic_above dynamic (slip angles
    // are ill-defined near standstill), blended linearly in between.
    double kinematic_below = 2.5;  // m/s
    double dynamic_above = 4.0;    // m/s
    // Real-time iterations: QPs per call (re-linearizing around each solution); the first call
    // after a reset, with no previous solution to linearize around, does more.
    int sqp_iterations = 1;
    int initial_sqp_iterations = 3;
    int max_qp_iterations = 30;
    double qp_tolerance = 1e-6;
    // On a solver failure the previous plan's command at this time is used while it is younger
    // than fallback_plan_age (s); after that, Stanley steering and fallback_decel braking.
    double fallback_plan_age = 0.3;
    double fallback_decel = 4.0;  // m/s^2
};

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
    std::size_t qp_iterations = 0;    // MPC: interior-point iterations, summed over calls
    std::size_t invalid_inputs = 0;   // calls with an invalid estimate or an empty reference
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

// Throws std::invalid_argument for an invalid configuration.
[[nodiscard]] std::unique_ptr<Controller> make_controller(const ControllerConfig& config,
                                                          const VehicleParams& nominal);

[[nodiscard]] std::string_view to_string(ControllerKind kind);

// JSON forms: kinds by name ("pure_pursuit", "stanley", "mpc"); configs as objects whose
// missing keys keep their defaults and whose unknown keys are rejected.
void to_json(nlohmann::json& j, ControllerKind kind);
void from_json(const nlohmann::json& j, ControllerKind& kind);
void to_json(nlohmann::json& j, const SpeedLoopConfig& config);
void from_json(const nlohmann::json& j, SpeedLoopConfig& config);
void to_json(nlohmann::json& j, const PurePursuitConfig& config);
void from_json(const nlohmann::json& j, PurePursuitConfig& config);
void to_json(nlohmann::json& j, const StanleyConfig& config);
void from_json(const nlohmann::json& j, StanleyConfig& config);
void to_json(nlohmann::json& j, const MpcConfig& config);
void from_json(const nlohmann::json& j, MpcConfig& config);
void to_json(nlohmann::json& j, const ControllerConfig& config);
void from_json(const nlohmann::json& j, ControllerConfig& config);

}  // namespace apex::control
