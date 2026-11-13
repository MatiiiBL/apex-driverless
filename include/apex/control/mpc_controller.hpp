#pragma once

#include <memory>
#include <string_view>

#include "apex/control/controller.hpp"
#include "apex/control/mpc_model.hpp"
#include "apex/control/mpc_plan.hpp"
#include "apex/control/qp/ocp_qp.hpp"

namespace apex::control {

namespace mpc {
using Qp = qp::OcpQp<kStates, kInputs>;
using Solution = qp::OcpSolution<kStates, kInputs>;
}  // namespace mpc

// Linear time-varying MPC on the dynamic single-track error model (mpc_model.hpp), solved by
// the module's Riccati interior-point method (qp/interior_point.hpp). Each call:
//
// 1. predicts the car's state through the actuators' dead time: the estimate is integrated
//    with the nominal model to t0 = now + delay, the steering and drive following the commands
//    already sent (each actuator approximated as a pure delay of its dead time, plus its lag
//    time constant when compensate_lag);
// 2. projects the prediction onto the reference: station s0 and the error state x0;
// 3. linearizes the model along a guess of the trajectory: the previous plan, shifted in time
//    and re-projected onto the current reference (mpc_plan.hpp), or one built from the
//    reference after a reset;
// 4. solves the QP: tracking cost on lateral error, course and speed, effort cost on steering
//    rate and on acceleration beyond the feed-forward; hard steering angle, rate and
//    acceleration limits; lateral bounds from the reference's widths, softened by a slack with
//    linear and quadratic penalties; then re-linearizes around the solution and solves again
//    for sqp_iterations (real-time iterations);
// 5. sends the plan's steering one period ahead and its first acceleration, and keeps the plan,
//    in the world frame, as the next call's warm start.
//
// On a failed solve it counts the failure and falls back to the previous plan while it is
// younger than fallback_plan_age, then to Stanley steering and braking. Details and the cost
// and constraint formulas in docs/control.md.
class MpcController final : public Controller {
public:
    // Throws std::invalid_argument for an invalid configuration or vehicle.
    MpcController(const MpcConfig& config, const VehicleParams& nominal);
    ~MpcController() override;
    MpcController(const MpcController&) = delete;
    MpcController& operator=(const MpcController&) = delete;

    [[nodiscard]] std::string_view name() const override { return "mpc"; }
    [[nodiscard]] ControlCommand compute(const StateEstimate& state, const Trajectory& reference,
                                         Time now) override;
    void reset() override;
    [[nodiscard]] const ControllerStats& stats() const override;

    // The last QP solved and its solution, for the solver's tests and benchmarks: the
    // controller's real problems. Empty before the first solve.
    [[nodiscard]] const mpc::Qp& last_problem() const;
    [[nodiscard]] const mpc::Solution& last_solution() const;
    // The current plan in the world frame (empty after a reset or a fallback to Stanley).
    [[nodiscard]] const mpc::Plan& plan() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace apex::control
