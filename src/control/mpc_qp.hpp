#pragma once

#include <vector>

#include "apex/control/controller.hpp"
#include "apex/control/mpc_controller.hpp"
#include "apex/control/mpc_plan.hpp"
#include "apex/control/vehicle_model.hpp"
#include "apex/core/trajectory.hpp"

// The MPC's quadratic program, built along a linearization point (docs/control.md, "MPC").
// With stage k's reference sampled at the guess's station s_k, and v_k the guess's speed:
//
//   cost  sum_{k=1..N} w_k/2 [q_y e_y^2 + q_c (e_psi + v_y / v_k)^2 + q_v (v_x - v_t,k)^2]
//       + sum_{k=0..N} 1/2 [r_d delta_rate^2 + r_a (a - a_ff,k)^2 + rho_2 sigma^2] + rho_1 sigma
//         with w_N = terminal_scale (1 otherwise), a_ff = a_ref + resistance(v_k), and the
//         speed target v_t,k the reference speed capped by what the car can reach from v_0
//   s.t.  x_{k+1} = A_k x_k + B_k u_k + c_k        (mpc_model.hpp, exact ZOH discretization)
//         |delta_rate| <= max_steer_rate, a within the traction limits at v_k     k < N
//         |delta| <= max_steer                                                   k >= 1
//         -w_right,k - sigma <= e_y <= w_left,k + sigma,  sigma >= 0            k >= 1
//
// where the lateral bounds are the reference's free widths less half the car's width and a
// margin. e_psi + v_y / v_x is the course error: the direction of travel, not the heading, has
// to follow the path once the car slides. The slack's linear penalty makes it exact (zero
// whenever the hard bounds are feasible, for a penalty above the bound's multiplier), the
// quadratic one keeps the QP strictly convex in it.
namespace apex::control::mpc {

// Fills `problem` (horizon guess.size() - 1) linearized along `guess`; guess[0].x is x0.
void build_qp(const MpcConfig& config, const NominalModel& model,
              const std::vector<StagePoint>& guess, double dt, Qp& problem);

// Moves the linearization point onto a solution: its inputs, rolled out from guess[0]'s state
// and station (mpc_plan.hpp).
void guess_from_solution(const NominalModel& model, const Trajectory& reference,
                         const Solution& solution, double dt, std::vector<StagePoint>& guess);

}  // namespace apex::control::mpc
