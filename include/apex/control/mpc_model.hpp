#pragma once

#include <array>
#include <cmath>

#include <Eigen/Core>

#include "apex/control/dual.hpp"
#include "apex/control/vehicle_model.hpp"

// The MPC's prediction model: the dynamic single-track model (vehicle_model.hpp) written in
// path coordinates along the reference (R. Rajamani, "Vehicle Dynamics and Control", Springer,
// 2nd ed. 2012, section 2.5; the Frenet frame of a racing MPC as in A. Liniger, A. Domahidi,
// M. Morari, "Optimization-based autonomous racing of 1:43 scale RC cars", Optimal Control
// Applications and Methods 36(5), 2015).
//
//   state  x = (e_y, e_psi, vx, vy, r, delta): lateral and heading error to the path, body
//              velocities, yaw rate, front wheel angle
//   input  u = (delta_rate, a, slack): steering rate, commanded acceleration (total tire force
//              over mass) and the slack of the soft lateral bounds, which does not enter the
//              dynamics
//
//   s_dot     = (vx cos e_psi - vy sin e_psi) / (1 - kappa e_y)
//   e_y_dot   = vx sin e_psi + vy cos e_psi
//   e_psi_dot = r - kappa s_dot
//   (vx, vy, r)_dot from the single-track model,   delta_dot = delta_rate
//
// with the path curvature kappa fixed per stage at the station the linearization point
// predicts. Making delta a state and its rate the input puts the actuator's rate limit into
// the QP as an input bound and lets the cost penalize steering rate directly.
namespace apex::control::mpc {

inline constexpr int kStates = 6;
inline constexpr int kInputs = 3;

enum StateIndex : int { kLateral = 0, kHeading = 1, kVx = 2, kVy = 3, kYawRate = 4, kSteer = 5 };
enum InputIndex : int { kSteerRate = 0, kAccel = 1, kSlack = 2 };

using State = Eigen::Matrix<double, kStates, 1>;
using Input = Eigen::Matrix<double, kInputs, 1>;

template <class T>
using StateArray = std::array<T, kStates>;

[[nodiscard]] inline StateArray<double> to_array(const State& x) {
    return {x(0), x(1), x(2), x(3), x(4), x(5)};
}

// Speed of the path station, s_dot. The denominator is floored at 0.1: a car within a tenth
// of the radius of the path's center of curvature is far outside any track.
template <class T>
[[nodiscard]] T path_speed(const StateArray<T>& x, double curvature) {
    using std::cos;
    using std::sin;
    T denominator = 1.0 - curvature * x[kLateral];
    if (value_of(denominator) < 0.1) denominator = 0.1;
    return (x[kVx] * cos(x[kHeading]) - x[kVy] * sin(x[kHeading])) / denominator;
}

template <class T>
[[nodiscard]] StateArray<T> error_dynamics(const NominalModel& model, const StateArray<T>& x,
                                           const T& steer_rate, const T& accel, double curvature) {
    using std::cos;
    using std::sin;
    const BodyRates<T> rates =
        model.body_rates(x[kVx], x[kVy], x[kYawRate], x[kSteer], steer_rate, accel);
    return {x[kVx] * sin(x[kHeading]) + x[kVy] * cos(x[kHeading]),
            x[kYawRate] - curvature * path_speed(x, curvature),
            rates.vx_dot,
            rates.vy_dot,
            rates.yaw_accel,
            steer_rate};
}

// x_{k+1} = A x_k + B u_k + c: the model linearized at (x, u) and discretized exactly over
// `dt` with the input held (zero-order hold). The Jacobians come from evaluating the model on
// dual numbers; the discretization is the exponential of the augmented matrix
// [[A_c, B_c, f - A_c x - B_c u], [0, 0, 0]] dt (matrix_exponential.hpp).
struct DiscreteModel {
    Eigen::Matrix<double, kStates, kStates> A;
    Eigen::Matrix<double, kStates, kInputs> B;
    State c;
};

[[nodiscard]] DiscreteModel discretize(const NominalModel& model, const State& x, const Input& u,
                                       double curvature, double dt);

// The nonlinear model integrated over `dt` with RK4 in `substeps` steps, input held: the
// reference the linearization is tested against.
[[nodiscard]] State simulate(const NominalModel& model, const State& x, const Input& u,
                             double curvature, double dt, int substeps);

}  // namespace apex::control::mpc
