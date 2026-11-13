#pragma once

#include <span>
#include <vector>

#include <Eigen/Core>

#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/planning/planning.hpp"
#include "apex/planning/qp.hpp"

// The minimum-curvature raceline (docs/planning.md, "Raceline"), after Heilmeier, Wischnewski,
// Hermansdorfer, Betz, Lienkamp and Lohmann, "Minimum curvature trajectory planning and control
// for an autonomous race car", Vehicle System Dynamics 58(10), 2020.
namespace apex::planning {

// The QP of one linearization. The path is p_i = r_i + alpha_i n_i around a closed reference
// line r with unit left normals n, interpolated by a closed cubic spline with chord-length knot
// spacing h_i. The spline's second derivatives at the knots are linear in the values,
// f'' = Q f with Q = A^-1 B (A the cyclic tridiagonal matrix of the C2 conditions, B the
// matching differences); applied to the coordinates of r it gives the reference's curvature
// kappa_0 = (r' x r'') / |r'|^3. For a normal offset alpha(s) of an arc-length curve,
// p' = (1 - alpha kappa) t + alpha' n, and to first order the offset curve's curvature is
//
//   kappa = kappa_0 + alpha'' + kappa_0^2 alpha,   so   M = Q + diag(kappa_0^2).
//
// The kappa^2 alpha term matters in hairpins, where it is as large as alpha''. Holding the
// spline's first derivatives at the reference's instead, the simplest way to make the
// curvature linear, gets it with the wrong sign: a circle moved inward would seem to curve
// less, and a ring's optimum would be its inner edge.
//
// The QP minimizes the curvature energy sum_i w_i kappa_i^2 (w_i the arc length a knot stands
// for, approximating the integral of kappa^2 over s) plus regularization sum_i w_i alpha_i^2,
//
//   P = 2 (M' W M + regularization W),   q = 2 M' W kappa_0,
//
// subject to lower <= alpha <= upper (the free space) and, when max_curvature > 0,
// -max_curvature <= kappa_0 + M alpha <= max_curvature as 2n inequality rows. M is dense
// (A^-1 couples every knot), which is why the solver is a dense one.
struct MinCurvatureQp {
    std::vector<Vec2> reference;  // r_i
    std::vector<Vec2> normals;    // n_i
    Eigen::VectorXd curvature;    // kappa_0, the reference's curvature at its points (1/m)
    Eigen::MatrixXd jacobian;     // M (1/m^2)
    Eigen::VectorXd arc_weights;  // w_i (m)
    QpProblem qp;                 // over alpha (m)
};

// Throws std::invalid_argument for fewer than 4 points, repeated consecutive points, bounds of
// the wrong size, or a negative regularization or curvature bound.
[[nodiscard]] MinCurvatureQp min_curvature_qp(std::span<const Vec2> reference,
                                              std::span<const double> lower,
                                              std::span<const double> upper, double max_curvature,
                                              double regularization);

// What optimize_raceline did: each linearization's QP and our solver's result, in order.
struct RacelineTrace {
    std::vector<MinCurvatureQp> problems;
    std::vector<QpResult> results;
};

// optimize_raceline (planning.hpp), also recording every QP it solved in `trace` if not null:
// the OSQP comparison and the benchmarks re-solve exactly these problems.
[[nodiscard]] Raceline optimize_raceline(const TrackModel& track, const VehicleParams& nominal,
                                         const RacelineConfig& config, RacelineTrace* trace);

}  // namespace apex::planning
