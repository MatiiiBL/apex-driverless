#pragma once

#include <cstddef>

#include <Eigen/Core>

namespace apex::planning {

// A dense convex quadratic program
//
//   minimize    1/2 x' P x + q' x
//   subject to  lower <= x <= upper   (entries may be -inf or +inf)
//               G x <= h
//
// with P symmetric positive semidefinite, n variables and m general inequality rows.
struct QpProblem {
    Eigen::MatrixXd P;      // n x n
    Eigen::VectorXd q;      // n
    Eigen::VectorXd lower;  // n, or empty for no lower bounds
    Eigen::VectorXd upper;  // n, or empty for no upper bounds
    Eigen::MatrixXd G;      // m x n (m may be 0)
    Eigen::VectorXd h;      // m
};

struct QpSettings {
    std::size_t max_iterations = 50;
    // Stop when the primal and dual residuals and the duality gap are all below this, relative
    // to the size of the data.
    double tolerance = 1e-9;
};

enum class QpStatus {
    Solved,
    InvalidBounds,  // some lower bound exceeds its upper bound
    NotConverged,   // iteration limit, or the problem is infeasible or unbounded
};

struct QpResult {
    Eigen::VectorXd x;  // the last iterate when not Solved
    QpStatus status = QpStatus::NotConverged;
    std::size_t iterations = 0;
    double objective = 0.0;
};

// Solves the QP with a primal-dual interior-point method: Mehrotra's predictor-corrector
// (Mehrotra, "On the implementation of a primal-dual interior point method", SIAM Journal on
// Optimization 2(4), 1992; for QPs as in Nocedal and Wright, "Numerical Optimization", 2nd ed.,
// Springer, 2006, section 16.6), from an infeasible start. Each iteration solves the reduced
// Newton system (P + C' W C) dx = r with one dense Cholesky factorization, C being the bound and
// inequality rows and W their diagonal of multiplier over slack; bounds only add to its
// diagonal. The iteration count (typically 10 to 25) barely depends on the problem's
// conditioning, which matters for the ill-conditioned minimum-curvature QPs of the raceline,
// where first-order methods such as ADMM need many iterations for a precise answer.
//
// Throws std::invalid_argument when the dimensions do not match.
[[nodiscard]] QpResult solve_qp(const QpProblem& problem, const QpSettings& settings = {});

}  // namespace apex::planning
