#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string_view>
#include <vector>

#include <Eigen/Core>

// Quadratic programs with the stage-wise structure of an optimal control problem (OCP-QP), the
// form a linear(ized) MPC produces:
//
//   min   sum_{k=0..N} 1/2 x_k' Q_k x_k + u_k' S_k x_k + 1/2 u_k' R_k u_k + q_k' x_k + r_k' u_k
//   s.t.  x_{k+1} = A_k x_k + B_k u_k + b_k,                 k = 0..N-1,  x_0 given
//         lower_kj <= c_kj' x_k + d_kj' u_k <= upper_kj      for each constraint j of stage k
//
// The last stage N may carry an input too (the MPC uses it for the slack of its terminal soft
// constraint). Bounds may be infinite. State and input sizes are compile-time constants, so the
// per-stage linear algebra runs on fixed-size Eigen matrices without heap allocation.
namespace apex::control::qp {

inline constexpr double kInfinity = std::numeric_limits<double>::infinity();

template <int Nx, int Nu>
struct Types {
    using StateVector = Eigen::Matrix<double, Nx, 1>;
    using InputVector = Eigen::Matrix<double, Nu, 1>;
    using StateMatrix = Eigen::Matrix<double, Nx, Nx>;
    using InputMatrix = Eigen::Matrix<double, Nu, Nu>;
    using InputStateMatrix = Eigen::Matrix<double, Nu, Nx>;  // S, Riccati gains
    using StateInputMatrix = Eigen::Matrix<double, Nx, Nu>;  // B
};

// One row of a stage's constraints: lower <= c' x + d' u <= upper.
template <int Nx, int Nu>
struct LinearConstraint {
    typename Types<Nx, Nu>::StateVector c = Types<Nx, Nu>::StateVector::Zero();
    typename Types<Nx, Nu>::InputVector d = Types<Nx, Nu>::InputVector::Zero();
    double lower = -kInfinity;
    double upper = kInfinity;
};

template <int Nx, int Nu>
struct OcpStage {
    using T = Types<Nx, Nu>;
    // Dynamics to the next stage; ignored at the last stage.
    typename T::StateMatrix A = T::StateMatrix::Identity();
    typename T::StateInputMatrix B = T::StateInputMatrix::Zero();
    typename T::StateVector b = T::StateVector::Zero();
    // Cost. The stage Hessian [Q S'; S R] must be positive semidefinite and R + B' P B positive
    // definite along the recursion (true whenever R is positive definite).
    typename T::StateMatrix Q = T::StateMatrix::Zero();
    typename T::InputStateMatrix S = T::InputStateMatrix::Zero();
    typename T::InputMatrix R = T::InputMatrix::Identity();
    typename T::StateVector q = T::StateVector::Zero();
    typename T::InputVector r = T::InputVector::Zero();
    std::vector<LinearConstraint<Nx, Nu>> constraints;
};

template <int Nx, int Nu>
struct OcpQp {
    typename Types<Nx, Nu>::StateVector x0 = Types<Nx, Nu>::StateVector::Zero();
    std::vector<OcpStage<Nx, Nu>> stages;  // N + 1 stages, 0..N

    [[nodiscard]] std::size_t horizon() const { return stages.empty() ? 0 : stages.size() - 1; }
};

enum class QpStatus {
    Solved,            // every residual within its tolerance
    SolvedInaccurate,  // iteration limit reached, residuals within a looser tolerance
    MaxIterations,     // iteration limit reached far from a solution
    PrimalInfeasible,  // a certificate of infeasibility was found (or lower > upper)
    NumericalError,    // a factorization failed or the iterates became non-finite
    InvalidProblem,    // inconsistent sizes or non-finite data
};

[[nodiscard]] constexpr std::string_view to_string(QpStatus status) {
    switch (status) {
        case QpStatus::Solved:
            return "solved";
        case QpStatus::SolvedInaccurate:
            return "solved_inaccurate";
        case QpStatus::MaxIterations:
            return "max_iterations";
        case QpStatus::PrimalInfeasible:
            return "primal_infeasible";
        case QpStatus::NumericalError:
            return "numerical_error";
        case QpStatus::InvalidProblem:
            return "invalid_problem";
    }
    return "unknown";
}

// A solution can be used by a controller in these two cases.
[[nodiscard]] constexpr bool usable(QpStatus status) {
    return status == QpStatus::Solved || status == QpStatus::SolvedInaccurate;
}

// Infinity norms of the KKT conditions' residuals.
struct QpResiduals {
    double stationarity = 0.0;     // gradient of the Lagrangian
    double dynamics = 0.0;         // x_{k+1} - A x_k - B u_k - b_k
    double inequality = 0.0;       // violation of the constraint bounds
    double complementarity = 0.0;  // multiplier times distance to its bound
};

template <int Nx, int Nu>
struct OcpSolution {
    using T = Types<Nx, Nu>;
    std::vector<typename T::StateVector> x;   // N + 1 states, x[0] = x0
    std::vector<typename T::InputVector> u;   // N + 1 inputs
    std::vector<typename T::StateVector> pi;  // N multipliers of the dynamics
    // Per stage and constraint, the signed multiplier y = y_upper - y_lower (positive when the
    // upper bound is active), the convention OSQP uses.
    std::vector<std::vector<double>> y;
    QpStatus status = QpStatus::InvalidProblem;
    int iterations = 0;
    QpResiduals residuals;
    double objective = 0.0;
};

// The objective of a candidate solution.
template <int Nx, int Nu>
[[nodiscard]] double objective(const OcpQp<Nx, Nu>& problem, const OcpSolution<Nx, Nu>& solution) {
    double value = 0.0;
    for (std::size_t k = 0; k < problem.stages.size(); ++k) {
        const OcpStage<Nx, Nu>& stage = problem.stages[k];
        const auto& x = solution.x[k];
        const auto& u = solution.u[k];
        value += 0.5 * x.dot(stage.Q * x) + u.dot(stage.S * x) + 0.5 * u.dot(stage.R * u) +
                 stage.q.dot(x) + stage.r.dot(u);
    }
    return value;
}

// The KKT residuals of a candidate primal-dual solution, computed from the problem data alone.
// Tests use it as the solver-independent check of optimality, and to score OSQP's solutions on
// the same scale.
template <int Nx, int Nu>
[[nodiscard]] QpResiduals kkt_residuals(const OcpQp<Nx, Nu>& problem,
                                        const OcpSolution<Nx, Nu>& solution) {
    QpResiduals res;
    const std::size_t n = problem.horizon();
    for (std::size_t k = 0; k <= n; ++k) {
        const OcpStage<Nx, Nu>& stage = problem.stages[k];
        const auto& x = solution.x[k];
        const auto& u = solution.u[k];
        typename Types<Nx, Nu>::StateVector grad_x =
            stage.Q * x + stage.S.transpose() * u + stage.q;
        typename Types<Nx, Nu>::InputVector grad_u = stage.R * u + stage.S * x + stage.r;
        if (k < n) {
            grad_x += stage.A.transpose() * solution.pi[k];
            grad_u += stage.B.transpose() * solution.pi[k];
            const auto defect = stage.A * x + stage.B * u + stage.b - solution.x[k + 1];
            res.dynamics = std::max(res.dynamics, defect.template lpNorm<Eigen::Infinity>());
        }
        if (k > 0) grad_x -= solution.pi[k - 1];
        for (std::size_t j = 0; j < stage.constraints.size(); ++j) {
            const LinearConstraint<Nx, Nu>& row = stage.constraints[j];
            const double y = solution.y[k][j];
            grad_x += y * row.c;
            grad_u += y * row.d;
            const double value = row.c.dot(x) + row.d.dot(u);
            res.inequality = std::max({res.inequality, value - row.upper, row.lower - value});
            const double gap = y > 0.0 ? y * (row.upper - value) : -y * (value - row.lower);
            if (std::isfinite(gap)) res.complementarity = std::max(res.complementarity, gap);
        }
        // x_0 is data, not a variable: its gradient is not part of the conditions.
        if (k > 0) {
            res.stationarity =
                std::max(res.stationarity, grad_x.template lpNorm<Eigen::Infinity>());
        }
        res.stationarity = std::max(res.stationarity, grad_u.template lpNorm<Eigen::Infinity>());
    }
    return res;
}

}  // namespace apex::control::qp
