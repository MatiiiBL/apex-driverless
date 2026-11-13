#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "apex/control/qp/ocp_qp.hpp"

// OSQP as the reference oracle for the control module's QP solver (tests and benchmarks only;
// the library never links OSQP). An OCP-QP is flattened into OSQP's form
//   min 1/2 z' P z + q' z   s.t.   l <= A z <= u
// over z = [u_0, x_1, u_1, ..., x_N, u_N] (x_0 is data), with the dynamics as equality rows
// A_k x_k + B_k u_k - x_{k+1} = -b_k, so OSQP's multipliers of those rows are the solver's pi_k
// and its multipliers of the constraint rows are the solver's signed y.
namespace apex::control::testing {

struct SparseQp {
    Eigen::SparseMatrix<double> P;  // symmetric, both triangles stored
    Eigen::VectorXd q;
    Eigen::SparseMatrix<double> A;
    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};

struct OsqpOptions {
    double eps_abs = 1e-6;
    double eps_rel = 0.0;
    int max_iter = 200000;
    bool polishing = false;
    bool scaled_termination = false;
    // Warm start from these primal and dual vectors when non-empty.
    Eigen::VectorXd warm_x;
    Eigen::VectorXd warm_y;
};

struct OsqpResult {
    bool solved = false;
    std::string status;
    Eigen::VectorXd x;
    Eigen::VectorXd y;
    int iterations = 0;
    double setup_seconds = 0.0;  // wall clock of osqp_setup (scaling, KKT factorization)
    double solve_seconds = 0.0;  // wall clock of osqp_solve (and warm start)
};

[[nodiscard]] OsqpResult solve_with_osqp(const SparseQp& qp, const OsqpOptions& options);

namespace detail {

using Index = Eigen::Index;

template <int Nx, int Nu>
[[nodiscard]] Index x_offset(std::size_t k) {  // k >= 1
    return Nu + (static_cast<Index>(k) - 1) * (Nx + Nu);
}

template <int Nx, int Nu>
[[nodiscard]] Index u_offset(std::size_t k) {
    return k == 0 ? 0 : x_offset<Nx, Nu>(k) + Nx;
}

template <int Nx, int Nu>
[[nodiscard]] Index variable_count(const qp::OcpQp<Nx, Nu>& problem) {
    return Nu + static_cast<Index>(problem.horizon()) * (Nx + Nu);
}

template <int Nx, int Nu>
[[nodiscard]] Index row_count(const qp::OcpQp<Nx, Nu>& problem) {
    Index rows = static_cast<Index>(problem.horizon()) * Nx;
    for (const auto& stage : problem.stages) rows += static_cast<Index>(stage.constraints.size());
    return rows;
}

}  // namespace detail

template <int Nx, int Nu>
[[nodiscard]] SparseQp to_sparse(const qp::OcpQp<Nx, Nu>& problem) {
    using detail::Index;
    using detail::u_offset;
    using detail::x_offset;
    using Triplet = Eigen::Triplet<double>;
    const std::size_t n = problem.horizon();
    const Index variables = detail::variable_count(problem);
    const Index rows = detail::row_count(problem);

    SparseQp out;
    out.q = Eigen::VectorXd::Zero(variables);
    out.lower = Eigen::VectorXd::Zero(rows);
    out.upper = Eigen::VectorXd::Zero(rows);
    std::vector<Triplet> p_entries;
    std::vector<Triplet> a_entries;
    auto add = [](std::vector<Triplet>& entries, Index i, Index j, double value) {
        if (value != 0.0) entries.emplace_back(static_cast<int>(i), static_cast<int>(j), value);
    };
    for (std::size_t k = 0; k <= n; ++k) {
        const auto& stage = problem.stages[k];
        const Index uo = u_offset<Nx, Nu>(k);
        for (Index i = 0; i < Nu; ++i) {
            for (Index j = 0; j < Nu; ++j) add(p_entries, uo + i, uo + j, stage.R(i, j));
        }
        if (k == 0) {
            out.q.segment<Nu>(uo) = stage.r + stage.S * problem.x0;
            continue;
        }
        const Index xo = x_offset<Nx, Nu>(k);
        for (Index i = 0; i < Nx; ++i) {
            for (Index j = 0; j < Nx; ++j) add(p_entries, xo + i, xo + j, stage.Q(i, j));
        }
        for (Index i = 0; i < Nu; ++i) {
            for (Index j = 0; j < Nx; ++j) {
                add(p_entries, uo + i, xo + j, stage.S(i, j));
                add(p_entries, xo + j, uo + i, stage.S(i, j));
            }
        }
        out.q.segment<Nx>(xo) = stage.q;
        out.q.segment<Nu>(uo) = stage.r;
    }

    Index row = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const auto& stage = problem.stages[k];
        Eigen::Matrix<double, Nx, 1> rhs = -stage.b;
        if (k == 0) rhs -= stage.A * problem.x0;
        for (Index i = 0; i < Nx; ++i) {
            if (k > 0) {
                for (Index j = 0; j < Nx; ++j) {
                    add(a_entries, row + i, x_offset<Nx, Nu>(k) + j, stage.A(i, j));
                }
            }
            for (Index j = 0; j < Nu; ++j) {
                add(a_entries, row + i, u_offset<Nx, Nu>(k) + j, stage.B(i, j));
            }
            add(a_entries, row + i, x_offset<Nx, Nu>(k + 1) + i, -1.0);
            out.lower(row + i) = rhs(i);
            out.upper(row + i) = rhs(i);
        }
        row += Nx;
    }
    for (std::size_t k = 0; k <= n; ++k) {
        for (const auto& c : problem.stages[k].constraints) {
            double shift = 0.0;
            if (k == 0) {
                shift = c.c.dot(problem.x0);
            } else {
                for (Index j = 0; j < Nx; ++j) add(a_entries, row, x_offset<Nx, Nu>(k) + j, c.c(j));
            }
            for (Index j = 0; j < Nu; ++j) add(a_entries, row, u_offset<Nx, Nu>(k) + j, c.d(j));
            out.lower(row) = c.lower - shift;
            out.upper(row) = c.upper - shift;
            ++row;
        }
    }
    out.P.resize(variables, variables);
    out.P.setFromTriplets(p_entries.begin(), p_entries.end());
    out.A.resize(rows, variables);
    out.A.setFromTriplets(a_entries.begin(), a_entries.end());
    return out;
}

// Maps OSQP's primal and dual vectors back onto the stage structure, with the residuals and
// objective evaluated by the same functions that score the solver's own solutions.
template <int Nx, int Nu>
[[nodiscard]] qp::OcpSolution<Nx, Nu> from_sparse(const qp::OcpQp<Nx, Nu>& problem,
                                                  const OsqpResult& result) {
    using detail::Index;
    using detail::u_offset;
    using detail::x_offset;
    const std::size_t n = problem.horizon();
    qp::OcpSolution<Nx, Nu> s;
    s.x.resize(n + 1);
    s.u.resize(n + 1);
    s.pi.resize(n);
    s.y.resize(n + 1);
    s.x[0] = problem.x0;
    for (std::size_t k = 0; k <= n; ++k) {
        s.u[k] = result.x.segment<Nu>(u_offset<Nx, Nu>(k));
        if (k > 0) s.x[k] = result.x.segment<Nx>(x_offset<Nx, Nu>(k));
    }
    Index row = 0;
    for (std::size_t k = 0; k < n; ++k, row += Nx) s.pi[k] = result.y.segment<Nx>(row);
    for (std::size_t k = 0; k <= n; ++k) {
        for (std::size_t j = 0; j < problem.stages[k].constraints.size(); ++j) {
            s.y[k].push_back(result.y(row++));
        }
    }
    s.status = result.solved ? qp::QpStatus::Solved : qp::QpStatus::MaxIterations;
    s.iterations = result.iterations;
    s.objective = qp::objective(problem, s);
    s.residuals = qp::kkt_residuals(problem, s);
    return s;
}

// A stage-structured solution as OSQP's primal and dual vectors, for warm starts.
template <int Nx, int Nu>
void to_sparse_vectors(const qp::OcpQp<Nx, Nu>& problem, const qp::OcpSolution<Nx, Nu>& s,
                       Eigen::VectorXd& x, Eigen::VectorXd& y) {
    using detail::Index;
    using detail::u_offset;
    using detail::x_offset;
    const std::size_t n = problem.horizon();
    x.resize(detail::variable_count(problem));
    y.resize(detail::row_count(problem));
    for (std::size_t k = 0; k <= n; ++k) {
        x.segment<Nu>(u_offset<Nx, Nu>(k)) = s.u[k];
        if (k > 0) x.segment<Nx>(x_offset<Nx, Nu>(k)) = s.x[k];
    }
    Index row = 0;
    for (std::size_t k = 0; k < n; ++k, row += Nx) y.segment<Nx>(row) = s.pi[k];
    for (std::size_t k = 0; k <= n; ++k) {
        for (const double value : s.y[k]) y(row++) = value;
    }
}

}  // namespace apex::control::testing
