#pragma once

#include <cstdint>

#include <Eigen/Core>

#include "apex/planning/qp.hpp"

// OSQP (Stellato et al., "OSQP: an operator splitting solver for quadratic programs",
// Mathematical Programming Computation 12, 2020) as the reference the planning module's own QP
// solver is checked and benchmarked against. Tests and benchmarks only: the car's software never
// links it.
namespace apex::planning::test_support {

struct OsqpResult {
    Eigen::VectorXd x;
    double objective = 0.0;
    bool solved = false;
    std::int64_t iterations = 0;
};

struct OsqpOptions {
    double tolerance = 1e-9;  // eps_abs and eps_rel
    bool polish = true;
    std::int64_t max_iterations = 1'000'000;
};

// Solves `problem` with OSQP: the bounds become identity rows of OSQP's l <= A x <= u, stacked
// over the rows of G. Includes setup (OSQP's factorization) in what it does.
[[nodiscard]] OsqpResult solve_with_osqp(const QpProblem& problem, const OsqpOptions& options = {});

}  // namespace apex::planning::test_support
