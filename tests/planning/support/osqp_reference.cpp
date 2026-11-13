#include "osqp_reference.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

#include <osqp.h>

namespace apex::planning::test_support {

namespace {

// A matrix in OSQP's compressed sparse column form, owning its arrays.
struct Csc {
    std::vector<OSQPFloat> values;
    std::vector<OSQPInt> rows;
    std::vector<OSQPInt> column_starts{0};
    OSQPCscMatrix matrix{};

    void end_column() { column_starts.push_back(static_cast<OSQPInt>(values.size())); }
    void add(Eigen::Index row, double value) {
        rows.push_back(static_cast<OSQPInt>(row));
        values.push_back(value);
    }
    OSQPCscMatrix* finish(Eigen::Index row_count, Eigen::Index column_count) {
        matrix.m = static_cast<OSQPInt>(row_count);
        matrix.n = static_cast<OSQPInt>(column_count);
        matrix.p = column_starts.data();
        matrix.i = rows.data();
        matrix.x = values.data();
        matrix.nzmax = static_cast<OSQPInt>(values.size());
        matrix.nz = -1;    // compressed columns, not triplets
        matrix.owned = 0;  // the vectors keep the arrays
        return &matrix;
    }
};

double clamp_infinity(double value) {
    return std::clamp(value, -OSQP_INFTY, OSQP_INFTY);
}

}  // namespace

OsqpResult solve_with_osqp(const QpProblem& problem, const OsqpOptions& options) {
    const Eigen::Index n = problem.q.size();
    const Eigen::Index m = problem.G.rows();

    Csc P;  // upper triangle, as OSQP requires
    for (Eigen::Index j = 0; j < n; ++j) {
        for (Eigen::Index i = 0; i <= j; ++i) {
            if (problem.P(i, j) != 0.0) P.add(i, problem.P(i, j));
        }
        P.end_column();
    }
    Csc A;  // [I; G]
    for (Eigen::Index j = 0; j < n; ++j) {
        A.add(j, 1.0);
        for (Eigen::Index k = 0; k < m; ++k) {
            if (problem.G(k, j) != 0.0) A.add(n + k, problem.G(k, j));
        }
        A.end_column();
    }
    std::vector<OSQPFloat> l(static_cast<std::size_t>(n + m), -OSQP_INFTY);
    std::vector<OSQPFloat> u(static_cast<std::size_t>(n + m), OSQP_INFTY);
    for (Eigen::Index i = 0; i < n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        if (problem.lower.size() > 0) l[k] = clamp_infinity(problem.lower[i]);
        if (problem.upper.size() > 0) u[k] = clamp_infinity(problem.upper[i]);
    }
    for (Eigen::Index k = 0; k < m; ++k) u[static_cast<std::size_t>(n + k)] = problem.h[k];
    std::vector<OSQPFloat> q(problem.q.data(), problem.q.data() + n);

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.eps_abs = options.tolerance;
    settings.eps_rel = options.tolerance;
    settings.max_iter = options.max_iterations;
    settings.polishing = options.polish ? 1 : 0;

    OSQPSolver* solver = nullptr;
    if (osqp_setup(&solver, P.finish(n, n), q.data(), A.finish(n + m, n), l.data(), u.data(),
                   static_cast<OSQPInt>(n + m), static_cast<OSQPInt>(n), &settings) != 0) {
        throw std::runtime_error("OSQP setup failed");
    }
    osqp_solve(solver);
    OsqpResult result;
    result.x = Eigen::Map<const Eigen::VectorXd>(solver->solution->x, n);
    result.objective = solver->info->obj_val;
    result.solved = solver->info->status_val == OSQP_SOLVED;
    result.iterations = solver->info->iter;
    osqp_cleanup(solver);
    return result;
}

}  // namespace apex::planning::test_support
