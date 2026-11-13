#include "osqp_reference.hpp"

#include <memory>
#include <vector>

#include <osqp.h>

#include "apex/core/timing.hpp"

namespace apex::control::testing {

namespace {

// A compressed-column copy with OSQP's index type. OSQP keeps pointers into these arrays, so
// they must outlive the solver.
struct CscArrays {
    std::vector<OSQPFloat> values;
    std::vector<OSQPInt> rows;
    std::vector<OSQPInt> columns;
    OSQPCscMatrix matrix{};

    explicit CscArrays(const Eigen::SparseMatrix<double>& m) {
        Eigen::SparseMatrix<double> compressed = m;
        compressed.makeCompressed();
        values.assign(compressed.valuePtr(), compressed.valuePtr() + compressed.nonZeros());
        rows.assign(compressed.innerIndexPtr(), compressed.innerIndexPtr() + compressed.nonZeros());
        columns.assign(compressed.outerIndexPtr(),
                       compressed.outerIndexPtr() + compressed.outerSize() + 1);
        OSQPCscMatrix_set_data(&matrix, static_cast<OSQPInt>(compressed.rows()),
                               static_cast<OSQPInt>(compressed.cols()),
                               static_cast<OSQPInt>(compressed.nonZeros()), values.data(),
                               rows.data(), columns.data());
    }
};

std::vector<OSQPFloat> clamp_infinity(const Eigen::VectorXd& v) {
    std::vector<OSQPFloat> out(static_cast<std::size_t>(v.size()));
    for (Eigen::Index i = 0; i < v.size(); ++i) {
        out[static_cast<std::size_t>(i)] = std::clamp(v(i), -OSQP_INFTY, OSQP_INFTY);
    }
    return out;
}

struct SolverDeleter {
    void operator()(OSQPSolver* solver) const { osqp_cleanup(solver); }
};

struct SettingsDeleter {
    void operator()(OSQPSettings* settings) const { OSQPSettings_free(settings); }
};

}  // namespace

OsqpResult solve_with_osqp(const SparseQp& qp, const OsqpOptions& options) {
    // OSQP takes the upper triangle of P.
    const Eigen::SparseMatrix<double> upper = qp.P.triangularView<Eigen::Upper>();
    CscArrays p(upper);
    CscArrays a(qp.A);
    const std::vector<OSQPFloat> q(qp.q.data(), qp.q.data() + qp.q.size());
    const std::vector<OSQPFloat> l = clamp_infinity(qp.lower);
    const std::vector<OSQPFloat> u = clamp_infinity(qp.upper);

    const std::unique_ptr<OSQPSettings, SettingsDeleter> settings(OSQPSettings_new());
    osqp_set_default_settings(settings.get());
    settings->verbose = 0;
    settings->eps_abs = options.eps_abs;
    settings->eps_rel = options.eps_rel;
    settings->max_iter = options.max_iter;
    settings->polishing = options.polishing ? 1 : 0;
    settings->scaled_termination = options.scaled_termination ? 1 : 0;
    settings->warm_starting = options.warm_x.size() > 0 ? 1 : 0;

    OsqpResult result;
    OSQPSolver* raw = nullptr;
    Stopwatch watch;
    const OSQPInt setup_error = osqp_setup(&raw, &p.matrix, q.data(), &a.matrix, l.data(), u.data(),
                                           static_cast<OSQPInt>(qp.A.rows()),
                                           static_cast<OSQPInt>(qp.A.cols()), settings.get());
    result.setup_seconds = watch.elapsed_seconds();
    const std::unique_ptr<OSQPSolver, SolverDeleter> solver(raw);
    if (setup_error != 0 || solver == nullptr) {
        result.status = "setup failed";
        return result;
    }
    watch.restart();
    if (options.warm_x.size() > 0) {
        osqp_warm_start(solver.get(), options.warm_x.data(),
                        options.warm_y.size() > 0 ? options.warm_y.data() : nullptr);
    }
    osqp_solve(solver.get());
    result.solve_seconds = watch.elapsed_seconds();

    const OSQPInfo* info = solver->info;
    result.status = info->status;
    result.iterations = static_cast<int>(info->iter);
    result.solved = info->status_val == OSQP_SOLVED;
    const OSQPSolution* solution = solver->solution;
    result.x = Eigen::Map<const Eigen::VectorXd>(solution->x, qp.A.cols());
    result.y = Eigen::Map<const Eigen::VectorXd>(solution->y, qp.A.rows());
    return result;
}

}  // namespace apex::control::testing
