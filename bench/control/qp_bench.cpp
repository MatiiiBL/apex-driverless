// The MPC's QP solver against OSQP on the controller's own problems.
//
// For each horizon, the MPC drives the simulated car around the "mixed" reference track and
// every QP it builds is solved again, outside the loop, four ways: by the Riccati
// interior-point method cold and warm-started from the previous call's solution, and by OSQP
// cold and warm-started the same way, both at the same absolute tolerance. OSQP pays a sparse
// LDL' factorization in its setup whenever the matrices change, which in an LTV MPC is every
// call; its solve-only time is reported too, as the bound it would reach if that came free.
// The KKT residuals of both solutions are measured on the same scale (qp::kkt_residuals).
//
//   apex_control_qp_bench [--out results/control] [--calls 1000]
//
// Writes qp_bench.json and qp_bench.md to the output directory.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/control/mpc_controller.hpp"
#include "apex/control/qp/interior_point.hpp"
#include "apex/core/timing.hpp"
#include "apex/sim/track_gen.hpp"

#include "closed_loop.hpp"
#include "osqp_reference.hpp"
#include "plant.hpp"
#include "references.hpp"

namespace {

using namespace apex;
using namespace apex::control;
using nlohmann::json;

constexpr double kTolerance = 1e-6;
constexpr int kWarmup = 25;  // calls before the first measured QP (the launch)

struct SolverRecord {
    LatencyStats time;
    std::size_t iterations = 0;
    std::size_t solved = 0;
    std::size_t count = 0;
    std::vector<double> residuals;  // per QP, max of stationarity, dynamics and inequality

    void add(double seconds, int iters, bool ok, const qp::QpResiduals& r) {
        time.add(seconds);
        iterations += static_cast<std::size_t>(iters);
        solved += ok ? 1 : 0;
        ++count;
        residuals.push_back(std::max({r.stationarity, r.dynamics, r.inequality}));
    }

    [[nodiscard]] json summary() const {
        json j = time.summary();
        j["mean_iterations"] =
            count > 0 ? static_cast<double>(iterations) / static_cast<double>(count) : 0.0;
        j["solved_fraction"] =
            count > 0 ? static_cast<double>(solved) / static_cast<double>(count) : 0.0;
        std::vector<double> sorted = residuals;
        std::sort(sorted.begin(), sorted.end());
        j["median_kkt_residual"] = sorted.empty() ? 0.0 : sorted[sorted.size() / 2];
        j["max_kkt_residual"] = sorted.empty() ? 0.0 : sorted.back();
        return j;
    }
};

double max_difference(const mpc::Solution& a, const mpc::Solution& b) {
    double d = 0.0;
    for (std::size_t k = 0; k < a.x.size(); ++k) {
        d = std::max(d, (a.x[k] - b.x[k]).lpNorm<Eigen::Infinity>());
        d = std::max(d, (a.u[k] - b.u[k]).lpNorm<Eigen::Infinity>());
    }
    return d;
}

json bench_horizon(int horizon, int calls, const Track& track, const Trajectory& reference) {
    const VehicleParams params;
    MpcConfig config;
    config.horizon = horizon;
    MpcController mpc(config, params);
    testing::Plant plant(params, track.start_pose);

    qp::InteriorPointSettings settings;
    settings.tolerance = kTolerance;
    settings.complementarity_tolerance = kTolerance;
    settings.max_iterations = 100;
    qp::InteriorPointSolver<mpc::kStates, mpc::kInputs> solver(settings);
    testing::OsqpOptions osqp_options;
    osqp_options.eps_abs = kTolerance;
    osqp_options.eps_rel = 0.0;
    osqp_options.max_iter = 100000;

    SolverRecord ours_cold;
    SolverRecord ours_warm;
    SolverRecord osqp_cold;
    SolverRecord osqp_warm;
    LatencyStats osqp_solve_only;
    double worst_difference = 0.0;
    mpc::Solution previous;
    Eigen::VectorXd previous_x;
    Eigen::VectorXd previous_y;
    for (int call = 0; call < calls + kWarmup; ++call) {
        const ControlCommand command = mpc.compute(plant.estimate(), reference, plant.time());
        plant.command(command);
        plant.advance(testing::kControlPeriod);
        if (call < kWarmup) continue;
        const mpc::Qp& problem = mpc.last_problem();

        mpc::Solution cold;
        Stopwatch watch;
        const qp::QpStatus status = solver.solve(problem, cold);
        ours_cold.add(watch.elapsed_seconds(), cold.iterations, qp::usable(status),
                      qp::kkt_residuals(problem, cold));
        if (!previous.x.empty() && previous.x.size() == cold.x.size()) {
            mpc::Solution warm;
            watch.restart();
            const qp::QpStatus warm_status = solver.solve(problem, warm, &previous);
            ours_warm.add(watch.elapsed_seconds(), warm.iterations, qp::usable(warm_status),
                          qp::kkt_residuals(problem, warm));
        }

        const testing::SparseQp sparse = testing::to_sparse(problem);
        const testing::OsqpResult osqp = testing::solve_with_osqp(sparse, osqp_options);
        const mpc::Solution osqp_solution = testing::from_sparse(problem, osqp);
        osqp_cold.add(osqp.setup_seconds + osqp.solve_seconds, osqp.iterations, osqp.solved,
                      osqp_solution.residuals);
        osqp_solve_only.add(osqp.solve_seconds);
        if (previous_x.size() == sparse.q.size() && previous_y.size() == sparse.lower.size()) {
            testing::OsqpOptions warm_options = osqp_options;
            warm_options.warm_x = previous_x;
            warm_options.warm_y = previous_y;
            const testing::OsqpResult warm = testing::solve_with_osqp(sparse, warm_options);
            osqp_warm.add(warm.setup_seconds + warm.solve_seconds, warm.iterations, warm.solved,
                          testing::from_sparse(problem, warm).residuals);
        }
        if (qp::usable(status) && osqp.solved) {
            worst_difference = std::max(worst_difference, max_difference(cold, osqp_solution));
        }
        previous = cold;
        previous_x = osqp.x;
        previous_y = osqp.y;
    }
    json j;
    j["horizon"] = horizon;
    j["qps"] = calls;
    j["ours_cold"] = ours_cold.summary();
    j["ours_warm"] = ours_warm.summary();
    j["osqp_cold"] = osqp_cold.summary();
    j["osqp_warm"] = osqp_warm.summary();
    j["osqp_solve_only"] = osqp_solve_only.summary();
    j["max_solution_difference"] = worst_difference;
    j["controller_failures"] = mpc.stats().solver_failures;
    return j;
}

std::string table(const json& results) {
    std::string md =
        "| N | ours cold p50 / p99 (us) | iters | ours warm p50 / p99 | iters | OSQP cold p50 / "
        "p99 | iters | OSQP warm p50 / p99 | iters | OSQP solve-only p50 | max diff |\n"
        "|---|---|---|---|---|---|---|---|---|---|---|\n";
    char row[512];
    for (const json& h : results.at("horizons")) {
        auto cell = [&](const char* key) {
            const json& s = h.at(key);
            char buffer[96];
            std::snprintf(buffer, sizeof(buffer), "%.0f / %.0f | %.1f",
                          s.at("p50_us").get<double>(), s.at("p99_us").get<double>(),
                          s.at("mean_iterations").get<double>());
            return std::string(buffer);
        };
        std::snprintf(row, sizeof(row), "| %d | %s | %s | %s | %s | %.0f | %.1e |\n",
                      h.at("horizon").get<int>(), cell("ours_cold").c_str(),
                      cell("ours_warm").c_str(), cell("osqp_cold").c_str(),
                      cell("osqp_warm").c_str(), h.at("osqp_solve_only").at("p50_us").get<double>(),
                      h.at("max_solution_difference").get<double>());
        md += row;
    }
    return md;
}

}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path out = "results/control";
    int calls = 1000;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string_view flag = argv[i];
        if (flag == "--out") {
            out = argv[i + 1];
        } else if (flag == "--calls") {
            calls = std::atoi(argv[i + 1]);
        } else {
            std::fprintf(stderr, "usage: %s [--out dir] [--calls n]\n", argv[0]);
            return 2;
        }
    }
    const Track track = sim::reference_track("mixed");
    Trajectory reference = track.centerline;
    testing::apply_speed_limits(reference, testing::grip_limited(VehicleParams{}));

    json results;
    results["track"] = "mixed";
    results["tolerance"] = kTolerance;
    results["calls_per_horizon"] = calls;
    for (const int horizon : {20, 30, 40, 50, 60}) {
        std::fprintf(stderr, "horizon %d\n", horizon);
        results["horizons"].push_back(bench_horizon(horizon, calls, track, reference));
    }
    std::filesystem::create_directories(out);
    std::ofstream(out / "qp_bench.json") << results.dump(2) << "\n";
    std::ofstream(out / "qp_bench.md")
        << "# MPC QP: Riccati interior point vs OSQP\n\n"
        << "Every QP of the MPC over " << calls
        << " calls on the 'mixed' track, per horizon N (stages of 50 ms), absolute tolerance "
        << kTolerance
        << ". Times per QP in microseconds; OSQP cold and warm include its setup (the KKT "
           "factorization), which an LTV MPC pays at every call. Max diff: largest difference "
           "between the two solvers' states and inputs.\n\n"
        << table(results);
    std::printf("%s", table(results).c_str());
    return 0;
}
