// The planning QP solver against OSQP, quoted in docs/planning.md ("QP solver"). Release:
//
//   make PRESET=release build
//   ./build/release/bench/planning/apex_planning_qp_bench --out results/planning/qp.json
//
// Two problem sets, each solved by both solvers to 1e-9 (OSQP with solution polishing, its
// setup and factorization included in its time): every QP that optimize_raceline solves on 10
// generated tracks, and random dense QPs of growing size with half as many inequality rows as
// variables. Reports wall-clock time (median of repeated solves), iterations, and how far the
// two solutions are apart.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/rng.hpp"
#include "apex/core/timing.hpp"
#include "apex/planning/qp.hpp"
#include "apex/planning/raceline.hpp"
#include "apex/sim/track_gen.hpp"

#include "osqp_reference.hpp"

namespace {

using apex::planning::QpProblem;
using apex::planning::QpResult;
using apex::planning::test_support::OsqpResult;
using nlohmann::json;

// Median wall-clock seconds of `repeats` calls of f.
template <class F>
double median_seconds(int repeats, F&& f) {
    std::vector<double> times;
    for (int i = 0; i < repeats; ++i) {
        const apex::Stopwatch watch;
        f();
        times.push_back(watch.elapsed_seconds());
    }
    std::sort(times.begin(), times.end());
    return times[times.size() / 2];
}

struct Comparison {
    json rows = json::array();
    double worst_objective_gap = 0.0;  // relative
    double worst_solution_gap = 0.0;   // infinity norm
    std::size_t ours_unsolved = 0;
    std::size_t osqp_unsolved = 0;
};

void compare(const std::string& name, const QpProblem& qp, int repeats, Comparison& out) {
    QpResult ours;
    OsqpResult reference;
    const double ours_s = median_seconds(repeats, [&] { ours = apex::planning::solve_qp(qp); });
    const double osqp_s = median_seconds(
        repeats, [&] { reference = apex::planning::test_support::solve_with_osqp(qp); });
    if (ours.status != apex::planning::QpStatus::Solved) ++out.ours_unsolved;
    if (!reference.solved) ++out.osqp_unsolved;
    const double objective_gap =
        std::abs(ours.objective - reference.objective) / (1.0 + std::abs(reference.objective));
    const double solution_gap = (ours.x - reference.x).lpNorm<Eigen::Infinity>();
    out.worst_objective_gap = std::max(out.worst_objective_gap, objective_gap);
    out.worst_solution_gap = std::max(out.worst_solution_gap, solution_gap);
    out.rows.push_back({{"problem", name},
                        {"variables", qp.q.size()},
                        {"rows", qp.G.rows()},
                        {"ours_ms", 1e3 * ours_s},
                        {"ours_iterations", ours.iterations},
                        {"osqp_ms", 1e3 * osqp_s},
                        {"osqp_iterations", reference.iterations},
                        {"objective_gap", objective_gap},
                        {"solution_gap", solution_gap}});
}

json summary(const Comparison& c) {
    return {{"worst_relative_objective_gap", c.worst_objective_gap},
            {"worst_solution_gap", c.worst_solution_gap},
            {"ours_unsolved", c.ours_unsolved},
            {"osqp_unsolved", c.osqp_unsolved},
            {"problems", c.rows}};
}

QpProblem random_qp(Eigen::Index n, apex::Rng& rng) {
    const Eigen::Index m = n / 2;
    Eigen::MatrixXd M(n, n);
    for (Eigen::Index i = 0; i < n; ++i) {
        for (Eigen::Index j = 0; j < n; ++j) M(i, j) = rng.normal();
    }
    QpProblem qp;
    qp.P = M.transpose() * M + 0.01 * Eigen::MatrixXd::Identity(n, n);
    qp.q = Eigen::VectorXd(n);
    qp.lower = Eigen::VectorXd(n);
    qp.upper = Eigen::VectorXd(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        qp.q[i] = 5.0 * rng.normal();
        qp.lower[i] = -rng.uniform(0.2, 2.0);
        qp.upper[i] = rng.uniform(0.2, 2.0);
    }
    qp.G = Eigen::MatrixXd(m, n);
    qp.h = Eigen::VectorXd(m);
    for (Eigen::Index k = 0; k < m; ++k) {
        for (Eigen::Index j = 0; j < n; ++j) qp.G(k, j) = rng.normal();
        qp.h[k] = rng.uniform(0.0, 1.0);
    }
    return qp;
}

}  // namespace

int main(int argc, char** argv) {
    std::string out_path;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (std::string_view(argv[i]) == "--out") out_path = argv[i + 1];
    }

    Comparison raceline;
    for (std::uint64_t seed = 1; seed <= 10; ++seed) {
        const apex::Track track = apex::sim::generate_track(seed);
        const apex::planning::TrackModel model{track.centerline, track.left_boundary(),
                                               track.right_boundary()};
        apex::planning::RacelineTrace trace;
        (void)apex::planning::optimize_raceline(model, apex::VehicleParams{}, {}, &trace);
        for (std::size_t i = 0; i < trace.problems.size(); ++i) {
            compare("generated-" + std::to_string(seed) + " QP " + std::to_string(i),
                    trace.problems[i].qp, 5, raceline);
        }
    }

    Comparison random;
    apex::Rng rng(11);
    for (const Eigen::Index n : {25, 50, 100, 200, 400}) {
        compare("random n=" + std::to_string(n), random_qp(n, rng), n >= 200 ? 3 : 9, random);
    }

    const json result = {
        {"tolerance", 1e-9}, {"raceline", summary(raceline)}, {"random", summary(random)}};
    std::cout << result.dump(2) << '\n';
    if (!out_path.empty()) std::ofstream(out_path) << result.dump(1) << '\n';
    return 0;
}
