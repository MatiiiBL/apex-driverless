#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "apex/control/qp/interior_point.hpp"
#include "apex/control/qp/ocp_qp.hpp"
#include "apex/control/qp/riccati.hpp"
#include "apex/core/rng.hpp"

#include "osqp_reference.hpp"
#include "random_qp.hpp"

namespace apex::control {
namespace {

using qp::InteriorPointSettings;
using qp::InteriorPointSolver;
using qp::OcpQp;
using qp::OcpSolution;
using qp::QpStatus;

template <int Nx, int Nu>
double max_difference(const OcpSolution<Nx, Nu>& a, const OcpSolution<Nx, Nu>& b) {
    double diff = 0.0;
    for (std::size_t k = 0; k < a.x.size(); ++k) {
        diff = std::max(diff, (a.x[k] - b.x[k]).template lpNorm<Eigen::Infinity>());
        diff = std::max(diff, (a.u[k] - b.u[k]).template lpNorm<Eigen::Infinity>());
    }
    return diff;
}

InteriorPointSettings tight_settings() {
    InteriorPointSettings settings;
    settings.tolerance = 1e-9;
    settings.complementarity_tolerance = 1e-9;
    return settings;
}

testing::OsqpOptions tight_osqp() {
    testing::OsqpOptions options;
    options.eps_abs = 1e-10;
    options.polishing = true;
    return options;
}

template <int Nx, int Nu>
void expect_matches_osqp(std::uint64_t seed, std::size_t horizon) {
    Rng rng(seed);
    testing::RandomQpOptions options;
    options.horizon = horizon;
    const OcpQp<Nx, Nu> problem = testing::random_ocp_qp<Nx, Nu>(rng, options);

    InteriorPointSolver<Nx, Nu> solver(tight_settings());
    OcpSolution<Nx, Nu> ours;
    ASSERT_EQ(solver.solve(problem, ours), QpStatus::Solved) << "seed " << seed;

    const testing::OsqpResult osqp =
        testing::solve_with_osqp(testing::to_sparse(problem), tight_osqp());
    ASSERT_TRUE(osqp.solved) << osqp.status;
    const OcpSolution<Nx, Nu> reference = testing::from_sparse(problem, osqp);

    // A constraint that is active with a near-zero multiplier is only pinned to within the
    // complementarity tolerance divided by that multiplier, so the primal comparison is looser
    // than the residuals below.
    EXPECT_LT(max_difference(ours, reference), 2e-5) << "seed " << seed;
    EXPECT_NEAR(ours.objective, reference.objective, 1e-7 * (1.0 + std::abs(ours.objective)));
    // The residuals are recomputed from the problem data, independently of the solver.
    const qp::QpResiduals residuals = qp::kkt_residuals(problem, ours);
    EXPECT_LT(residuals.stationarity, 1e-8);
    EXPECT_LT(residuals.dynamics, 1e-8);
    EXPECT_LT(residuals.inequality, 1e-8);
    EXPECT_LT(residuals.complementarity, 1e-8);
    // The multipliers agree too: the KKT point is unique for a strictly convex QP whose active
    // constraints are linearly independent, which holds for these random problems.
    for (std::size_t k = 0; k < ours.pi.size(); ++k) {
        EXPECT_LT((ours.pi[k] - reference.pi[k]).template lpNorm<Eigen::Infinity>(),
                  1e-4 * (1.0 + reference.pi[k].template lpNorm<Eigen::Infinity>()));
    }
}

TEST(InteriorPoint, MatchesOsqpOnRandomMpcProblems) {
    for (std::uint64_t seed = 1; seed <= 12; ++seed) {
        expect_matches_osqp<4, 2>(seed, 5 + 5 * (seed % 4));
        expect_matches_osqp<6, 3>(100 + seed, 20 + 10 * (seed % 5));
    }
}

TEST(InteriorPoint, SingleInputSystems) {
    for (std::uint64_t seed = 1; seed <= 4; ++seed) expect_matches_osqp<2, 1>(seed, 30);
}

// Without inequalities a single Newton step solves the problem, so this checks the Riccati
// recursion against a dense solve of the KKT system.
TEST(Riccati, MatchesDenseKktSolve) {
    constexpr int Nx = 3;
    constexpr int Nu = 2;
    Rng rng(7);
    testing::RandomQpOptions options;
    options.horizon = 8;
    OcpQp<Nx, Nu> problem = testing::random_ocp_qp<Nx, Nu>(rng, options);
    for (auto& stage : problem.stages) stage.constraints.clear();

    const testing::SparseQp sparse = testing::to_sparse(problem);
    const Eigen::Index n = sparse.P.rows();
    const Eigen::Index m = sparse.A.rows();
    Eigen::MatrixXd kkt = Eigen::MatrixXd::Zero(n + m, n + m);
    kkt.topLeftCorner(n, n) = Eigen::MatrixXd(sparse.P);
    kkt.topRightCorner(n, m) = Eigen::MatrixXd(sparse.A).transpose();
    kkt.bottomLeftCorner(m, n) = Eigen::MatrixXd(sparse.A);
    Eigen::VectorXd rhs(n + m);
    rhs << -sparse.q, sparse.lower;
    const Eigen::VectorXd dense = kkt.partialPivLu().solve(rhs);

    InteriorPointSolver<Nx, Nu> solver;
    OcpSolution<Nx, Nu> solution;
    ASSERT_EQ(solver.solve(problem, solution), QpStatus::Solved);
    EXPECT_LE(solution.iterations, 1);
    Eigen::VectorXd z;
    Eigen::VectorXd y;
    testing::to_sparse_vectors(problem, solution, z, y);
    EXPECT_LT((z - dense.head(n)).lpNorm<Eigen::Infinity>(), 1e-9);
    EXPECT_LT((y - dense.tail(m)).lpNorm<Eigen::Infinity>(), 1e-9);
}

// An MPC solves a sequence of nearby problems; the previous solution should save iterations.
TEST(InteriorPoint, WarmStartFromANearbyProblemSavesIterations) {
    constexpr int Nx = 6;
    constexpr int Nu = 3;
    int cold_total = 0;
    int warm_total = 0;
    for (std::uint64_t seed = 1; seed <= 10; ++seed) {
        Rng rng(seed);
        testing::RandomQpOptions options;
        options.horizon = 30;
        OcpQp<Nx, Nu> problem = testing::random_ocp_qp<Nx, Nu>(rng, options);
        InteriorPointSolver<Nx, Nu> solver;
        OcpSolution<Nx, Nu> first;
        ASSERT_EQ(solver.solve(problem, first), QpStatus::Solved);

        problem.x0 += 0.01 * testing::random_matrix<Nx, 1>(rng, 1.0);
        for (auto& stage : problem.stages)
            stage.q += 0.01 * testing::random_matrix<Nx, 1>(rng, 1.0);
        OcpSolution<Nx, Nu> cold;
        OcpSolution<Nx, Nu> warm;
        ASSERT_EQ(solver.solve(problem, cold), QpStatus::Solved);
        ASSERT_EQ(solver.solve(problem, warm, &first), QpStatus::Solved);
        // Both meet the default 1e-6 tolerance, which leaves the weakly curved directions of a
        // random problem free to ~1e-4; the objectives agree much more closely.
        EXPECT_LT(max_difference(cold, warm), 1e-3);
        EXPECT_NEAR(cold.objective, warm.objective, 1e-7 * (1.0 + std::abs(cold.objective)));
        cold_total += cold.iterations;
        warm_total += warm.iterations;
    }
    EXPECT_LT(warm_total, cold_total) << warm_total << " vs " << cold_total;
}

TEST(InteriorPoint, DetectsInfeasibleConstraints) {
    // A double integrator that must reach x = 10 in two steps with |u| <= 0.1.
    constexpr int Nx = 2;
    constexpr int Nu = 1;
    OcpQp<Nx, Nu> problem;
    problem.stages.resize(6);
    for (auto& stage : problem.stages) {
        stage.A << 1.0, 0.1, 0.0, 1.0;
        stage.B << 0.005, 0.1;
        stage.Q = Eigen::Matrix2d::Identity();
        qp::LinearConstraint<Nx, Nu> input;
        input.d(0) = 1.0;
        input.lower = -0.1;
        input.upper = 0.1;
        stage.constraints.push_back(input);
    }
    qp::LinearConstraint<Nx, Nu> target;
    target.c(0) = 1.0;
    target.lower = 10.0;
    problem.stages[2].constraints.push_back(target);

    InteriorPointSolver<Nx, Nu> solver;
    OcpSolution<Nx, Nu> solution;
    EXPECT_EQ(solver.solve(problem, solution), QpStatus::PrimalInfeasible);
    EXPECT_LT(solution.iterations, 50);

    // Crossed bounds are reported without iterating.
    problem.stages[2].constraints.back().upper = 5.0;
    EXPECT_EQ(solver.solve(problem, solution), QpStatus::PrimalInfeasible);
    EXPECT_EQ(solution.iterations, 0);
}

TEST(InteriorPoint, RejectsInvalidProblems) {
    InteriorPointSolver<2, 1> solver;
    OcpSolution<2, 1> solution;
    OcpQp<2, 1> empty;
    EXPECT_EQ(solver.solve(empty, solution), QpStatus::InvalidProblem);

    OcpQp<2, 1> problem;
    problem.stages.resize(3);
    problem.stages[1].Q(0, 0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(solver.solve(problem, solution), QpStatus::InvalidProblem);
    EXPECT_TRUE(solution.x.empty());
}

TEST(InteriorPoint, ReportsNonConvexInputCost) {
    OcpQp<2, 1> problem;
    problem.stages.resize(3);
    problem.stages[1].R(0, 0) = -1.0;
    // A non-zero gradient, so that the zero initial iterate is not already optimal.
    problem.stages[1].r(0) = 1.0;
    InteriorPointSolver<2, 1> solver;
    OcpSolution<2, 1> solution;
    EXPECT_EQ(solver.solve(problem, solution), QpStatus::NumericalError);
}

TEST(InteriorPoint, IterationLimit) {
    Rng rng(3);
    const OcpQp<6, 3> problem = testing::random_ocp_qp<6, 3>(rng, {});
    InteriorPointSettings settings;
    settings.max_iterations = 2;
    InteriorPointSolver<6, 3> solver(settings);
    OcpSolution<6, 3> solution;
    const QpStatus status = solver.solve(problem, solution);
    EXPECT_EQ(status, QpStatus::MaxIterations);
    EXPECT_FALSE(qp::usable(status));
    EXPECT_EQ(solution.iterations, 2);
    for (const auto& u : solution.u) {
        EXPECT_TRUE(u.allFinite());
    }
}

TEST(InteriorPoint, Deterministic) {
    Rng rng(11);
    const OcpQp<6, 3> problem = testing::random_ocp_qp<6, 3>(rng, {});
    InteriorPointSolver<6, 3> a;
    InteriorPointSolver<6, 3> b;
    OcpSolution<6, 3> first;
    OcpSolution<6, 3> second;
    ASSERT_EQ(a.solve(problem, first), QpStatus::Solved);
    ASSERT_EQ(b.solve(problem, second), QpStatus::Solved);
    ASSERT_EQ(first.iterations, second.iterations);
    for (std::size_t k = 0; k < first.u.size(); ++k) {
        EXPECT_EQ(first.u[k], second.u[k]);
        EXPECT_EQ(first.x[k], second.x[k]);
    }
}

}  // namespace
}  // namespace apex::control
