// The planning QP solver against OSQP on identical problems (docs/design.md, section 7).
#include "apex/planning/qp.hpp"

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "apex/core/rng.hpp"
#include "apex/core/track.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/planning/planning.hpp"
#include "apex/planning/raceline.hpp"
#include "apex/sim/track_gen.hpp"

#include "osqp_reference.hpp"

namespace apex::planning {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

// A random feasible QP: P = M'M (+ a ridge when strictly convex), bounds around 0 with some
// sides open, and inequality rows that x = 0 satisfies with a random margin.
QpProblem random_qp(Eigen::Index n, Eigen::Index m, Eigen::Index rank, bool ridge, Rng& rng) {
    MatrixXd M(rank, n);
    for (Eigen::Index i = 0; i < rank; ++i) {
        for (Eigen::Index j = 0; j < n; ++j) M(i, j) = rng.normal();
    }
    QpProblem qp;
    qp.P = M.transpose() * M;
    if (ridge) qp.P += 0.01 * MatrixXd::Identity(n, n);
    qp.q = VectorXd(n);
    qp.lower = VectorXd(n);
    qp.upper = VectorXd(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        qp.q[i] = 5.0 * rng.normal();
        qp.lower[i] =
            rng.bernoulli(0.1) ? -std::numeric_limits<double>::infinity() : -rng.uniform(0.2, 2.0);
        qp.upper[i] = rng.uniform(0.2, 2.0);
    }
    qp.G = MatrixXd(m, n);
    qp.h = VectorXd(m);
    for (Eigen::Index k = 0; k < m; ++k) {
        for (Eigen::Index j = 0; j < n; ++j) qp.G(k, j) = rng.normal();
        qp.h[k] = rng.uniform(0.0, 1.0);
    }
    return qp;
}

TEST(QpSolverVsOsqp, RandomStrictlyConvexProblems) {
    Rng rng(42);
    for (int trial = 0; trial < 60; ++trial) {
        const auto n = static_cast<Eigen::Index>(2 + rng.index(40));
        const auto m = static_cast<Eigen::Index>(rng.index(20));
        const QpProblem qp = random_qp(n, m, n, true, rng);
        const QpResult ours = solve_qp(qp);
        const test_support::OsqpResult reference = test_support::solve_with_osqp(qp);
        ASSERT_EQ(ours.status, QpStatus::Solved) << "trial " << trial;
        ASSERT_TRUE(reference.solved) << "trial " << trial;
        EXPECT_NEAR(ours.objective, reference.objective, 1e-6 * (1.0 + std::abs(ours.objective)))
            << "trial " << trial;
        EXPECT_LT((ours.x - reference.x).lpNorm<Eigen::Infinity>(), 1e-5) << "trial " << trial;
        EXPECT_LE(ours.iterations, 30U) << "trial " << trial;
    }
}

// The raceline's own QPs: minimum curvature on the oval and on a generated track, every
// linearization. They are dense and ill-conditioned (offsets on straights barely change the
// curvature), and the curvature bound adds 2n inequality rows.
TEST(QpSolverVsOsqp, RacelineProblems) {
    for (const Track& track : {make_oval_track(), sim::reference_track("technical")}) {
        const TrackModel model{track.centerline, track.left_boundary(), track.right_boundary()};
        RacelineTrace trace;
        (void)optimize_raceline(model, VehicleParams{}, RacelineConfig{}, &trace);
        ASSERT_FALSE(trace.problems.empty());
        for (std::size_t i = 0; i < trace.problems.size(); ++i) {
            const QpResult& ours = trace.results[i];
            const test_support::OsqpResult reference =
                test_support::solve_with_osqp(trace.problems[i].qp);
            ASSERT_EQ(ours.status, QpStatus::Solved) << track.name << " QP " << i;
            ASSERT_TRUE(reference.solved) << track.name << " QP " << i;
            EXPECT_NEAR(ours.objective, reference.objective,
                        1e-6 * (1.0 + std::abs(reference.objective)))
                << track.name << " QP " << i;
            EXPECT_LT((ours.x - reference.x).lpNorm<Eigen::Infinity>(), 1e-4)
                << track.name << " QP " << i;
        }
    }
}

// A rank-deficient P has a set of optimal points: compare the optimal value only.
TEST(QpSolverVsOsqp, RankDeficientProblems) {
    Rng rng(7);
    for (int trial = 0; trial < 30; ++trial) {
        const auto n = static_cast<Eigen::Index>(4 + rng.index(30));
        const auto m = static_cast<Eigen::Index>(rng.index(10));
        const QpProblem qp = random_qp(n, m, n / 2, false, rng);
        const QpResult ours = solve_qp(qp);
        const test_support::OsqpResult reference = test_support::solve_with_osqp(qp);
        ASSERT_EQ(ours.status, QpStatus::Solved) << "trial " << trial;
        ASSERT_TRUE(reference.solved) << "trial " << trial;
        EXPECT_NEAR(ours.objective, reference.objective, 1e-6 * (1.0 + std::abs(ours.objective)))
            << "trial " << trial;
    }
}

}  // namespace
}  // namespace apex::planning
