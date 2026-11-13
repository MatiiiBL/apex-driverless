#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include "apex/core/rng.hpp"
#include "apex/planning/qp.hpp"

namespace apex::planning {
namespace {

using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr double kInf = std::numeric_limits<double>::infinity();

MatrixXd random_spd(Eigen::Index n, Rng& rng) {
    MatrixXd M(n, n);
    for (Eigen::Index i = 0; i < n; ++i) {
        for (Eigen::Index j = 0; j < n; ++j) M(i, j) = rng.normal();
    }
    return M.transpose() * M + 0.1 * MatrixXd::Identity(n, n);
}

TEST(QpSolver, UnconstrainedIsTheLinearSolve) {
    Rng rng(1);
    QpProblem qp;
    qp.P = random_spd(8, rng);
    qp.q = VectorXd(8);
    for (Eigen::Index i = 0; i < 8; ++i) qp.q[i] = rng.normal();
    const QpResult result = solve_qp(qp);
    ASSERT_EQ(result.status, QpStatus::Solved);
    const VectorXd expected = qp.P.ldlt().solve(-qp.q);
    EXPECT_LT((result.x - expected).lpNorm<Eigen::Infinity>(), 1e-9);
    EXPECT_LE(result.iterations, 2U);
}

// min 1/2 |x - c|^2 over a box is the projection of c onto the box.
TEST(QpSolver, SeparableQuadraticIsTheProjectionOntoTheBox) {
    const Eigen::Index n = 6;
    QpProblem qp;
    qp.P = MatrixXd::Identity(n, n);
    VectorXd c(n);
    c << -3.0, -0.5, 0.0, 0.7, 2.0, 9.0;
    qp.q = -c;
    qp.lower = VectorXd::Constant(n, -1.0);
    qp.upper = VectorXd::Constant(n, 1.0);
    qp.upper[5] = kInf;  // one side unbounded
    const QpResult result = solve_qp(qp);
    ASSERT_EQ(result.status, QpStatus::Solved);
    VectorXd expected(n);
    expected << -1.0, -0.5, 0.0, 0.7, 1.0, 9.0;
    EXPECT_LT((result.x - expected).lpNorm<Eigen::Infinity>(), 1e-7);
}

// With P = 0 the problem is a linear program over the box: every variable goes to the bound its
// cost pushes it to.
TEST(QpSolver, LinearObjectiveGoesToTheCorner) {
    const Eigen::Index n = 5;
    QpProblem qp;
    qp.P = MatrixXd::Zero(n, n);
    qp.q = VectorXd(n);
    qp.q << 1.0, -2.0, 0.5, -0.1, 3.0;
    qp.lower = VectorXd::Constant(n, -2.0);
    qp.upper = VectorXd::Constant(n, 3.0);
    const QpResult result = solve_qp(qp);
    ASSERT_EQ(result.status, QpStatus::Solved);
    for (Eigen::Index i = 0; i < n; ++i) {
        EXPECT_NEAR(result.x[i], qp.q[i] > 0.0 ? -2.0 : 3.0, 1e-7);
    }
}

// The projection of (cx, cy) onto {x + y <= 1, y >= 0}: min (x - cx)^2 + (y - cy)^2.
QpProblem projection_onto_wedge(double cx, double cy) {
    QpProblem qp;
    qp.P = 2.0 * MatrixXd::Identity(2, 2);
    qp.q = VectorXd(2);
    qp.q << -2.0 * cx, -2.0 * cy;
    qp.lower = VectorXd(2);
    qp.lower << -kInf, 0.0;
    qp.G = MatrixXd::Ones(1, 2);
    qp.h = VectorXd::Ones(1);
    return qp;
}

// (3, -1) projects onto the corner (1, 0), where both constraints are active with multipliers
// 4 and 6: a nondegenerate solution, found to full precision.
TEST(QpSolver, InequalityRowsAndBoundsTogether) {
    const QpResult result = solve_qp(projection_onto_wedge(3.0, -1.0));
    ASSERT_EQ(result.status, QpStatus::Solved);
    EXPECT_NEAR(result.x[0], 1.0, 1e-8);
    EXPECT_NEAR(result.x[1], 0.0, 1e-8);
    EXPECT_NEAR(result.objective, 1.0 - 6.0, 1e-8);  // x^2 + y^2 - 6 x + 2 y at (1, 0)
}

// (3, 2) also projects onto (1, 0), but y >= 0 is active with a zero multiplier. There an
// interior-point method approaches the solution only like the square root of the duality gap
// (slack and multiplier both shrink like sqrt(mu)), so the point is precise to about
// sqrt(tolerance) while the objective keeps full precision.
TEST(QpSolver, DegenerateActiveConstraintKeepsTheObjectivePrecise) {
    const QpResult result = solve_qp(projection_onto_wedge(3.0, 2.0));
    ASSERT_EQ(result.status, QpStatus::Solved);
    EXPECT_NEAR(result.x[0], 1.0, 1e-4);
    EXPECT_NEAR(result.x[1], 0.0, 1e-4);
    EXPECT_NEAR(result.objective, 1.0 - 6.0, 1e-8);  // x^2 + y^2 - 6 x - 4 y at (1, 0)
}

TEST(QpSolver, InvalidBoundsAreReported) {
    QpProblem qp;
    qp.P = MatrixXd::Identity(2, 2);
    qp.q = VectorXd::Zero(2);
    qp.lower = VectorXd::Constant(2, 1.0);
    qp.upper = VectorXd::Constant(2, 0.0);
    EXPECT_EQ(solve_qp(qp).status, QpStatus::InvalidBounds);
}

TEST(QpSolver, InfeasibleRowsDoNotConverge) {
    QpProblem qp;
    qp.P = MatrixXd::Identity(1, 1);
    qp.q = VectorXd::Zero(1);
    qp.lower = VectorXd::Constant(1, 2.0);
    qp.G = MatrixXd::Ones(1, 1);
    qp.h = VectorXd::Constant(1, 1.0);  // x <= 1 and x >= 2
    EXPECT_EQ(solve_qp(qp).status, QpStatus::NotConverged);
}

TEST(QpSolver, DimensionMismatchThrows) {
    QpProblem qp;
    qp.P = MatrixXd::Identity(3, 3);
    qp.q = VectorXd::Zero(2);
    EXPECT_THROW((void)solve_qp(qp), std::invalid_argument);
    qp.q = VectorXd::Zero(3);
    qp.G = MatrixXd::Ones(2, 3);
    qp.h = VectorXd::Zero(1);
    EXPECT_THROW((void)solve_qp(qp), std::invalid_argument);
}

}  // namespace
}  // namespace apex::planning
