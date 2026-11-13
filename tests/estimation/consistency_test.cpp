#include "apex/estimation/eval/consistency.hpp"

#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <gtest/gtest.h>

#include "apex/core/rng.hpp"
#include "apex/estimation/chi_square.hpp"

namespace apex::estimation::eval {
namespace {

TEST(Nees, IsTheMahalanobisDistance) {
    Eigen::MatrixXd p(2, 2);
    p << 4.0, 1.0, 1.0, 2.0;
    Eigen::VectorXd e(2);
    e << 1.0, -1.0;
    EXPECT_NEAR(nees(e, p), e.dot(p.inverse() * e), 1e-12);
}

TEST(Nees, IsInfiniteForACovarianceThatIsNotPositiveDefinite) {
    Eigen::MatrixXd p(2, 2);
    p << 1.0, 2.0, 2.0, 1.0;
    EXPECT_EQ(nees(Eigen::VectorXd::Ones(2), p), std::numeric_limits<double>::infinity());
}

TEST(Nees, BoundsAreTheChiSquareInterval) {
    const NeesBounds one = nees_bounds(3, 1);
    EXPECT_NEAR(one.lower, chi_square_quantile(0.025, 3), 1e-12);
    EXPECT_NEAR(one.upper, chi_square_quantile(0.975, 3), 1e-12);
    // The mean of N samples: chi^2_{3N} / N, which narrows around 3.
    const NeesBounds many = nees_bounds(3, 100);
    EXPECT_NEAR(many.lower, chi_square_quantile(0.025, 300) / 100.0, 1e-12);
    EXPECT_GT(many.lower, 2.5);
    EXPECT_LT(many.upper, 3.5);
    EXPECT_THROW((void)nees_bounds(0, 10), std::invalid_argument);
    EXPECT_THROW((void)nees_bounds(2, 0), std::invalid_argument);
}

TEST(Nees, AccumulatorOfAConsistentEstimateAveragesTheDimension) {
    // Errors drawn from the covariance the estimate claims.
    Eigen::MatrixXd p(2, 2);
    p << 0.5, 0.2, 0.2, 0.3;
    const Eigen::MatrixXd l = p.llt().matrixL();
    Rng rng(7);
    NeesAccumulator accumulator(2);
    for (int i = 0; i < 4000; ++i) {
        const Eigen::VectorXd e = l * Eigen::Vector2d(rng.normal(), rng.normal());
        accumulator.add(nees(e, p));
    }
    const NeesSummary s = accumulator.summary();
    EXPECT_EQ(s.samples, 4000U);
    const NeesBounds mean_bounds = nees_bounds(2, s.samples);
    EXPECT_GT(s.mean, mean_bounds.lower);
    EXPECT_LT(s.mean, mean_bounds.upper);
    EXPECT_NEAR(s.inside, 0.95, 0.015);
}

TEST(Nees, AccumulatorFlagsAnOverconfidentEstimate) {
    Rng rng(8);
    NeesAccumulator accumulator(1);
    Eigen::MatrixXd claimed(1, 1);
    claimed << 0.25;  // the true variance is 1
    for (int i = 0; i < 1000; ++i) {
        accumulator.add(nees(Eigen::VectorXd::Constant(1, rng.normal()), claimed));
    }
    const NeesSummary s = accumulator.summary();
    EXPECT_GT(s.mean, nees_bounds(1, s.samples).upper);
    EXPECT_LT(s.inside, 0.8);
}

}  // namespace
}  // namespace apex::estimation::eval
