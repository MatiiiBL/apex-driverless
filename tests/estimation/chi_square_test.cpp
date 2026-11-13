#include "apex/estimation/chi_square.hpp"

#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

namespace apex::estimation {
namespace {

TEST(ChiSquare, CdfOfTwoDegreesIsExponential) {
    // chi^2 with 2 degrees of freedom is exponential with mean 2.
    for (const double x : {0.0, 0.1, 1.0, 5.0, 13.8, 40.0}) {
        EXPECT_NEAR(chi_square_cdf(x, 2), 1.0 - std::exp(-0.5 * x), 1e-14) << x;
    }
}

TEST(ChiSquare, CdfOfOneDegreeIsTheNormalsTwoSidedProbability) {
    for (const double z : {0.5, 1.0, 1.96, 3.0}) {
        EXPECT_NEAR(chi_square_cdf(z * z, 1), std::erf(z / std::sqrt(2.0)), 1e-13) << z;
    }
}

TEST(ChiSquare, QuantilesMatchTables) {
    // Abramowitz and Stegun, table 26.8.
    struct Row {
        double p;
        int dof;
        double x;
    };
    const Row rows[] = {
        {0.95, 1, 3.841459},     {0.975, 1, 5.023886},   {0.95, 2, 5.991465},
        {0.999, 2, 13.815511},   {0.99, 3, 11.344867},   {0.95, 5, 11.070498},
        {0.05, 20, 10.850811},   {0.999, 10, 29.588298}, {0.05, 100, 77.929465},
        {0.95, 100, 124.342113},
    };
    for (const Row& row : rows) {
        EXPECT_NEAR(chi_square_quantile(row.p, row.dof), row.x, 2e-6 * row.x)
            << row.p << " " << row.dof;
    }
}

TEST(ChiSquare, QuantileInvertsTheCdfForLargeDegrees) {
    // The NEES bounds of a whole run need chi^2 with thousands of degrees of freedom.
    for (const int dof : {1, 7, 50, 400, 3000}) {
        for (const double p : {0.025, 0.5, 0.975, 0.99999}) {
            EXPECT_NEAR(chi_square_cdf(chi_square_quantile(p, dof), dof), p, 1e-10)
                << dof << " " << p;
        }
    }
}

TEST(ChiSquare, RejectsInvalidArguments) {
    EXPECT_THROW((void)chi_square_quantile(0.0, 2), std::invalid_argument);
    EXPECT_THROW((void)chi_square_quantile(1.0, 2), std::invalid_argument);
    EXPECT_THROW((void)chi_square_quantile(0.5, 0), std::invalid_argument);
    EXPECT_THROW((void)chi_square_cdf(1.0, 0), std::invalid_argument);
    EXPECT_THROW(ChiSquareTable(1.5), std::invalid_argument);
    EXPECT_EQ(chi_square_cdf(-1.0, 3), 0.0);
}

TEST(ChiSquare, TableCachesTheQuantiles) {
    ChiSquareTable table(0.99);
    EXPECT_DOUBLE_EQ(table(6), chi_square_quantile(0.99, 6));
    EXPECT_DOUBLE_EQ(table(1), chi_square_quantile(0.99, 1));
    EXPECT_DOUBLE_EQ(table(40), chi_square_quantile(0.99, 40));
    EXPECT_THROW((void)table(0), std::invalid_argument);
}

}  // namespace
}  // namespace apex::estimation
