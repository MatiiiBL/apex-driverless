#include "apex/perception/assignment.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/rng.hpp"

namespace apex::perception {
namespace {

// Cost of a matching under the convention of solve_assignment.
double total_cost(const std::vector<double>& cost, std::size_t cols, const std::vector<int>& match,
                  double unmatched) {
    double total = 0.0;
    for (std::size_t row = 0; row < match.size(); ++row) {
        total +=
            match[row] < 0 ? unmatched : cost[row * cols + static_cast<std::size_t>(match[row])];
    }
    return total;
}

// The reference: every assignment of rows to distinct columns or to nothing, enumerated.
double brute_force(const std::vector<double>& cost, std::size_t rows, std::size_t cols,
                   double unmatched, std::size_t row, std::vector<char>& used) {
    if (row == rows) return 0.0;
    double best = unmatched + brute_force(cost, rows, cols, unmatched, row + 1, used);
    for (std::size_t col = 0; col < cols; ++col) {
        const double c = cost[row * cols + col];
        if (used[col] != 0 || !std::isfinite(c) || c >= unmatched) continue;
        used[col] = 1;
        best = std::min(best, c + brute_force(cost, rows, cols, unmatched, row + 1, used));
        used[col] = 0;
    }
    return best;
}

TEST(SolveAssignment, MatchesBruteForceOnRandomProblems) {
    Rng rng(21);
    for (int trial = 0; trial < 400; ++trial) {
        const std::size_t rows = rng.index(6);
        const std::size_t cols = rng.index(6);
        const double unmatched = rng.uniform(1.0, 10.0);
        std::vector<double> cost(rows * cols);
        for (double& c : cost) {
            c = rng.bernoulli(0.1) ? std::numeric_limits<double>::infinity()
                                   : rng.uniform(0.0, 12.0);
        }
        const std::vector<int> match = solve_assignment(cost, rows, cols, unmatched);
        ASSERT_EQ(match.size(), rows);
        std::vector<char> used(cols, 0);
        for (std::size_t row = 0; row < rows; ++row) {
            if (match[row] < 0) continue;
            const auto col = static_cast<std::size_t>(match[row]);
            ASSERT_LT(col, cols);
            EXPECT_EQ(used[col], 0) << "column used twice";
            used[col] = 1;
            EXPECT_LT(cost[row * cols + col], unmatched) << "matched outside the gate";
        }
        std::vector<char> scratch(cols, 0);
        EXPECT_NEAR(total_cost(cost, cols, match, unmatched),
                    brute_force(cost, rows, cols, unmatched, 0, scratch), 1e-9)
            << "trial " << trial << ": " << rows << " x " << cols;
    }
}

TEST(SolveAssignment, PrefersTheGlobalOptimumOverGreedy) {
    // Greedy would give row 0 its cheapest column (0) and leave row 1 unmatched.
    const std::vector<double> cost = {1.0, 2.0,  //
                                      1.5, 100.0};
    const std::vector<int> match = solve_assignment(cost, 2, 2, 5.0);
    EXPECT_EQ(match[0], 1);
    EXPECT_EQ(match[1], 0);
}

TEST(SolveAssignment, HandlesEmptyAndOneSidedProblems) {
    EXPECT_TRUE(solve_assignment({}, 0, 3, 1.0).empty());
    EXPECT_EQ(solve_assignment({}, 2, 0, 1.0), (std::vector<int>{-1, -1}));
    EXPECT_THROW((void)solve_assignment(std::vector<double>(3), 2, 2, 1.0), std::invalid_argument);
}

}  // namespace
}  // namespace apex::perception
