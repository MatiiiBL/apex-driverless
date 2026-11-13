#include "apex/perception/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace apex::perception {

std::vector<int> solve_assignment(std::span<const double> cost, std::size_t rows, std::size_t cols,
                                  double unmatched_cost) {
    if (cost.size() != rows * cols) {
        throw std::invalid_argument("solve_assignment: cost must have rows * cols entries");
    }
    std::vector<int> result(rows, -1);
    if (rows == 0) return result;

    // Forbidden pairs get a cost no optimal solution would pay: each row's own unmatched slot is
    // always cheaper. Finite, so the potentials never meet inf - inf.
    const double forbidden = (std::abs(unmatched_cost) + 1.0) * static_cast<double>(rows + 1) * 1e6;
    const std::size_t width = cols + rows;
    const auto entry = [&](std::size_t row, std::size_t col) {
        if (col < cols) {
            const double c = cost[row * cols + col];
            return std::isfinite(c) && c < unmatched_cost ? c : forbidden;
        }
        return col - cols == row ? unmatched_cost : forbidden;
    };

    // 1-based arrays: index 0 of the columns is the virtual start of each augmenting path.
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> u(rows + 1, 0.0);
    std::vector<double> v(width + 1, 0.0);
    std::vector<std::size_t> owner(width + 1, 0);  // row matched to each column, 0 for none
    std::vector<std::size_t> way(width + 1, 0);
    std::vector<double> min_slack(width + 1);
    std::vector<char> used(width + 1);
    for (std::size_t row = 1; row <= rows; ++row) {
        owner[0] = row;
        std::size_t col0 = 0;
        std::fill(min_slack.begin(), min_slack.end(), kInf);
        std::fill(used.begin(), used.end(), 0);
        do {
            used[col0] = 1;
            const std::size_t row0 = owner[col0];
            double delta = kInf;
            std::size_t col1 = 0;
            for (std::size_t col = 1; col <= width; ++col) {
                if (used[col] != 0) continue;
                const double slack = entry(row0 - 1, col - 1) - u[row0] - v[col];
                if (slack < min_slack[col]) {
                    min_slack[col] = slack;
                    way[col] = col0;
                }
                if (min_slack[col] < delta) {
                    delta = min_slack[col];
                    col1 = col;
                }
            }
            for (std::size_t col = 0; col <= width; ++col) {
                if (used[col] != 0) {
                    u[owner[col]] += delta;
                    v[col] -= delta;
                } else {
                    min_slack[col] -= delta;
                }
            }
            col0 = col1;
        } while (owner[col0] != 0);
        do {
            const std::size_t col1 = way[col0];
            owner[col0] = owner[col1];
            col0 = col1;
        } while (col0 != 0);
    }

    for (std::size_t col = 1; col <= cols; ++col) {
        if (owner[col] != 0) result[owner[col] - 1] = static_cast<int>(col - 1);
    }
    return result;
}

}  // namespace apex::perception
