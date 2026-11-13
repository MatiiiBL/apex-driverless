#pragma once

#include <cstddef>
#include <span>
#include <vector>

namespace apex::perception {

// Minimum-cost matching of rows to columns in which a row may also stay unmatched, at cost
// `unmatched_cost`. `cost` is row-major, rows x cols; an entry that is not finite or not below
// `unmatched_cost` is never matched (it is outside the gate). Returns, for each row, its column
// or -1. Each column is used at most once.
//
// The Hungarian method on the rows x (cols + rows) matrix whose extra columns are one private
// "unmatched" slot per row, in the O(n^2 m) shortest-augmenting-path form with dual potentials:
//   H. W. Kuhn, "The Hungarian method for the assignment problem", Naval Research Logistics
//   Quarterly 2, 1955.
//   R. Jonker and A. Volgenant, "A shortest augmenting path algorithm for dense and sparse
//   linear assignment problems", Computing 38, 1987.
// Deterministic: ties go to the lower column index. Throws std::invalid_argument if `cost` does
// not have rows * cols entries.
[[nodiscard]] std::vector<int> solve_assignment(std::span<const double> cost, std::size_t rows,
                                                std::size_t cols, double unmatched_cost);

}  // namespace apex::perception
