#include "apex/planning/smoothing.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

namespace apex::planning {

std::vector<Vec2> resample_polyline(std::span<const Vec2> points, bool closed, double spacing) {
    if (!(spacing > 0.0)) throw std::invalid_argument("resample_polyline: spacing must be > 0");
    if (points.size() < 2) return {points.begin(), points.end()};
    const std::vector<double> cumulative = cumulative_length(points, closed);
    const double total = cumulative.back();
    if (!(total > 0.0)) return {points.front()};
    const auto intervals =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::round(total / spacing)));
    const double step = total / static_cast<double>(intervals);
    const std::size_t count = closed ? intervals : intervals + 1;
    const std::size_t n = points.size();

    std::vector<Vec2> result;
    result.reserve(count);
    std::size_t segment = 0;
    for (std::size_t k = 0; k < count; ++k) {
        const double s = std::min(step * static_cast<double>(k), total);
        while (segment + 2 < cumulative.size() && cumulative[segment + 1] < s) ++segment;
        const Vec2& a = points[segment];
        const Vec2& b = points[(segment + 1) % n];
        const double length = cumulative[segment + 1] - cumulative[segment];
        const double t = length > 0.0 ? (s - cumulative[segment]) / length : 0.0;
        result.push_back(a + std::clamp(t, 0.0, 1.0) * (b - a));
    }
    return result;
}

std::vector<Vec2> smooth_polyline(std::span<const Vec2> points, const SmoothingOptions& options) {
    const std::size_t n = points.size();
    if (n < (options.closed ? 4U : 3U)) return {points.begin(), points.end()};
    if (!(options.spacing > 0.0) || !(options.cutoff_wavelength > 0.0)) {
        throw std::invalid_argument("smooth_polyline: spacing and cutoff must be > 0");
    }
    const double lambda = std::pow(options.cutoff_wavelength / (kTwoPi * options.spacing), 4);

    // A start direction pins the first point and puts the second one spacing along the
    // direction: both become known values, and the path bends away from the direction no
    // faster than the smoothing allows.
    std::vector<Vec2> result(points.begin(), points.end());
    const bool pinned = !options.closed && options.start_direction.has_value();
    const std::size_t first_unknown = pinned ? 2 : 0;
    if (pinned) result[1] = result[0] + options.spacing * *options.start_direction;
    const auto unknowns = static_cast<Eigen::Index>(n - first_unknown);
    const auto column = [&](std::size_t i) { return static_cast<Eigen::Index>(i - first_unknown); };

    // Normal equations (I + lambda D'D) p = m - lambda D' k, with k the known part of each
    // second difference, one row (p_{i-1}, p_i, p_{i+1}) at a time.
    std::vector<Eigen::Triplet<double>> entries;
    Eigen::MatrixX2d rhs(unknowns, 2);
    for (std::size_t i = first_unknown; i < n; ++i) {
        entries.emplace_back(column(i), column(i), 1.0);
        rhs.row(column(i)) = points[i].transpose();
    }
    const auto add_row = [&](const std::array<std::size_t, 3>& index) {
        constexpr std::array<double, 3> kWeights = {1.0, -2.0, 1.0};
        Vec2 known = Vec2::Zero();
        for (std::size_t a = 0; a < 3; ++a) {
            if (index[a] < first_unknown) known += kWeights[a] * result[index[a]];
        }
        for (std::size_t a = 0; a < 3; ++a) {
            if (index[a] < first_unknown) continue;
            const Eigen::Index row = column(index[a]);
            rhs.row(row) -= lambda * kWeights[a] * known.transpose();
            for (std::size_t b = 0; b < 3; ++b) {
                if (index[b] < first_unknown) continue;
                entries.emplace_back(row, column(index[b]), lambda * kWeights[a] * kWeights[b]);
            }
        }
    };
    if (options.closed) {
        for (std::size_t i = 0; i < n; ++i) add_row({(i + n - 1) % n, i, (i + 1) % n});
    } else {
        for (std::size_t i = 1; i + 1 < n; ++i) add_row({i - 1, i, i + 1});
    }

    Eigen::SparseMatrix<double> normal(unknowns, unknowns);
    normal.setFromTriplets(entries.begin(), entries.end());
    const Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver(normal);
    const Eigen::MatrixX2d solution = solver.solve(rhs);
    for (std::size_t i = first_unknown; i < n; ++i) result[i] = solution.row(column(i)).transpose();
    return result;
}

}  // namespace apex::planning
