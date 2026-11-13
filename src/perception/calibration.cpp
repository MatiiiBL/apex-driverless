#include "apex/perception/calibration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include <Eigen/Dense>

namespace apex::perception {
namespace {

constexpr int kTerms = 4;  // constant, r^2, 1/n, r^2/n: VarianceModel's order
constexpr int kMaxIterations = 100;
constexpr double kTolerance = 1e-10;

Eigen::Vector4d features(const VarianceSample& s) {
    const double r2 = s.range * s.range;
    const double inv_n = 1.0 / static_cast<double>(std::max<std::size_t>(s.points, 1));
    return {1.0, r2, inv_n, r2 * inv_n};
}

// Negative log-likelihood of the errors, up to a constant: sum of log v + e^2 / v. Infinite when
// a variance is not positive.
double negative_log_likelihood(const Eigen::MatrixXd& phi, const Eigen::VectorXd& e2,
                               const Eigen::Vector4d& theta) {
    double sum = 0.0;
    for (Eigen::Index i = 0; i < phi.rows(); ++i) {
        const double v = phi.row(i).dot(theta);
        if (!(v > 0.0)) return std::numeric_limits<double>::infinity();
        sum += std::log(v) + e2[i] / v;
    }
    return sum;
}

// The fit restricted to the terms in `active` (bit k for term k; the constant is always in), or
// nothing when it leaves a coefficient negative or a variance non-positive.
std::optional<Eigen::Vector4d> fit_terms(const Eigen::MatrixXd& phi, const Eigen::VectorXd& e2,
                                         unsigned active) {
    std::vector<Eigen::Index> columns;
    for (int k = 0; k < kTerms; ++k) {
        if (k == 0 || (active & (1U << k)) != 0) columns.push_back(k);
    }
    const auto rows = phi.rows();
    const auto cols = static_cast<Eigen::Index>(columns.size());
    // Columns scaled to unit maximum: r^2 reaches 10^3 and 1/n falls to 10^-3.
    Eigen::MatrixXd a(rows, cols);
    Eigen::VectorXd scale(cols);
    for (Eigen::Index j = 0; j < cols; ++j) {
        a.col(j) = phi.col(columns[static_cast<std::size_t>(j)]);
        scale[j] = a.col(j).cwiseAbs().maxCoeff();
        if (!(scale[j] > 0.0)) return std::nullopt;
        a.col(j) /= scale[j];
    }

    Eigen::VectorXd theta = Eigen::VectorXd::Zero(cols);
    theta[0] = e2.mean() / scale[0];
    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        const Eigen::VectorXd v = a * theta;
        if (!(v.minCoeff() > 0.0)) return std::nullopt;
        const Eigen::VectorXd w = v.cwiseInverse();  // square root of the weights 1 / v^2
        const Eigen::VectorXd next =
            (w.asDiagonal() * a).colPivHouseholderQr().solve(w.cwiseProduct(e2));
        if (!next.allFinite()) return std::nullopt;
        const double change = (next - theta).norm();
        theta = next;
        if (change <= kTolerance * theta.norm()) break;
    }
    if (!(theta.minCoeff() >= 0.0) || !(theta[0] > 0.0)) return std::nullopt;

    Eigen::Vector4d full = Eigen::Vector4d::Zero();
    for (Eigen::Index j = 0; j < cols; ++j) {
        full[columns[static_cast<std::size_t>(j)]] = theta[j] / scale[j];
    }
    return full;
}

}  // namespace

VarianceModel fit_variance_model(std::span<const VarianceSample> samples) {
    if (samples.size() < static_cast<std::size_t>(kTerms)) {
        throw std::invalid_argument("fit_variance_model: needs at least 4 samples");
    }
    const auto n = static_cast<Eigen::Index>(samples.size());
    Eigen::MatrixXd phi(n, kTerms);
    Eigen::VectorXd e2(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        const VarianceSample& s = samples[static_cast<std::size_t>(i)];
        if (!std::isfinite(s.range) || !std::isfinite(s.error)) {
            throw std::invalid_argument("fit_variance_model: non-finite sample");
        }
        phi.row(i) = features(s).transpose();
        e2[i] = s.error * s.error;
    }
    if (!(e2.mean() > 0.0)) throw std::invalid_argument("fit_variance_model: every error is zero");

    std::optional<Eigen::Vector4d> best;
    double best_cost = std::numeric_limits<double>::infinity();
    // Even masks only: bit 0 is the constant, which every fit keeps.
    for (unsigned active = 0; active < (1U << kTerms); active += 2) {
        const std::optional<Eigen::Vector4d> theta = fit_terms(phi, e2, active);
        if (!theta) continue;
        const double cost = negative_log_likelihood(phi, e2, *theta);
        if (cost < best_cost) {
            best_cost = cost;
            best = theta;
        }
    }
    // The constant alone always fits (it is the mean of e^2), so `best` is set.
    Eigen::Vector4d theta = *best;
    double mean_ratio = 0.0;
    for (Eigen::Index i = 0; i < n; ++i) mean_ratio += e2[i] / phi.row(i).dot(theta);
    theta *= mean_ratio / static_cast<double>(n);
    return {theta[0], theta[1], theta[2], theta[3]};
}

}  // namespace apex::perception
