#pragma once

#include <vector>

// Chi-square distribution functions for gating and consistency checks. A Mahalanobis distance
// d^2 = nu^T S^-1 nu of a k-dimensional Gaussian innovation follows chi^2 with k degrees of
// freedom, so its quantiles are the gate thresholds (Bar-Shalom, Li and Kirubarajan, "Estimation
// with Applications to Tracking and Navigation", Wiley, 2001, section 1.4.17).
namespace apex::estimation {

// P(X <= x) for X ~ chi^2(dof), dof >= 1.
[[nodiscard]] double chi_square_cdf(double x, int dof);

// The x with chi_square_cdf(x, dof) = probability, for probability in (0, 1). Throws
// std::invalid_argument outside that range or for dof < 1.
[[nodiscard]] double chi_square_quantile(double probability, int dof);

// Quantiles of one probability for 1, 2, ... degrees of freedom, computed once and cached: the
// joint compatibility test needs chi^2_{2k} for every hypothesis size k.
class ChiSquareTable {
public:
    explicit ChiSquareTable(double probability);
    [[nodiscard]] double probability() const { return probability_; }
    // chi^2_dof quantile; extends the cache on first use of a larger dof.
    [[nodiscard]] double operator()(int dof);

private:
    double probability_;
    std::vector<double> quantiles_;  // [dof - 1]
};

}  // namespace apex::estimation
