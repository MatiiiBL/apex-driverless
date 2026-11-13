#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>

// The normalized estimation error squared (NEES) consistency check: for a consistent filter the
// error e of an n-dimensional estimate with covariance P makes e^T P^-1 e chi^2(n) distributed,
// so its mean is n and N independent values average within the chi^2_{nN} / N interval
// (Bar-Shalom, Li and Kirubarajan, "Estimation with Applications to Tracking and Navigation",
// Wiley, 2001, section 5.4.2).
namespace apex::estimation::eval {

// e^T P^-1 e, or +inf if P is not positive definite.
[[nodiscard]] double nees(const Eigen::VectorXd& error, const Eigen::MatrixXd& covariance);

struct NeesBounds {
    double lower = 0.0;
    double upper = 0.0;
};

// Two-sided `probability` interval of the mean of `samples` independent NEES values of `dof`
// dimensions each.
[[nodiscard]] NeesBounds nees_bounds(int dof, std::size_t samples, double probability = 0.95);

struct NeesSummary {
    int dof = 0;
    std::size_t samples = 0;
    double mean = 0.0;
    // Of the samples, the share inside the single-sample interval: about `probability` for a
    // consistent filter, even when successive samples are correlated.
    double inside = 0.0;
    NeesBounds single;  // single-sample interval
};

class NeesAccumulator {
public:
    explicit NeesAccumulator(int dof, double probability = 0.95);
    void add(double value);
    [[nodiscard]] NeesSummary summary() const;
    [[nodiscard]] const std::vector<double>& values() const { return values_; }

private:
    int dof_;
    NeesBounds single_;
    std::vector<double> values_;
};

}  // namespace apex::estimation::eval
