#include "apex/estimation/eval/consistency.hpp"

#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>

#include "apex/estimation/chi_square.hpp"

namespace apex::estimation::eval {

double nees(const Eigen::VectorXd& error, const Eigen::MatrixXd& covariance) {
    const Eigen::LLT<Eigen::MatrixXd> llt(covariance);
    if (llt.info() != Eigen::Success) return std::numeric_limits<double>::infinity();
    return error.dot(llt.solve(error));
}

NeesBounds nees_bounds(int dof, std::size_t samples, double probability) {
    if (dof < 1 || samples < 1) throw std::invalid_argument("nees_bounds: empty");
    const double tail = 0.5 * (1.0 - probability);
    const int total = dof * static_cast<int>(samples);
    const auto n = static_cast<double>(samples);
    return {chi_square_quantile(tail, total) / n, chi_square_quantile(1.0 - tail, total) / n};
}

NeesAccumulator::NeesAccumulator(int dof, double probability)
    : dof_(dof), single_(nees_bounds(dof, 1, probability)) {}

void NeesAccumulator::add(double value) {
    values_.push_back(value);
}

NeesSummary NeesAccumulator::summary() const {
    NeesSummary s;
    s.dof = dof_;
    s.samples = values_.size();
    s.single = single_;
    if (values_.empty()) return s;
    std::size_t inside = 0;
    double sum = 0.0;
    for (const double v : values_) {
        sum += v;
        if (v >= single_.lower && v <= single_.upper) ++inside;
    }
    const auto n = static_cast<double>(values_.size());
    s.mean = sum / n;
    s.inside = static_cast<double>(inside) / n;
    return s;
}

}  // namespace apex::estimation::eval
