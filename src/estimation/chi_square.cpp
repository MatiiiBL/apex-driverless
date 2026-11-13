#include "apex/estimation/chi_square.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace apex::estimation {

namespace {

// log Gamma(dof / 2), exact for the integer and half-integer arguments chi^2 needs. Avoids
// std::lgamma, which writes the global signgam on some C libraries.
double log_gamma_half(int dof) {
    if (dof % 2 == 0) {
        double sum = 0.0;  // Gamma(n) = (n - 1)!
        for (int i = 2; i < dof / 2; ++i) sum += std::log(static_cast<double>(i));
        return sum;
    }
    double sum = 0.5 * std::log(std::numbers::pi);  // Gamma(n + 1/2) = sqrt(pi) prod (i + 1/2)
    for (int i = 0; i < dof / 2; ++i) sum += std::log(static_cast<double>(i) + 0.5);
    return sum;
}

// Regularized lower incomplete gamma P(a, x) with a = dof / 2: by its power series below
// x = a + 1, and as 1 - Q(a, x) from the continued fraction of Q evaluated with the modified
// Lentz method above (Press, Teukolsky, Vetterling and Flannery, "Numerical Recipes", 3rd ed.,
// Cambridge University Press, 2007, section 6.2).
double regularized_gamma_p(int dof, double x) {
    const double a = 0.5 * dof;
    if (x <= 0.0) return 0.0;
    const double log_prefactor = -x + a * std::log(x) - log_gamma_half(dof);
    constexpr double kEpsilon = 1e-16;
    constexpr int kMaxIterations = 10000;
    if (x < a + 1.0) {
        double term = 1.0 / a;
        double sum = term;
        for (int n = 1; n < kMaxIterations; ++n) {
            term *= x / (a + n);
            sum += term;
            if (std::abs(term) < std::abs(sum) * kEpsilon) break;
        }
        return std::min(1.0, sum * std::exp(log_prefactor));
    }
    constexpr double kTiny = std::numeric_limits<double>::min() / kEpsilon;
    double b = x + 1.0 - a;
    double c = 1.0 / kTiny;
    double d = 1.0 / b;
    double h = d;
    for (int i = 1; i < kMaxIterations; ++i) {
        const double an = -i * (i - a);
        b += 2.0;
        d = an * d + b;
        if (std::abs(d) < kTiny) d = kTiny;
        c = b + an / c;
        if (std::abs(c) < kTiny) c = kTiny;
        d = 1.0 / d;
        const double delta = d * c;
        h *= delta;
        if (std::abs(delta - 1.0) < kEpsilon) break;
    }
    return std::max(0.0, 1.0 - std::exp(log_prefactor) * h);
}

}  // namespace

double chi_square_cdf(double x, int dof) {
    if (dof < 1) throw std::invalid_argument("chi_square_cdf: dof must be at least 1");
    return regularized_gamma_p(dof, 0.5 * x);
}

double chi_square_quantile(double probability, int dof) {
    if (!(probability > 0.0 && probability < 1.0)) {
        throw std::invalid_argument("chi_square_quantile: probability must be in (0, 1)");
    }
    if (dof < 1) throw std::invalid_argument("chi_square_quantile: dof must be at least 1");
    double lo = 0.0;
    double hi = std::max(1.0, static_cast<double>(dof));
    while (chi_square_cdf(hi, dof) < probability) {
        lo = hi;
        hi *= 2.0;
    }
    // The cdf is monotone: bisection to full double precision is robust and fast enough for a
    // value that is computed once and cached.
    for (int i = 0; i < 200 && hi - lo > 1e-14 * hi; ++i) {
        const double mid = 0.5 * (lo + hi);
        (chi_square_cdf(mid, dof) < probability ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

ChiSquareTable::ChiSquareTable(double probability) : probability_(probability) {
    if (!(probability > 0.0 && probability < 1.0)) {
        throw std::invalid_argument("ChiSquareTable: probability must be in (0, 1)");
    }
}

double ChiSquareTable::operator()(int dof) {
    if (dof < 1) throw std::invalid_argument("ChiSquareTable: dof must be at least 1");
    const auto index = static_cast<std::size_t>(dof - 1);
    while (quantiles_.size() <= index) {
        quantiles_.push_back(
            chi_square_quantile(probability_, static_cast<int>(quantiles_.size()) + 1));
    }
    return quantiles_[index];
}

}  // namespace apex::estimation
