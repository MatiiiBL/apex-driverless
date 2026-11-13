#pragma once

#include <cmath>

#include <Eigen/Core>

namespace apex::control {

// exp(M) for a small fixed-size matrix, by scaling and squaring with a Taylor polynomial
// (C. Moler, C. Van Loan, "Nineteen dubious ways to compute the exponential of a matrix,
// twenty-five years later", SIAM Review 45(1), 2003, methods 1 and 3): M is scaled by 2^-s so
// that its 1-norm is at most 1/2, the degree-8 Taylor polynomial of the scaled matrix is
// evaluated by Horner's rule, and the result squared s times. The truncation error is below
// (1/2)^9 / 9! ~ 5e-9 relative to the exponential, well under what a linearization needs, and
// the scaling keeps it accurate for the stiff lateral dynamics of a car at low speed, where an
// explicit Runge-Kutta step of the same length would be unstable.
template <int N>
[[nodiscard]] Eigen::Matrix<double, N, N> matrix_exponential(const Eigen::Matrix<double, N, N>& m) {
    using Matrix = Eigen::Matrix<double, N, N>;
    constexpr int kDegree = 8;
    const double norm = m.cwiseAbs().colwise().sum().maxCoeff();
    int squarings = 0;
    if (norm > 0.5) squarings = static_cast<int>(std::ceil(std::log2(norm / 0.5)));
    const Matrix scaled = m * std::ldexp(1.0, -squarings);
    Matrix result = Matrix::Identity();
    for (int k = kDegree; k >= 1; --k) {
        Matrix next = Matrix::Identity();
        next.noalias() += (scaled * result) / static_cast<double>(k);
        result = next;
    }
    for (int i = 0; i < squarings; ++i) result = (result * result).eval();
    return result;
}

}  // namespace apex::control
