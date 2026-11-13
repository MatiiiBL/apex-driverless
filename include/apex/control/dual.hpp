#pragma once

#include <cmath>

#include <Eigen/Core>

// Forward-mode automatic differentiation with dual numbers: a value and its gradient with
// respect to N inputs, propagated through every arithmetic operation by the chain rule. The MPC
// writes its vehicle model once, as a template on the scalar type, and gets exact Jacobians for
// its linearization by evaluating it on Dual<N> (see L. B. Rall, "Automatic Differentiation:
// Techniques and Applications", Springer LNCS 120, 1981). Only the functions the model uses are
// provided.
namespace apex::control {

template <int N>
struct Dual {
    using Gradient = Eigen::Matrix<double, N, 1>;

    double value = 0.0;
    Gradient grad = Gradient::Zero();

    Dual() = default;
    // Implicit on purpose: a plain number is a constant (zero gradient), so model code written
    // for double compiles unchanged for Dual.
    Dual(double v) : value(v) {}
    Dual(double v, const Gradient& g) : value(v), grad(g) {}

    // The i-th independent variable, with value v.
    [[nodiscard]] static Dual variable(double v, int i) {
        Dual d(v);
        d.grad(i) = 1.0;
        return d;
    }

    Dual& operator+=(const Dual& o) {
        value += o.value;
        grad += o.grad;
        return *this;
    }
    Dual& operator-=(const Dual& o) {
        value -= o.value;
        grad -= o.grad;
        return *this;
    }
    Dual& operator*=(const Dual& o) {
        grad = grad * o.value + value * o.grad;
        value *= o.value;
        return *this;
    }
    Dual& operator/=(const Dual& o) {
        const double inv = 1.0 / o.value;
        grad = (grad - (value * inv) * o.grad) * inv;
        value *= inv;
        return *this;
    }
};

template <int N>
Dual<N> operator-(const Dual<N>& a) {
    return {-a.value, -a.grad};
}
template <int N>
Dual<N> operator+(Dual<N> a, const Dual<N>& b) {
    return a += b;
}
template <int N>
Dual<N> operator-(Dual<N> a, const Dual<N>& b) {
    return a -= b;
}
template <int N>
Dual<N> operator*(Dual<N> a, const Dual<N>& b) {
    return a *= b;
}
template <int N>
Dual<N> operator/(Dual<N> a, const Dual<N>& b) {
    return a /= b;
}
template <int N>
Dual<N> operator+(Dual<N> a, double b) {
    a.value += b;
    return a;
}
template <int N>
Dual<N> operator+(double a, Dual<N> b) {
    b.value += a;
    return b;
}
template <int N>
Dual<N> operator-(Dual<N> a, double b) {
    a.value -= b;
    return a;
}
template <int N>
Dual<N> operator-(double a, const Dual<N>& b) {
    return {a - b.value, -b.grad};
}
template <int N>
Dual<N> operator*(Dual<N> a, double b) {
    a.value *= b;
    a.grad *= b;
    return a;
}
template <int N>
Dual<N> operator*(double a, Dual<N> b) {
    return b * a;
}
template <int N>
Dual<N> operator/(Dual<N> a, double b) {
    return a * (1.0 / b);
}
template <int N>
Dual<N> operator/(double a, const Dual<N>& b) {
    const double inv = 1.0 / b.value;
    return {a * inv, (-a * inv * inv) * b.grad};
}

// Elementary functions: f(a) with gradient f'(a) a.grad.
template <int N>
Dual<N> sin(const Dual<N>& a) {
    return {std::sin(a.value), std::cos(a.value) * a.grad};
}
template <int N>
Dual<N> cos(const Dual<N>& a) {
    return {std::cos(a.value), -std::sin(a.value) * a.grad};
}
template <int N>
Dual<N> tan(const Dual<N>& a) {
    const double t = std::tan(a.value);
    return {t, (1.0 + t * t) * a.grad};
}
template <int N>
Dual<N> atan(const Dual<N>& a) {
    return {std::atan(a.value), a.grad / (1.0 + a.value * a.value)};
}
template <int N>
Dual<N> atan2(const Dual<N>& y, const Dual<N>& x) {
    const double r2 = x.value * x.value + y.value * y.value;
    return {std::atan2(y.value, x.value), (x.value * y.grad - y.value * x.grad) / r2};
}
template <int N>
Dual<N> sqrt(const Dual<N>& a) {
    const double s = std::sqrt(a.value);
    return {s, a.grad * (0.5 / s)};
}
template <int N>
Dual<N> tanh(const Dual<N>& a) {
    const double t = std::tanh(a.value);
    return {t, (1.0 - t * t) * a.grad};
}

// The value of a scalar, for branches (limits, blends) that the derivative does not follow.
[[nodiscard]] inline double value_of(double x) {
    return x;
}
template <int N>
[[nodiscard]] double value_of(const Dual<N>& x) {
    return x.value;
}

}  // namespace apex::control
