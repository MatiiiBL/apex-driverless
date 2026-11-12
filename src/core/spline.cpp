#include "apex/core/spline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace apex {

namespace {

// Solves a tridiagonal system with the Thomas algorithm. sub[0] and super[n-1] are unused.
std::vector<double> solve_tridiagonal(const std::vector<double>& sub, std::vector<double> diag,
                                      const std::vector<double>& super, std::vector<double> rhs) {
    const std::size_t n = diag.size();
    for (std::size_t i = 1; i < n; ++i) {
        const double w = sub[i] / diag[i - 1];
        diag[i] -= w * super[i - 1];
        rhs[i] -= w * rhs[i - 1];
    }
    std::vector<double> x(n);
    x[n - 1] = rhs[n - 1] / diag[n - 1];
    for (std::size_t i = n - 1; i-- > 0;) x[i] = (rhs[i] - super[i] * x[i + 1]) / diag[i];
    return x;
}

// Solves a cyclic tridiagonal system (sub[0] couples x[0] to x[n-1], super[n-1] couples x[n-1]
// to x[0]) with the Sherman-Morrison correction of the plain tridiagonal solve.
std::vector<double> solve_cyclic_tridiagonal(const std::vector<double>& sub,
                                             const std::vector<double>& diag,
                                             const std::vector<double>& super,
                                             const std::vector<double>& rhs) {
    const std::size_t n = diag.size();
    const double alpha = super[n - 1];  // A[n-1][0]
    const double beta = sub[0];         // A[0][n-1]
    const double gamma = -diag[0];
    std::vector<double> modified = diag;
    modified[0] -= gamma;
    modified[n - 1] -= alpha * beta / gamma;
    const std::vector<double> x = solve_tridiagonal(sub, modified, super, rhs);
    std::vector<double> u(n, 0.0);
    u[0] = gamma;
    u[n - 1] = alpha;
    const std::vector<double> z = solve_tridiagonal(sub, modified, super, u);
    const double factor = (x[0] + beta * x[n - 1] / gamma) / (1.0 + z[0] + beta * z[n - 1] / gamma);
    std::vector<double> result(n);
    for (std::size_t i = 0; i < n; ++i) result[i] = x[i] - factor * z[i];
    return result;
}

// 5-point Gauss-Legendre nodes and weights on [-1, 1].
constexpr std::array<double, 5> kGaussNodes = {-0.9061798459386640, -0.5384693101056831, 0.0,
                                               0.5384693101056831, 0.9061798459386640};
constexpr std::array<double, 5> kGaussWeights = {0.2369268850561891, 0.4786286704993665,
                                                 0.5688888888888889, 0.4786286704993665,
                                                 0.2369268850561891};

}  // namespace

CubicSpline1D::CubicSpline1D(std::vector<double> knots, std::vector<double> values,
                             Boundary boundary)
    : knots_(std::move(knots)), values_(std::move(values)), boundary_(boundary) {
    const std::size_t count = knots_.size();
    if (count != values_.size()) throw std::invalid_argument("spline: knots and values differ");
    const std::size_t min_count = boundary_ == Boundary::Periodic ? 3 : 2;
    if (count < min_count) throw std::invalid_argument("spline: too few knots");
    for (std::size_t i = 1; i < count; ++i) {
        if (!(knots_[i] > knots_[i - 1])) {
            throw std::invalid_argument("spline: knots must be strictly increasing");
        }
    }
    const std::size_t n = count - 1;  // intervals
    std::vector<double> h(n);
    for (std::size_t i = 0; i < n; ++i) h[i] = knots_[i + 1] - knots_[i];
    auto slope = [&](std::size_t i) { return (values_[i + 1] - values_[i]) / h[i]; };

    second_.assign(count, 0.0);
    if (boundary_ == Boundary::Periodic) {
        if (std::abs(values_.front() - values_.back()) >
            1e-9 * std::max(1.0, std::abs(values_.front()))) {
            throw std::invalid_argument("spline: periodic ends must match");
        }
        // Unknowns M_0 .. M_{n-1}; M_n = M_0. Equation i couples M_{i-1}, M_i, M_{i+1} cyclically.
        std::vector<double> sub(n), diag(n), super(n), rhs(n);
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t prev = (i + n - 1) % n;
            sub[i] = h[prev];
            diag[i] = 2.0 * (h[prev] + h[i]);
            super[i] = h[i];
            rhs[i] = 6.0 * (slope(i) - slope(prev));
        }
        if (n == 2) {
            // Both neighbours of each unknown are the same unknown: a dense 2x2 system.
            const double a = diag[0], b = sub[0] + super[0];
            const double c = sub[1] + super[1], d = diag[1];
            const double det = a * d - b * c;
            second_[0] = (rhs[0] * d - b * rhs[1]) / det;
            second_[1] = (a * rhs[1] - c * rhs[0]) / det;
        } else {
            const std::vector<double> m = solve_cyclic_tridiagonal(sub, diag, super, rhs);
            std::copy(m.begin(), m.end(), second_.begin());
        }
        second_[n] = second_[0];
    } else if (n >= 2) {
        // Natural: M_0 = M_n = 0; unknowns M_1 .. M_{n-1}.
        const std::size_t m = n - 1;
        std::vector<double> sub(m), diag(m), super(m), rhs(m);
        for (std::size_t k = 0; k < m; ++k) {
            const std::size_t i = k + 1;
            sub[k] = h[i - 1];
            diag[k] = 2.0 * (h[i - 1] + h[i]);
            super[k] = h[i];
            rhs[k] = 6.0 * (slope(i) - slope(i - 1));
        }
        const std::vector<double> inner = solve_tridiagonal(sub, diag, super, rhs);
        std::copy(inner.begin(), inner.end(), second_.begin() + 1);
    }
}

std::size_t CubicSpline1D::locate(double& t) const {
    const double lo = knots_.front();
    const double hi = knots_.back();
    if (boundary_ == Boundary::Periodic) {
        const double period = hi - lo;
        t = lo + std::fmod(t - lo, period);
        if (t < lo) t += period;
        if (t >= hi) t = lo;
    } else {
        t = std::clamp(t, lo, hi);
    }
    const auto it = std::upper_bound(knots_.begin(), knots_.end(), t);
    const auto index = static_cast<std::size_t>(std::distance(knots_.begin(), it));
    return std::min(index == 0 ? 0 : index - 1, knots_.size() - 2);
}

double CubicSpline1D::value(double t) const {
    const std::size_t i = locate(t);
    const double h = knots_[i + 1] - knots_[i];
    const double a = (knots_[i + 1] - t) / h;
    const double b = (t - knots_[i]) / h;
    return a * values_[i] + b * values_[i + 1] +
           ((a * a * a - a) * second_[i] + (b * b * b - b) * second_[i + 1]) * h * h / 6.0;
}

double CubicSpline1D::derivative(double t) const {
    const std::size_t i = locate(t);
    const double h = knots_[i + 1] - knots_[i];
    const double a = (knots_[i + 1] - t) / h;
    const double b = (t - knots_[i]) / h;
    return (values_[i + 1] - values_[i]) / h - (3.0 * a * a - 1.0) * h * second_[i] / 6.0 +
           (3.0 * b * b - 1.0) * h * second_[i + 1] / 6.0;
}

double CubicSpline1D::second_derivative(double t) const {
    const std::size_t i = locate(t);
    const double h = knots_[i + 1] - knots_[i];
    const double a = (knots_[i + 1] - t) / h;
    const double b = (t - knots_[i]) / h;
    return a * second_[i] + b * second_[i + 1];
}

Spline2D::Spline2D(std::span<const Vec2> points, bool closed) : closed_(closed) {
    const std::size_t min_points = closed ? 3 : 2;
    if (points.size() < min_points) throw std::invalid_argument("Spline2D: too few points");
    std::vector<double> t{0.0};
    std::vector<double> xs{points[0].x()};
    std::vector<double> ys{points[0].y()};
    const std::size_t count = closed ? points.size() + 1 : points.size();
    for (std::size_t i = 1; i < count; ++i) {
        const Vec2& p = points[i % points.size()];
        const double chord = (p - points[i - 1]).norm();
        if (!(chord > 1e-9)) throw std::invalid_argument("Spline2D: repeated consecutive points");
        t.push_back(t.back() + chord);
        xs.push_back(p.x());
        ys.push_back(p.y());
    }
    const auto boundary =
        closed ? CubicSpline1D::Boundary::Periodic : CubicSpline1D::Boundary::Natural;
    x_ = CubicSpline1D(t, std::move(xs), boundary);
    y_ = CubicSpline1D(std::move(t), std::move(ys), boundary);
}

Vec2 Spline2D::position(double t) const {
    return {x_.value(t), y_.value(t)};
}

Vec2 Spline2D::derivative(double t) const {
    return {x_.derivative(t), y_.derivative(t)};
}

Vec2 Spline2D::second_derivative(double t) const {
    return {x_.second_derivative(t), y_.second_derivative(t)};
}

double Spline2D::heading(double t) const {
    const Vec2 d = derivative(t);
    return std::atan2(d.y(), d.x());
}

double Spline2D::curvature(double t) const {
    const Vec2 d = derivative(t);
    const Vec2 dd = second_derivative(t);
    const double speed = d.norm();
    if (speed < 1e-12) return 0.0;
    return (d.x() * dd.y() - d.y() * dd.x()) / (speed * speed * speed);
}

namespace {

// Arc length of the spline between parameters a and b (a <= b, within one knot interval or a
// short sub-interval, where the integrand is smooth).
double arc_length_between(const Spline2D& spline, double a, double b) {
    const double half = 0.5 * (b - a);
    const double mid = 0.5 * (a + b);
    double sum = 0.0;
    for (std::size_t k = 0; k < kGaussNodes.size(); ++k) {
        sum += kGaussWeights[k] * spline.derivative(mid + half * kGaussNodes[k]).norm();
    }
    return sum * half;
}

}  // namespace

double Spline2D::arc_length() const {
    // Sub-intervals of the chord parameter; cheap and accurate to ~1e-9 relative for smooth
    // curves. Splitting per knot is not needed because the integrand is smooth within the
    // sub-intervals, which are far shorter than any feature of a track.
    constexpr double kStep = 0.25;
    const double total = t_max();
    const auto steps = static_cast<std::size_t>(std::ceil(total / kStep));
    double length = 0.0;
    for (std::size_t i = 0; i < steps; ++i) {
        const double a = total * static_cast<double>(i) / static_cast<double>(steps);
        const double b = total * static_cast<double>(i + 1) / static_cast<double>(steps);
        length += arc_length_between(*this, a, b);
    }
    return length;
}

std::vector<Spline2D::Sample> Spline2D::resample(double spacing) const {
    if (!(spacing > 0.0)) throw std::invalid_argument("Spline2D::resample: spacing must be > 0");
    // Table of (parameter, arc length) on a fine grid, then Newton refinement per sample.
    constexpr double kStep = 0.25;
    const double total_t = t_max();
    const auto steps =
        std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(total_t / kStep)));
    std::vector<double> grid_t(steps + 1);
    std::vector<double> grid_s(steps + 1, 0.0);
    for (std::size_t i = 0; i <= steps; ++i) {
        grid_t[i] = total_t * static_cast<double>(i) / static_cast<double>(steps);
        if (i > 0) grid_s[i] = grid_s[i - 1] + arc_length_between(*this, grid_t[i - 1], grid_t[i]);
    }
    const double length = grid_s.back();
    const double ratio = length / spacing;
    std::size_t intervals = static_cast<std::size_t>(std::max(1.0, std::round(ratio)));
    if (closed_) intervals = std::max<std::size_t>(intervals, 3);
    const double ds = length / static_cast<double>(intervals);
    const std::size_t count = closed_ ? intervals : intervals + 1;

    std::vector<Sample> samples;
    samples.reserve(count);
    for (std::size_t k = 0; k < count; ++k) {
        const double target = ds * static_cast<double>(k);
        const auto it = std::upper_bound(grid_s.begin(), grid_s.end(), target);
        std::size_t j = static_cast<std::size_t>(std::distance(grid_s.begin(), it));
        j = std::clamp<std::size_t>(j == 0 ? 0 : j - 1, 0, steps - 1);
        const double seg = grid_s[j + 1] - grid_s[j];
        double t = grid_t[j];
        if (seg > 0.0) t += (grid_t[j + 1] - grid_t[j]) * (target - grid_s[j]) / seg;
        for (int iter = 0; iter < 3; ++iter) {
            const double error = grid_s[j] + arc_length_between(*this, grid_t[j], t) - target;
            const double speed = derivative(t).norm();
            if (speed < 1e-12) break;
            t -= error / speed;
        }
        if (k == count - 1 && !closed_) t = total_t;
        samples.push_back({target, position(t), heading(t), curvature(t)});
    }
    return samples;
}

}  // namespace apex
