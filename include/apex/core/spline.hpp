#pragma once

#include <span>
#include <vector>

#include "apex/core/types.hpp"

namespace apex {

// An interpolating cubic spline y(t) through knots t_0 < t_1 < ... < t_n.
//
// Natural: zero second derivative at both ends. Periodic: the curve wraps from t_n back to t_0
// with continuous value, slope and curvature; it requires y_n == y_0 (the caller repeats the
// first value at the end).
class CubicSpline1D {
public:
    enum class Boundary { Natural, Periodic };

    CubicSpline1D() = default;
    // Throws std::invalid_argument for fewer than 2 knots (3 for periodic), non-increasing
    // knots, mismatched sizes, or a periodic spline whose ends differ.
    CubicSpline1D(std::vector<double> knots, std::vector<double> values, Boundary boundary);

    [[nodiscard]] double value(double t) const;
    [[nodiscard]] double derivative(double t) const;
    [[nodiscard]] double second_derivative(double t) const;

    [[nodiscard]] double t_min() const { return knots_.front(); }
    [[nodiscard]] double t_max() const { return knots_.back(); }
    [[nodiscard]] bool periodic() const { return boundary_ == Boundary::Periodic; }

private:
    // Interval index i with knots_[i] <= t <= knots_[i + 1], and t mapped into the domain
    // (wrapped when periodic, clamped otherwise).
    [[nodiscard]] std::size_t locate(double& t) const;

    std::vector<double> knots_;
    std::vector<double> values_;
    std::vector<double> second_;  // second derivative at each knot
    Boundary boundary_ = Boundary::Natural;
};

// A planar curve through points, parametrized by cumulative chord length, with one cubic spline
// per coordinate. Closed curves wrap smoothly through the first point. Arc length differs from
// the chord parameter by well under 1% for points spaced along a smooth curve; resample() uses
// true arc length.
class Spline2D {
public:
    Spline2D() = default;
    // Throws std::invalid_argument for fewer than 2 points (3 if closed) or repeated
    // consecutive points.
    Spline2D(std::span<const Vec2> points, bool closed);

    [[nodiscard]] Vec2 position(double t) const;
    [[nodiscard]] Vec2 derivative(double t) const;
    [[nodiscard]] Vec2 second_derivative(double t) const;
    [[nodiscard]] double heading(double t) const;
    // Signed curvature in 1/m, positive turning left.
    [[nodiscard]] double curvature(double t) const;

    [[nodiscard]] double t_max() const { return x_.t_max(); }
    [[nodiscard]] bool closed() const { return closed_; }

    // True arc length of the whole curve (Gauss-Legendre quadrature per knot interval).
    [[nodiscard]] double arc_length() const;

    struct Sample {
        double s;  // arc length from the start
        Vec2 position;
        double heading;
        double curvature;
    };
    // Samples at arc-length spacing as close to `spacing` as divides the length evenly. Closed
    // curves do not repeat the first sample at the end.
    [[nodiscard]] std::vector<Sample> resample(double spacing) const;

private:
    CubicSpline1D x_;
    CubicSpline1D y_;
    bool closed_ = false;
};

}  // namespace apex
