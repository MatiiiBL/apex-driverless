#include "apex/core/spline.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

namespace apex {
namespace {

std::vector<Vec2> circle(double radius, std::size_t n) {
    std::vector<Vec2> points;
    for (std::size_t i = 0; i < n; ++i) {
        points.push_back(radius *
                         heading_vector(kTwoPi * static_cast<double>(i) / static_cast<double>(n)));
    }
    return points;
}

TEST(CubicSpline1D, InterpolatesKnotsAndReproducesLines) {
    const CubicSpline1D line({0.0, 1.0, 3.0, 4.0}, {1.0, 3.0, 7.0, 9.0},
                             CubicSpline1D::Boundary::Natural);
    for (const double t : {0.0, 0.5, 1.0, 2.2, 4.0}) {
        EXPECT_NEAR(line.value(t), 1.0 + 2.0 * t, 1e-12);
        EXPECT_NEAR(line.derivative(t), 2.0, 1e-12);
        EXPECT_NEAR(line.second_derivative(t), 0.0, 1e-12);
    }
    // Outside the domain a natural spline clamps to its end values.
    EXPECT_NEAR(line.value(10.0), 9.0, 1e-12);
}

TEST(CubicSpline1D, PeriodicIsSmoothAcrossTheSeam) {
    std::vector<double> t;
    std::vector<double> y;
    for (int i = 0; i <= 12; ++i) {
        t.push_back(i * kTwoPi / 12.0);
        y.push_back(std::sin(t.back()));
    }
    y.back() = y.front();
    const CubicSpline1D spline(t, y, CubicSpline1D::Boundary::Periodic);
    const double eps = 1e-7;
    EXPECT_NEAR(spline.value(kTwoPi - eps), spline.value(eps), 1e-6);
    EXPECT_NEAR(spline.derivative(kTwoPi - eps), spline.derivative(eps), 1e-5);
    EXPECT_NEAR(spline.second_derivative(kTwoPi - eps), spline.second_derivative(eps), 1e-5);
    EXPECT_NEAR(spline.value(1.0), std::sin(1.0), 2e-3);
    EXPECT_NEAR(spline.value(1.0 + kTwoPi), spline.value(1.0), 1e-12);  // wraps
}

TEST(CubicSpline1D, RejectsBadInput) {
    using B = CubicSpline1D::Boundary;
    EXPECT_THROW(CubicSpline1D({0.0}, {1.0}, B::Natural), std::invalid_argument);
    EXPECT_THROW(CubicSpline1D({0.0, 0.0, 1.0}, {1, 2, 3}, B::Natural), std::invalid_argument);
    EXPECT_THROW(CubicSpline1D({0.0, 1.0, 2.0}, {1, 2, 3}, B::Periodic), std::invalid_argument);
    EXPECT_THROW(CubicSpline1D({0.0, 1.0}, {1.0}, B::Natural), std::invalid_argument);
}

TEST(Spline2D, ClosedCircleHasConstantCurvature) {
    const double r = 10.0;
    const Spline2D spline(circle(r, 24), true);
    EXPECT_NEAR(spline.arc_length(), kTwoPi * r, 1e-3 * kTwoPi * r);
    for (double t = 0.0; t < spline.t_max(); t += 0.37) {
        EXPECT_NEAR(spline.curvature(t), 1.0 / r, 2e-3) << t;
        EXPECT_NEAR(spline.position(t).norm(), r, 2e-3) << t;
    }
}

TEST(Spline2D, ResampleIsEvenlySpacedInArcLength) {
    const Spline2D spline(circle(10.0, 24), true);
    const auto samples = spline.resample(0.5);
    const double length = spline.arc_length();
    const double expected = length / static_cast<double>(samples.size());
    ASSERT_GT(samples.size(), 100U);
    for (std::size_t i = 0; i < samples.size(); ++i) {
        const Vec2& a = samples[i].position;
        const Vec2& b = samples[(i + 1) % samples.size()].position;
        EXPECT_NEAR((b - a).norm(), expected, 1e-4) << i;
        EXPECT_NEAR(samples[i].s, expected * static_cast<double>(i), 1e-6);
    }
    // Counter-clockwise circle: heading is 90 degrees ahead of the polar angle.
    EXPECT_NEAR(angle_diff(samples[0].heading, kPi / 2.0), 0.0, 1e-3);
}

TEST(Spline2D, OpenResampleEndsOnTheLastPoint) {
    const std::vector<Vec2> points{{0, 0}, {3, 1}, {6, 0}, {9, -2}};
    const Spline2D spline(points, false);
    const auto samples = spline.resample(1.0);
    EXPECT_TRUE(samples.front().position.isApprox(points.front()));
    EXPECT_NEAR((samples.back().position - points.back()).norm(), 0.0, 1e-9);
    EXPECT_NEAR(samples.back().s, spline.arc_length(), 1e-6);
}

}  // namespace
}  // namespace apex
