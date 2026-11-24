#include "apex/planning/smoothing.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/rng.hpp"

namespace apex::planning {
namespace {

std::vector<Vec2> circle(double radius, std::size_t n) {
    std::vector<Vec2> points;
    for (std::size_t i = 0; i < n; ++i) {
        const double angle = kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        points.push_back(radius * heading_vector(angle));
    }
    return points;
}

TEST(ResamplePolyline, OpenKeepsBothEndsAtEqualSteps) {
    const std::vector<Vec2> line = {{0.0, 0.0}, {4.0, 0.0}, {10.0, 0.0}};
    const std::vector<Vec2> result = resample_polyline(line, false, 3.0);
    ASSERT_EQ(result.size(), 4U);  // 10 m in three steps of 3.33 m
    for (std::size_t i = 0; i < result.size(); ++i) {
        EXPECT_NEAR(result[i].x(), 10.0 * static_cast<double>(i) / 3.0, 1e-12);
        EXPECT_EQ(result[i].y(), 0.0);
    }
}

TEST(ResamplePolyline, ClosedWrapsThroughTheFirstPoint) {
    const std::vector<Vec2> square = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}};
    const std::vector<Vec2> result = resample_polyline(square, true, 1.0);
    ASSERT_EQ(result.size(), 16U);
    EXPECT_EQ(result.front(), square.front());
    EXPECT_NEAR((result[12] - Vec2(0.0, 4.0)).norm(), 0.0, 1e-12);
    EXPECT_NEAR((result[15] - Vec2(0.0, 1.0)).norm(), 0.0, 1e-12);
    EXPECT_THROW((void)resample_polyline(square, true, 0.0), std::invalid_argument);
}

TEST(SmoothPolyline, LeavesAStraightLineAlone) {
    std::vector<Vec2> line;
    for (int i = 0; i < 20; ++i) line.emplace_back(1.0 * i, 0.5 * i);
    const std::vector<Vec2> result =
        smooth_polyline(line, {.spacing = std::hypot(1.0, 0.5), .cutoff_wavelength = 30.0});
    for (std::size_t i = 0; i < line.size(); ++i) {
        EXPECT_LT((result[i] - line[i]).norm(), 1e-9);
    }
}

// The half-gain wavelength is the cutoff: a lateral sine of that wavelength comes out with
// half its amplitude away from the ends.
TEST(SmoothPolyline, HalvesAWiggleAtTheCutoffWavelength) {
    constexpr double kWavelength = 8.0;
    std::vector<Vec2> line;
    for (int i = 0; i <= 400; ++i) {
        const double x = 0.25 * i;
        line.emplace_back(x, std::sin(kTwoPi * x / kWavelength));
    }
    const std::vector<Vec2> result =
        smooth_polyline(line, {.spacing = 0.25, .cutoff_wavelength = kWavelength});
    double amplitude = 0.0;
    for (std::size_t i = 150; i < 250; ++i) amplitude = std::max(amplitude, result[i].y());
    EXPECT_NEAR(amplitude, 0.5, 0.02);
}

// A closed circle shrinks by (cutoff / 2 pi)^4 / R^3 and loses most of its noise.
TEST(SmoothPolyline, ClosedCircleShrinksByThePredictedBias) {
    constexpr double kRadius = 6.0;
    constexpr double kCutoff = 10.0;
    const std::size_t n = 200;
    const double spacing = kTwoPi * kRadius / static_cast<double>(n);
    std::vector<Vec2> noisy = circle(kRadius, n);
    Rng rng(3);
    for (Vec2& p : noisy) p += Vec2{rng.normal(0.0, 0.1), rng.normal(0.0, 0.1)};
    const std::vector<Vec2> result =
        smooth_polyline(noisy, {.spacing = spacing, .cutoff_wavelength = kCutoff, .closed = true});
    const double bias = std::pow(kCutoff / kTwoPi, 4) / std::pow(kRadius, 3);
    double mean_radius = 0.0;
    double worst = 0.0;
    for (const Vec2& p : result) {
        mean_radius += p.norm() / static_cast<double>(n);
        worst = std::max(worst, std::abs(p.norm() - (kRadius - bias)));
    }
    EXPECT_NEAR(mean_radius, kRadius - bias, 0.01);
    EXPECT_LT(worst, 0.1);  // the noise alone reaches 0.3 m
}

TEST(SmoothPolyline, PinnedStartLeavesAlongTheGivenDirection) {
    // Points heading along +x, but the car at the first one faces 10 degrees to the left.
    constexpr double kHeadingError = deg_to_rad(10.0);
    std::vector<Vec2> line;
    for (int i = 0; i < 40; ++i) line.emplace_back(1.0 * i, 0.0);
    const Vec2 direction = heading_vector(kHeadingError);
    const std::vector<Vec2> result = smooth_polyline(
        line, {.spacing = 1.0, .cutoff_wavelength = 12.0, .start_direction = direction});
    EXPECT_EQ(result.front(), line.front());
    EXPECT_LT((result[1] - (result[0] + direction)).norm(), 1e-12);
    // It turns back onto the points over several steps, sharpest right after the start (the
    // minimal-bending curve from a clamped end), and settles onto them.
    double first_turn = 0.0;
    for (std::size_t i = 1; i + 1 < result.size(); ++i) {
        const Vec2 before = result[i] - result[i - 1];
        const Vec2 after = result[i + 1] - result[i];
        const double turn = std::abs(std::atan2(cross(before, after), before.dot(after)));
        if (i == 1) first_turn = turn;
        EXPECT_LE(turn, first_turn + 1e-12);
    }
    EXPECT_LT(first_turn, 0.75 * kHeadingError);
    EXPECT_LT(std::abs(result.back().y()), 0.01);
}

// The exploration planner's case: the car on a 7 m arc, heading along it, the first data point
// 6 m further along the arc, and a straight run interpolated between them (the chord). Fitted,
// the chord kinks the path where it leaves the pinned heading, and the path then runs straight
// along it; left free, the path bends from the pinned start into the arc.
TEST(SmoothPolyline, FreeStartBendsFromThePinnedStartIntoTheData) {
    constexpr double kRadius = 7.0;
    const auto on_arc = [&](double s) {
        return Vec2{kRadius * std::sin(s / kRadius), kRadius * (1.0 - std::cos(s / kRadius))};
    };
    std::vector<Vec2> raw{on_arc(0.0)};
    for (double s = 6.0; s <= 20.0; s += 2.0) raw.push_back(on_arc(s));
    const std::vector<Vec2> even = resample_polyline(raw, false, 1.0);
    const double chord = (raw[1] - raw[0]).norm();
    const double spacing =
        cumulative_length(raw, false).back() / static_cast<double>(even.size() - 1);
    // The largest turn per meter between consecutive points over the first `until` meters.
    const auto sharpest = [&](const std::vector<Vec2>& p, double until) {
        double worst = 0.0;
        for (std::size_t i = 1; i + 1 < p.size() && static_cast<double>(i) * spacing < until; ++i) {
            const Vec2 before = p[i] - p[i - 1];
            const Vec2 after = p[i + 1] - p[i];
            const double turn = std::atan2(cross(before, after), before.dot(after));
            worst = std::max(worst, std::abs(turn) / spacing);
        }
        return worst;
    };
    SmoothingOptions options{
        .spacing = spacing, .cutoff_wavelength = 8.0, .start_direction = Vec2{1.0, 0.0}};
    const std::vector<Vec2> fitted = smooth_polyline(even, options);
    options.free_start = chord;
    const std::vector<Vec2> free = smooth_polyline(even, options);

    EXPECT_GT(sharpest(fitted, chord), 2.5 / kRadius);
    // The first spacing is still pinned straight, so the path turns harder than the arc right
    // after it, but less than half as hard as the kink.
    EXPECT_LT(sharpest(free, chord + 2.0), 0.5 * sharpest(fitted, chord));
    EXPECT_LT(sharpest(free, chord + 2.0), 1.6 / kRadius);
    EXPECT_EQ(free.front(), even.front());
    EXPECT_LT((free[1] - (free[0] + spacing * Vec2{1.0, 0.0})).norm(), 1e-12);
    // Past the chord it lies on the arc again, as the data does: chords 2 m long, up to
    // 2^2 / (8 R) = 7 cm inside it.
    const Vec2 center{0.0, kRadius};
    for (std::size_t i = 0; i < even.size(); ++i) {
        const double s = static_cast<double>(i) * spacing;
        if (s > chord + 2.0 && s < 16.0) {
            EXPECT_LT(std::abs((free[i] - center).norm() - kRadius), 0.08) << i;
        }
    }
}

TEST(SmoothPolyline, RejectsBadOptionsAndPassesTinyInputsThrough) {
    const std::vector<Vec2> line = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 1.0}};
    EXPECT_THROW((void)smooth_polyline(line, {.spacing = 0.0}), std::invalid_argument);
    EXPECT_EQ(smooth_polyline(line, {.closed = true}), line);
}

}  // namespace
}  // namespace apex::planning
