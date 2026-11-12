#include "apex/planning/predicates.hpp"

#include <cmath>

#include <gtest/gtest.h>

#include "apex/core/rng.hpp"

namespace apex::planning {
namespace {

TEST(Predicates, OrientationOfSimpleTriangles) {
    EXPECT_EQ(orientation({0, 0}, {1, 0}, {0, 1}), 1);
    EXPECT_EQ(orientation({0, 0}, {0, 1}, {1, 0}), -1);
    EXPECT_EQ(orientation({0, 0}, {1, 1}, {3, 3}), 0);
    EXPECT_EQ(orientation({1, 2}, {1, 2}, {5, 7}), 0);
}

// Shewchuk's classic failure of the naive determinant: points within a few ulps of the line
// y = x. For a = (0.5 + i u, 0.5 + j u), b = (12, 12), c = (24, 24) the exact determinant is
// 12 (j - i) u, so its sign is the sign of j - i. Every coordinate is exactly representable.
TEST(Predicates, OrientationIsExactNextToALine) {
    const double u = std::ldexp(1.0, -53);  // the spacing of doubles in [0.5, 1)
    int naive_wrong = 0;
    for (int i = 0; i < 64; ++i) {
        for (int j = 0; j < 64; ++j) {
            const Vec2 a{0.5 + i * u, 0.5 + j * u};
            const Vec2 b{12.0, 12.0};
            const Vec2 c{24.0, 24.0};
            const int expected = (j > i) - (j < i);
            EXPECT_EQ(orientation(a, b, c), expected) << i << ", " << j;
            EXPECT_EQ(orientation(b, c, a), expected);
            EXPECT_EQ(orientation(b, a, c), -expected);
            const double naive =
                (a.x() - c.x()) * (b.y() - c.y()) - (a.y() - c.y()) * (b.x() - c.x());
            if ((naive > 0) - (naive < 0) != expected) ++naive_wrong;
        }
    }
    EXPECT_GT(naive_wrong, 0);  // the test does reach the exact fallback
}

// Integer points on the circle x^2 + y^2 = 25, exactly representable and exactly cocircular.
TEST(Predicates, InCircleIsExactForCocircularPoints) {
    for (const double offset : {0.0, 1e7, -3.5e9}) {
        const Vec2 shift{offset, -offset};
        const Vec2 a = Vec2{5, 0} + shift;
        const Vec2 b = Vec2{0, 5} + shift;
        const Vec2 c = Vec2{-5, 0} + shift;
        EXPECT_EQ(in_circle(a, b, c, Vec2{3, 4} + shift), 0);
        EXPECT_EQ(in_circle(a, b, c, Vec2{-4, -3} + shift), 0);
        EXPECT_EQ(in_circle(a, b, c, Vec2{0, -5} + shift), 0);
        EXPECT_EQ(in_circle(a, b, c, shift), 1);
        EXPECT_EQ(in_circle(a, b, c, Vec2{6, 0} + shift), -1);
        EXPECT_EQ(in_circle(a, c, b, shift), -1);  // clockwise flips the sign

        // One ulp off the circle, either way.
        const Vec2 on = Vec2{3, 4} + shift;
        const Vec2 outside{on.x(), std::nextafter(on.y(), on.y() + 1.0)};
        const Vec2 inside{on.x(), std::nextafter(on.y(), on.y() - 1.0)};
        EXPECT_EQ(in_circle(a, b, c, outside), -1) << offset;
        EXPECT_EQ(in_circle(a, b, c, inside), 1) << offset;
    }
}

TEST(Predicates, InCircleIsAntisymmetricOnRandomPoints) {
    Rng rng(7);
    for (int trial = 0; trial < 2000; ++trial) {
        const Vec2 a{rng.uniform(-10, 10), rng.uniform(-10, 10)};
        const Vec2 b{rng.uniform(-10, 10), rng.uniform(-10, 10)};
        const Vec2 c{rng.uniform(-10, 10), rng.uniform(-10, 10)};
        const Vec2 d{rng.uniform(-10, 10), rng.uniform(-10, 10)};
        const int s = in_circle(a, b, c, d);
        EXPECT_EQ(in_circle(b, a, c, d), -s);
        EXPECT_EQ(in_circle(b, c, a, d), s);
        // Swapping d with a vertex flips the sign too (row swap in the 4x4 lifted form).
        EXPECT_EQ(in_circle(d, b, c, a), -s);
    }
}

}  // namespace
}  // namespace apex::planning
