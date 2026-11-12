#include "apex/core/rng.hpp"

#include <cmath>

#include <gtest/gtest.h>

namespace apex {
namespace {

TEST(Rng, SameSeedSameStream) {
    Rng a(42);
    Rng b(42);
    for (int i = 0; i < 100; ++i) EXPECT_EQ(a.next_u64(), b.next_u64());
    Rng c(43);
    EXPECT_NE(Rng(42).next_u64(), c.next_u64());
}

TEST(Rng, SplitStreamsAreIndependentOfDrawsElsewhere) {
    Rng parent(7);
    Rng lidar = parent.split(1);
    const std::uint64_t first = lidar.next_u64();
    // Drawing from the parent does not shift a split stream: splits depend on the seed only.
    for (int i = 0; i < 10; ++i) parent.next_u64();
    EXPECT_EQ(parent.split(1).next_u64(), first);
    EXPECT_NE(Rng(7).split(2).next_u64(), first);
}

TEST(Rng, DistributionsHaveTheRightMoments) {
    Rng rng(123);
    constexpr int kN = 200000;
    double sum = 0.0;
    double sum_sq = 0.0;
    double uniform_sum = 0.0;
    double poisson_sum = 0.0;
    int heads = 0;
    for (int i = 0; i < kN; ++i) {
        const double x = rng.normal(2.0, 3.0);
        sum += x;
        sum_sq += x * x;
        const double u = rng.uniform();
        ASSERT_GE(u, 0.0);
        ASSERT_LT(u, 1.0);
        uniform_sum += u;
        heads += rng.bernoulli(0.3) ? 1 : 0;
        poisson_sum += rng.poisson(2.5);
        ASSERT_LT(rng.index(7), 7U);
    }
    const double mean = sum / kN;
    EXPECT_NEAR(mean, 2.0, 0.03);
    EXPECT_NEAR(std::sqrt(sum_sq / kN - mean * mean), 3.0, 0.03);
    EXPECT_NEAR(uniform_sum / kN, 0.5, 0.005);
    EXPECT_NEAR(static_cast<double>(heads) / kN, 0.3, 0.005);
    EXPECT_NEAR(poisson_sum / kN, 2.5, 0.02);
}

}  // namespace
}  // namespace apex
