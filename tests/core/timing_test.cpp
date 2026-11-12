#include "apex/core/timing.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

namespace apex {
namespace {

TEST(LatencyStats, NearestRankPercentiles) {
    LatencyStats stats;
    EXPECT_EQ(stats.percentile(0.5), 0.0);
    for (int i = 1; i <= 100; ++i) stats.add(i * 1e-6);
    EXPECT_EQ(stats.count(), 100U);
    EXPECT_NEAR(stats.percentile(0.50), 50e-6, 1e-12);
    EXPECT_NEAR(stats.percentile(0.99), 99e-6, 1e-12);
    EXPECT_NEAR(stats.percentile(1.0), 100e-6, 1e-12);
    EXPECT_NEAR(stats.max(), 100e-6, 1e-12);
    EXPECT_NEAR(stats.mean(), 50.5e-6, 1e-12);
    const nlohmann::json summary = stats.summary();
    EXPECT_NEAR(summary["p99_us"].get<double>(), 99.0, 1e-6);

    LatencyStats other;
    other.add(1.0);
    stats.merge(other);
    EXPECT_EQ(stats.count(), 101U);
    EXPECT_DOUBLE_EQ(stats.max(), 1.0);
}

TEST(Stopwatch, MeasuresNonNegativeTime) {
    Stopwatch watch;
    EXPECT_GE(watch.elapsed_seconds(), 0.0);
}

}  // namespace
}  // namespace apex
