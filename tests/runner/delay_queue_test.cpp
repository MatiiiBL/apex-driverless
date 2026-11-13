#include "apex/runner/delay_queue.hpp"

#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace apex::runner {
namespace {

TEST(DelayQueue, ReleasesEachItemOnItsTick) {
    DelayQueue<int> queue;
    queue.push(1, 50);
    queue.push(2, 40);
    queue.push(3, 10);
    std::vector<int> out;
    queue.pop_due(9, out);
    EXPECT_TRUE(out.empty());
    queue.pop_due(10, out);
    EXPECT_EQ(out, std::vector<int>{3});
    out.clear();
    queue.pop_due(49, out);
    EXPECT_EQ(out, std::vector<int>{2});
    out.clear();
    queue.pop_due(50, out);
    EXPECT_EQ(out, std::vector<int>{1});
    EXPECT_TRUE(queue.empty());
}

TEST(DelayQueue, SameTickInPushOrder) {
    // A camera frame stamped at tick 60 with 40 ticks of latency comes due with the IMU sample
    // of tick 100; it was produced first, so it is handed over first.
    DelayQueue<std::string> queue;
    queue.push("camera@60", 100);
    queue.push("lidar@100", 150);
    queue.push("imu@100", 100);
    queue.push("wheels@100", 100);
    std::vector<std::string> out;
    queue.pop_due(100, out);
    EXPECT_EQ(out, (std::vector<std::string>{"camera@60", "imu@100", "wheels@100"}));
    EXPECT_EQ(queue.size(), 1U);
}

TEST(DelayQueue, CatchesUpOnEverythingOverdue) {
    DelayQueue<int> queue;
    for (int i = 0; i < 100; ++i) queue.push(i, 100 - i);
    std::vector<int> out;
    queue.pop_due(1000, out);
    ASSERT_EQ(out.size(), 100U);
    for (int i = 0; i < 100; ++i) EXPECT_EQ(out[static_cast<std::size_t>(i)], 99 - i);
}

TEST(DelayQueue, MovesItemsOut) {
    DelayQueue<std::vector<int>> queue;
    queue.push(std::vector<int>(1000, 7), 0);
    std::vector<std::vector<int>> out;
    queue.pop_due(0, out);
    ASSERT_EQ(out.size(), 1U);
    EXPECT_EQ(out[0].size(), 1000U);
}

}  // namespace
}  // namespace apex::runner
