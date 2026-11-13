#include "scene.hpp"

#include <vector>

#include <gtest/gtest.h>

#include "apex/core/vehicle.hpp"
#include "apex/sim/lidar.hpp"

namespace apex::perception {
namespace {

// The tests' ray caster, which also simulates pitch and a wrong mount height, agrees with the
// simulator's reference LiDAR on the scans both can produce.
TEST(TestRayCaster, MatchesTheSimulatorsReferenceLidar) {
    const LidarGeometry geometry;
    const std::vector<Cone> cones = {{0, {8.0, 3.0}, ConeColor::Blue},
                                     {1, {8.3, -2.5}, ConeColor::Yellow},
                                     {2, {14.0, 2.0}, ConeColor::BigOrange},
                                     {3, {20.0, 2.15}, ConeColor::Blue},
                                     {4, {3.0, 1.9}, ConeColor::Orange}};
    VehicleState state;
    state.pose = {1.0, -0.5, 0.1};
    const LidarScan reference = sim::simulate_lidar_brute_force(geometry, state, cones, 0.0);
    Rng rng(1);
    const LidarScan ours =
        testing::cast_lidar(geometry, testing::lidar_pose(state.pose, geometry), cones, {}, rng);
    ASSERT_EQ(ours.points.size(), reference.points.size());
    std::size_t cone_returns = 0;
    for (std::size_t i = 0; i < ours.points.size(); ++i) {
        const LidarPoint& a = ours.points[i];
        const LidarPoint& b = reference.points[i];
        ASSERT_EQ(a.truth_id, b.truth_id) << "point " << i;
        EXPECT_NEAR(a.x, b.x, 1e-4) << "point " << i;
        EXPECT_NEAR(a.y, b.y, 1e-4) << "point " << i;
        EXPECT_NEAR(a.z, b.z, 1e-4) << "point " << i;
        cone_returns += a.truth_id >= 0 ? 1 : 0;
    }
    EXPECT_GT(cone_returns, 100U);
}

}  // namespace
}  // namespace apex::perception
