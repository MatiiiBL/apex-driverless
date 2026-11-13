#include "apex/sim/lidar.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/rng.hpp"
#include "apex/sim/sensors.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::sim {
namespace {

const ConeShape kSmall = cone_shape(ConeColor::Blue);

LidarNoise silent() {
    LidarNoise noise;
    noise.range_stddev = 0.0;
    noise.dropout = 0.0;
    return noise;
}

VehicleState at_pose(double x, double y, double yaw) {
    VehicleState state;
    state.pose = {x, y, yaw};
    return state;
}

TEST(IntersectCone, HitsTheSurfaceWhereItsRadiusMatches) {
    // Horizontal rays towards the axis at several heights meet the surface at radius
    // R (1 - z / H) from it.
    const Vec2 base(5.0, 0.0);
    for (const double z : {0.01, 0.1, 0.2, 0.3}) {
        const std::optional<double> t =
            intersect_cone(Vec3(0.0, 0.0, z), Vec3(1.0, 0.0, 0.0), base, kSmall);
        ASSERT_TRUE(t.has_value()) << z;
        EXPECT_NEAR(*t, 5.0 - kSmall.base_radius * (1.0 - z / kSmall.height), 1e-12);
    }
    // Straight down, 5 cm off the axis: onto the surface where the radius is 5 cm.
    const std::optional<double> down =
        intersect_cone(Vec3(5.05, 0.0, 1.0), Vec3(0.0, 0.0, -1.0), base, kSmall);
    ASSERT_TRUE(down.has_value());
    EXPECT_NEAR(*down, 1.0 - kSmall.height * (1.0 - 0.05 / kSmall.base_radius), 1e-12);
    // Slanting down onto the far side's surface from above the apex height.
    const Vec3 slant = Vec3(1.0, 0.0, -0.05).normalized();
    const std::optional<double> t = intersect_cone(Vec3(0.0, 0.0, 0.5), slant, base, kSmall);
    ASSERT_TRUE(t.has_value());
    const Vec3 hit = Vec3(0.0, 0.0, 0.5) + *t * slant;
    EXPECT_NEAR((hit.head<2>() - base).norm(), kSmall.base_radius * (1.0 - hit.z() / kSmall.height),
                1e-12);
}

TEST(IntersectCone, MissesAboveBesideAndBehind) {
    const Vec2 base(5.0, 0.0);
    // Above the apex: the quadric's upper nappe is not part of the body.
    EXPECT_FALSE(intersect_cone(Vec3(0.0, 0.0, 0.4), Vec3(1.0, 0.0, 0.0), base, kSmall));
    // Passing beside the base.
    EXPECT_FALSE(intersect_cone(Vec3(0.0, 0.2, 0.05), Vec3(1.0, 0.0, 0.0), base, kSmall));
    // Pointing away.
    EXPECT_FALSE(intersect_cone(Vec3(0.0, 0.0, 0.05), Vec3(-1.0, 0.0, 0.0), base, kSmall));
    // Below the ground.
    EXPECT_FALSE(intersect_cone(Vec3(0.0, 0.0, -0.1), Vec3(1.0, 0.0, 0.0), base, kSmall));
}

void expect_identical(const LidarScan& fast, const LidarScan& reference) {
    ASSERT_EQ(fast.points.size(), reference.points.size());
    int cone_points = 0;
    for (std::size_t i = 0; i < fast.points.size(); ++i) {
        const LidarPoint& a = fast.points[i];
        const LidarPoint& b = reference.points[i];
        ASSERT_TRUE(a.x == b.x && a.y == b.y && a.z == b.z && a.truth_id == b.truth_id)
            << "point " << i << ": (" << a.x << ", " << a.y << ", " << a.z << ") id " << a.truth_id
            << " vs (" << b.x << ", " << b.y << ", " << b.z << ") id " << b.truth_id;
        if (a.truth_id >= 0) ++cone_points;
    }
    EXPECT_GT(cone_points, 50);
}

// The culled raycaster returns exactly what testing every ray against every cone returns:
// from the staging position and around a generated track, on a 360 degree scan whose columns
// wrap around the back, and with the sensor standing right over a cone's base.
TEST(Lidar, MatchesBruteForceExactly) {
    const Track track = generate_track(3);
    Rng rng(1);
    LidarGeometry coarse;
    coarse.azimuth_step = deg_to_rad(0.5);
    for (const double s : {track.centerline.length() - 6.0, 60.0, 140.0, 230.0}) {
        SCOPED_TRACE(s);
        const TrajectoryPoint p = track.centerline.at(s);
        const VehicleState state = at_pose(p.position.x(), p.position.y(), p.yaw + 0.2);
        expect_identical(simulate_lidar(coarse, silent(), state, track.cones, 0.0, rng),
                         simulate_lidar_brute_force(coarse, state, track.cones, 0.0));
    }
    LidarGeometry all_around;
    all_around.azimuth_min = -kPi;
    all_around.azimuth_max = kPi - deg_to_rad(1.0);
    all_around.azimuth_step = deg_to_rad(1.0);
    const TrajectoryPoint p = track.centerline.at(100.0);
    const VehicleState state = at_pose(p.position.x(), p.position.y(), p.yaw);
    expect_identical(simulate_lidar(all_around, silent(), state, track.cones, 0.0, rng),
                     simulate_lidar_brute_force(all_around, state, track.cones, 0.0));

    // Mount 0.05 m from a cone's axis.
    const LidarGeometry standard;
    const Vec2 cone = track.cones[20].position;
    const Vec2 cog = cone - standard.mount.x() * heading_vector(0.7) + Vec2(0.05, 0.0);
    const VehicleState over = at_pose(cog.x(), cog.y(), 0.7);
    expect_identical(simulate_lidar(coarse, silent(), over, track.cones, 0.0, rng),
                     simulate_lidar_brute_force(coarse, over, track.cones, 0.0));
}

// The default 57,600-ray scan from the staging position, once (the comparison above covers
// the geometry; this covers the default channel layout and step).
TEST(Lidar, MatchesBruteForceAtFullResolution) {
    const Track track = reference_track("technical");
    Rng rng(2);
    const VehicleState state =
        at_pose(track.start_pose.x, track.start_pose.y, track.start_pose.yaw);
    const LidarGeometry geometry;
    expect_identical(simulate_lidar(geometry, silent(), state, track.cones, 0.0, rng),
                     simulate_lidar_brute_force(geometry, state, track.cones, 0.0));
}

// Returns lie on the bodies they are tagged with and on the ground; a cone casts a shadow on the
// ground and on a cone behind it.
TEST(Lidar, ReturnsLieOnConesAndGroundWithShadows) {
    const LidarGeometry geometry;
    const double h = geometry.mount.z();
    Rng rng(3);
    const VehicleState state;  // CoG at the origin, LiDAR at (1.2, 0, 0.2)
    const std::vector<Cone> one = {{0, {11.2, 0.0}, ConeColor::Yellow}};
    const LidarScan scan = simulate_lidar(geometry, silent(), state, one, 0.5, rng);
    EXPECT_EQ(scan.stamp, 0.5);
    int on_cone = 0;
    for (const LidarPoint& p : scan.points) {
        if (p.truth_id == 0) {
            ++on_cone;
            const double z = p.z + h;  // above the ground
            EXPECT_NEAR(std::hypot(p.x - 10.0, p.y), kSmall.base_radius * (1.0 - z / kSmall.height),
                        1e-5);
            EXPECT_LT(p.x, 10.0 + 1e-5);  // the near face only
        } else {
            EXPECT_EQ(p.truth_id, -1);
            EXPECT_NEAR(p.z, -h, 1e-5);
            // No ground seen through the cone: where a ray to a ground point behind the cone
            // passes x = 10 m, it must be outside the body at that height.
            if (p.x > 10.5) {
                const double f = 10.0 / p.x;
                const double z = h * (1.0 - f);
                EXPECT_GE(std::abs(f * p.y), kSmall.base_radius * (1.0 - z / kSmall.height) - 1e-3)
                    << p.x << ", " << p.y;
            }
        }
        const double range = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        EXPECT_GE(range, geometry.min_range - 1e-4);
        EXPECT_LE(range, geometry.max_range + 1e-4);
    }
    // 0.23 m wide at 10 m: about 6 columns, and channels from -1.1 to +0.7 degrees.
    EXPECT_GT(on_cone, 20);
    // The cone's shadow costs ground returns.
    const std::vector<Cone> none;
    EXPECT_LT(scan.points.size() - static_cast<std::size_t>(on_cone),
              simulate_lidar(geometry, silent(), state, none, 0.5, rng).points.size());

    std::vector<Cone> two = one;
    two.push_back({1, {6.2, 0.0}, ConeColor::Blue});
    int far_points = 0;
    for (const LidarPoint& p : simulate_lidar(geometry, silent(), state, two, 0.5, rng).points) {
        if (p.truth_id == 0) ++far_points;
    }
    EXPECT_LT(far_points, on_cone / 2);
}

TEST(Lidar, RangeNoiseAndDropoutHaveTheirConfiguredLevels) {
    const Track track = make_oval_track();
    const VehicleState state =
        at_pose(track.start_pose.x, track.start_pose.y, track.start_pose.yaw);
    const LidarGeometry geometry;
    Rng rng(4);
    const LidarScan truth = simulate_lidar(geometry, silent(), state, track.cones, 0.0, rng);

    LidarNoise ranged = silent();
    ranged.range_stddev = 0.02;
    const LidarScan noisy = simulate_lidar(geometry, ranged, state, track.cones, 0.0, rng);
    ASSERT_EQ(noisy.points.size(), truth.points.size());
    double sum = 0.0;
    double squares = 0.0;
    for (std::size_t i = 0; i < truth.points.size(); ++i) {
        const LidarPoint& a = truth.points[i];
        const LidarPoint& b = noisy.points[i];
        ASSERT_EQ(a.truth_id, b.truth_id);
        const double ra = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
        const double rb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
        sum += rb - ra;
        squares += (rb - ra) * (rb - ra);
    }
    const auto n = static_cast<double>(truth.points.size());
    EXPECT_NEAR(sum / n, 0.0, 0.001);
    EXPECT_NEAR(std::sqrt(squares / n), 0.02, 0.001);

    LidarNoise dropping = silent();
    dropping.dropout = 0.02;
    const LidarScan thinned = simulate_lidar(geometry, dropping, state, track.cones, 0.0, rng);
    EXPECT_NEAR(static_cast<double>(thinned.points.size()) / n, 0.98, 0.003);
}

}  // namespace
}  // namespace apex::sim
