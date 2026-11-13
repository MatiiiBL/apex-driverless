#include "apex/perception/perception.hpp"

#include <cmath>
#include <limits>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "apex/core/track.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/sim/sensors.hpp"
#include "apex/sim/track_gen.hpp"

#include "scene.hpp"

namespace apex::perception {
namespace {

using testing::cast_lidar;
using testing::detect_cones;
using testing::lidar_pose;
using testing::LidarSimOptions;

struct Scene {
    SensorRig rig;
    std::vector<Cone> cones;
    Pose2 body;  // car pose in the world
};

// The car on the centerline of the generator's "technical" layout (hairpins and chicanes, so
// cones at every range and bearing), `s` meters past the timekeeping line, heading along it.
Scene track_scene(double s = 30.0) {
    static const Track track = sim::reference_track("technical");
    Scene scene;
    scene.cones = track.cones;
    const TrajectoryPoint p = track.centerline.at(s);
    scene.body = {p.position.x(), p.position.y(), p.yaw};
    return scene;
}

LidarScan scan_of(const Scene& scene, std::uint64_t seed, Time stamp = 0.0,
                  LidarSimOptions options = {.range_sigma = 0.02, .dropout = 0.02}) {
    Rng rng(seed);
    return cast_lidar(scene.rig.lidar, lidar_pose(scene.body, scene.rig.lidar), scene.cones,
                      options, rng, stamp);
}

CameraFrame camera_of(const Scene& scene, std::uint64_t seed, Time stamp = 0.0) {
    Rng rng(seed);
    return detect_cones(scene.rig.camera, scene.body, scene.cones,
                        {.bearing_sigma = 0.005, .range_sigma_fraction = 0.05}, rng, stamp);
}

// Returns per cone id in the scan.
std::map<std::int32_t, std::size_t> returns_per_cone(const LidarScan& scan) {
    std::map<std::int32_t, std::size_t> count;
    for (const LidarPoint& p : scan.points) {
        if (p.truth_id >= 0) ++count[p.truth_id];
    }
    return count;
}

Vec2 truth_in_body(const Scene& scene, std::int32_t id) {
    return scene.body.to_local(scene.cones[static_cast<std::size_t>(id)].position);
}

TEST(ConePerception, FindsTheConesOfATrackWithNoFalsePositives) {
    double sum_sq = 0.0;
    int near = 0;
    for (const double s : {0.0, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0}) {
        const Scene scene = track_scene(s);
        ConePerception perception(PerceptionConfig{}, scene.rig);
        const LidarScan scan = scan_of(scene, static_cast<std::uint64_t>(s) + 1);
        const ConeObservations observations = perception.on_lidar(scan, 0.0);

        std::map<std::int32_t, int> reported;
        for (const ConeObservation& cone : observations.cones) {
            ASSERT_GE(cone.truth_id, 0)
                << "false positive at " << cone.position.transpose() << ", s = " << s;
            ++reported[cone.truth_id];
            const Vec2 truth = truth_in_body(scene, cone.truth_id);
            const double error = (cone.position - truth).norm();
            EXPECT_LT(error, 0.10) << "cone " << cone.truth_id << ", s = " << s;
            if (truth.norm() < 20.0) {
                sum_sq += error * error;
                ++near;
            }
            EXPECT_GT(cone.covariance.determinant(), 0.0);
            EXPECT_EQ(cone.color, ConeColor::Unknown);  // no camera frames yet
        }
        for (const auto& [id, count] : reported) {
            EXPECT_EQ(count, 1) << "cone " << id << " twice, s = " << s;
        }
        // Every cone within 20 m that the LiDAR sees well is found.
        for (const auto& [id, returns] : returns_per_cone(scan)) {
            if (truth_in_body(scene, id).norm() < 20.0 && returns >= 8) {
                EXPECT_TRUE(reported.contains(id)) << "missed cone " << id << ", s = " << s;
            }
        }
        for (std::size_t i = 1; i < observations.cones.size(); ++i) {
            EXPECT_LE(observations.cones[i - 1].position.norm(),
                      observations.cones[i].position.norm());
        }
    }
    ASSERT_GT(near, 50);
    EXPECT_LT(std::sqrt(sum_sq / near), 0.03);
}

TEST(ConePerception, TakesColorsFromTheCamera) {
    const Scene scene = track_scene();
    ConePerception perception(PerceptionConfig{}, scene.rig);
    perception.on_camera(camera_of(scene, 2, 0.0));
    const ConeObservations observations = perception.on_lidar(scan_of(scene, 2), 0.0);
    std::size_t colored = 0;
    for (const ConeObservation& cone : observations.cones) {
        const Cone& truth = scene.cones[static_cast<std::size_t>(cone.truth_id)];
        // Cones the camera sees clearly: within 20 m, and a little inside its field of view.
        const Vec2 from_camera =
            truth_in_body(scene, cone.truth_id) - scene.rig.camera.mount.head<2>();
        const double bearing = std::atan2(from_camera.y(), from_camera.x());
        if (from_camera.norm() < 20.0 &&
            std::abs(bearing) < 0.5 * scene.rig.camera.horizontal_fov - 0.05) {
            EXPECT_EQ(cone.color, truth.color) << "cone " << cone.truth_id << ", bearing "
                                               << bearing << ", range " << from_camera.norm();
        }
        if (cone.color != ConeColor::Unknown) {
            ++colored;
            EXPECT_GT(cone.color_confidence, 0.0);
            EXPECT_LE(cone.color_confidence, 1.0);
        } else {
            EXPECT_EQ(cone.color_confidence, 0.0);
        }
    }
    EXPECT_GT(colored, 8U);
    EXPECT_EQ(perception.stats().colored_by_camera, colored);
}

// The simulator's own sensor models with their default noise: what the car sees in a run.
TEST(ConePerception, WorksOnTheSimulatorsSensors) {
    const sim::SensorNoise noise;
    std::size_t colored = 0;
    std::size_t right = 0;
    double sum_sq = 0.0;
    int near = 0;
    for (const double s : {20.0, 110.0, 200.0}) {
        const Scene scene = track_scene(s);
        VehicleState state;
        state.pose = scene.body;
        const Rng rng(static_cast<std::uint64_t>(s));
        Rng lidar_rng = rng.split(0);
        Rng camera_rng = rng.split(1);
        ConePerception perception(PerceptionConfig{}, scene.rig);
        perception.on_camera(sim::simulate_camera(scene.rig.camera, noise.camera, state,
                                                  scene.cones, 1.0, camera_rng));
        const LidarScan scan =
            sim::simulate_lidar(scene.rig.lidar, noise.lidar, state, scene.cones, 1.0, lidar_rng);
        for (const ConeObservation& cone : perception.on_lidar(scan, 0.0).cones) {
            ASSERT_GE(cone.truth_id, 0) << "false positive at " << cone.position.transpose();
            const Vec2 truth = truth_in_body(scene, cone.truth_id);
            const double error = (cone.position - truth).norm();
            EXPECT_LT(error, 0.10) << "cone " << cone.truth_id << ", s = " << s;
            if (truth.norm() < 20.0) {
                sum_sq += error * error;
                ++near;
            }
            if (cone.color != ConeColor::Unknown) {
                ++colored;
                right += cone.color == scene.cones[static_cast<std::size_t>(cone.truth_id)].color;
            }
        }
    }
    ASSERT_GT(near, 20);
    EXPECT_LT(std::sqrt(sum_sq / near), 0.03);
    EXPECT_GT(colored, 20U);
    EXPECT_GE(right * 10, colored * 9) << right << " of " << colored << " colors right";
}

TEST(ConePerception, UsesTheClosestFrameAndCopesWithMissingOrLateFrames) {
    const Scene scene = track_scene();
    ConePerception perception(PerceptionConfig{}, scene.rig);
    // Frames arrive out of order; the one at the scan's stamp has the true colors, the others
    // claim every cone is orange.
    CameraFrame wrong_before = camera_of(scene, 3, 0.95);
    CameraFrame wrong_after = camera_of(scene, 3, 1.05);
    for (CameraDetection& d : wrong_before.detections) d.color = ConeColor::Orange;
    for (CameraDetection& d : wrong_after.detections) d.color = ConeColor::Orange;
    perception.on_camera(wrong_after);
    perception.on_camera(camera_of(scene, 3, 1.0));
    perception.on_camera(wrong_before);
    const ConeObservations colored = perception.on_lidar(scan_of(scene, 3, 1.0), 0.0);
    std::size_t orange = 0;
    for (const ConeObservation& cone : colored.cones) orange += cone.color == ConeColor::Orange;
    EXPECT_EQ(orange, 0U);
    EXPECT_GT(perception.stats().colored_by_camera, 0U);

    // A scan long after the last frame: no colors rather than stale ones.
    const ConeObservations stale = perception.on_lidar(scan_of(scene, 4, 2.0), 0.0);
    ASSERT_FALSE(stale.cones.empty());
    for (const ConeObservation& cone : stale.cones) EXPECT_EQ(cone.color, ConeColor::Unknown);
    EXPECT_EQ(perception.stats().scans_without_camera, 1U);
    EXPECT_EQ(perception.stats().camera_frames, 3U);
}

TEST(ConePerception, ReturnsNothingForAnEmptyScanOrBareGround) {
    const Scene scene;
    ConePerception perception(PerceptionConfig{}, scene.rig);
    LidarScan empty;
    empty.stamp = 2.5;
    const ConeObservations none = perception.on_lidar(empty, 0.0);
    EXPECT_EQ(none.stamp, 2.5);
    EXPECT_TRUE(none.cones.empty());
    EXPECT_EQ(perception.stats().ground_fallbacks, 1U);

    const ConeObservations ground = perception.on_lidar(scan_of(scene, 5, 2.6), 0.0);
    EXPECT_TRUE(ground.cones.empty());
    EXPECT_EQ(perception.stats().scans, 2U);
    EXPECT_EQ(perception.stats().lidar_time.count(), 2U);
    EXPECT_EQ(perception.stats().ground_fallbacks, 1U);
}

TEST(ConePerception, RejectsFarAwayNoise) {
    Scene scene;  // no cones: every return off the ground is clutter
    LidarScan scan = scan_of(scene, 6);
    Rng rng(6);
    for (int i = 0; i < 300; ++i) {
        scan.points.push_back({static_cast<float>(rng.uniform(5.0, 40.0)),
                               static_cast<float>(rng.uniform(-20.0, 20.0)),
                               static_cast<float>(rng.uniform(-0.15, 1.0)), -1});
    }
    ConePerception perception(PerceptionConfig{}, scene.rig);
    EXPECT_TRUE(perception.on_lidar(scan, 0.0).cones.empty());
}

TEST(ConePerception, KeepsAdjacentConesApart) {
    Scene scene;
    // The timing line's big cones, 1 m apart, and two small cones 0.7 m apart (center to center).
    scene.cones = {{0, {10.0, -0.5}, ConeColor::BigOrange},
                   {1, {10.0, 0.5}, ConeColor::BigOrange},
                   {2, {7.0, 3.0}, ConeColor::Blue},
                   {3, {7.0, 3.7}, ConeColor::Blue}};
    ConePerception perception(PerceptionConfig{}, scene.rig);
    const ConeObservations observations = perception.on_lidar(scan_of(scene, 7), 0.0);
    ASSERT_EQ(observations.cones.size(), 4U);
    for (const ConeObservation& cone : observations.cones) {
        EXPECT_LT((cone.position - truth_in_body(scene, cone.truth_id)).norm(), 0.05);
    }
}

TEST(ConePerception, HandlesOccludedAndPartlyVisibleCones) {
    Scene scene;
    const Vec2 lidar = scene.rig.lidar.mount.head<2>();
    // Cone 1 hides completely behind cone 0; cone 2 is half hidden behind its edge.
    scene.cones = {{0, lidar + Vec2(6.0, 0.0), ConeColor::Yellow},
                   {1, lidar + Vec2(12.0, 0.0), ConeColor::Yellow},
                   {2, lidar + Vec2(12.0, -0.23), ConeColor::Blue}};
    const LidarScan scan = scan_of(scene, 8);
    const auto returns = returns_per_cone(scan);
    ASSERT_FALSE(returns.contains(1));
    ConePerception perception(PerceptionConfig{}, scene.rig);
    const ConeObservations observations = perception.on_lidar(scan, 0.0);
    ASSERT_EQ(observations.cones.size(), 2U);
    EXPECT_EQ(observations.cones[0].truth_id, 0);
    EXPECT_EQ(observations.cones[1].truth_id, 2);
    EXPECT_LT((observations.cones[1].position - scene.cones[2].position).norm(), 0.06);
    // Fewer returns than an unoccluded cone at that range: a larger covariance.
    LidarScan clear_scan;
    {
        Scene alone;
        alone.cones = {scene.cones[2]};
        clear_scan = scan_of(alone, 8);
    }
    ConePerception clear(PerceptionConfig{}, scene.rig);
    const ConeObservations unoccluded = clear.on_lidar(clear_scan, 0.0);
    ASSERT_EQ(unoccluded.cones.size(), 1U);
    EXPECT_GT(observations.cones[1].covariance.trace(), unoccluded.cones[0].covariance.trace());
}

TEST(ConePerception, DoesNotNeedTheExactMountHeight) {
    const Scene scene = track_scene();
    // The rig says 0.20 m; the LiDAR really sits 0.30 m up (a different car, a heavy pitch).
    ConePerception perception(PerceptionConfig{}, scene.rig);
    LidarSimOptions options;
    options.sensor_height = 0.30;
    options.range_sigma = 0.02;
    const LidarScan scan = scan_of(scene, 9, 0.0, options);
    const ConeObservations observations = perception.on_lidar(scan, 0.0);
    std::size_t found = 0;
    for (const ConeObservation& cone : observations.cones) {
        ASSERT_GE(cone.truth_id, 0);
        EXPECT_LT((cone.position - truth_in_body(scene, cone.truth_id)).norm(), 0.10);
        found += truth_in_body(scene, cone.truth_id).norm() < 20.0;
    }
    EXPECT_GE(found, 8U);
    EXPECT_EQ(perception.stats().ground_fallbacks, 0U);
}

TEST(ConePerception, IgnoresNonFiniteReturns) {
    const Scene scene = track_scene();
    const LidarScan clean = scan_of(scene, 10);
    LidarScan dirty = clean;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    dirty.points.insert(dirty.points.begin() + 100, {nan, 1.0F, 0.0F, 3});
    dirty.points.push_back({5.0F, std::numeric_limits<float>::infinity(), 0.0F, -1});
    ConePerception a(PerceptionConfig{}, scene.rig);
    ConePerception b(PerceptionConfig{}, scene.rig);
    const ConeObservations expected = a.on_lidar(clean, 0.0);
    const ConeObservations got = b.on_lidar(dirty, 0.0);
    ASSERT_EQ(got.cones.size(), expected.cones.size());
    for (std::size_t i = 0; i < got.cones.size(); ++i) {
        EXPECT_EQ(got.cones[i].position, expected.cones[i].position);
    }
    EXPECT_EQ(b.stats().invalid_points, 2U);
}

// Equal inputs give bit-identical outputs, across instances and repeated calls.
TEST(ConePerception, IsDeterministic) {
    const Scene scene = track_scene();
    const LidarScan scan = scan_of(scene, 11);
    const CameraFrame frame = camera_of(scene, 11);
    ConePerception a(PerceptionConfig{}, scene.rig);
    ConePerception b(PerceptionConfig{}, scene.rig);
    a.on_camera(frame);
    b.on_camera(frame);
    (void)b.on_lidar(scan_of(scene, 99), 0.3);  // b has seen another scan before
    const ConeObservations first = a.on_lidar(scan, 0.3);
    const ConeObservations second = b.on_lidar(scan, 0.3);
    ASSERT_EQ(first.cones.size(), second.cones.size());
    for (std::size_t i = 0; i < first.cones.size(); ++i) {
        EXPECT_EQ(first.cones[i].position, second.cones[i].position);
        EXPECT_EQ(first.cones[i].covariance, second.cones[i].covariance);
        EXPECT_EQ(first.cones[i].color, second.cones[i].color);
        EXPECT_EQ(first.cones[i].color_confidence, second.cones[i].color_confidence);
        EXPECT_EQ(first.cones[i].truth_id, second.cones[i].truth_id);
    }
}

// Truth ids are for the evaluation only: scrambling them changes nothing else.
TEST(ConePerception, NeverDecidesOnTruthIds) {
    const Scene scene = track_scene();
    const LidarScan scan = scan_of(scene, 12);
    const CameraFrame frame = camera_of(scene, 12);
    LidarScan scrambled_scan = scan;
    CameraFrame scrambled_frame = frame;
    for (LidarPoint& p : scrambled_scan.points) p.truth_id = (p.truth_id * 7919 + 13) % 101;
    for (CameraDetection& d : scrambled_frame.detections) d.truth_id = -d.truth_id - 5;
    ConePerception a(PerceptionConfig{}, scene.rig);
    ConePerception b(PerceptionConfig{}, scene.rig);
    a.on_camera(frame);
    b.on_camera(scrambled_frame);
    const ConeObservations first = a.on_lidar(scan, 0.0);
    const ConeObservations second = b.on_lidar(scrambled_scan, 0.0);
    ASSERT_EQ(first.cones.size(), second.cones.size());
    for (std::size_t i = 0; i < first.cones.size(); ++i) {
        EXPECT_EQ(first.cones[i].position, second.cones[i].position);
        EXPECT_EQ(first.cones[i].covariance, second.cones[i].covariance);
        EXPECT_EQ(first.cones[i].color, second.cones[i].color);
        EXPECT_EQ(first.cones[i].color_confidence, second.cones[i].color_confidence);
    }
}

TEST(PerceptionConfig, RoundTripsThroughJson) {
    PerceptionConfig config;
    config.ground.threshold = 0.05;
    config.clustering.cell_size = 0.2;
    config.cone_check.min_points = 3;
    config.covariance.tangential.range_sq = 1e-6;
    config.camera.buffer_frames = 4;
    const nlohmann::json j = config;
    const PerceptionConfig back = j.get<PerceptionConfig>();
    EXPECT_EQ(nlohmann::json(back), j);
    EXPECT_EQ(back.ground.threshold, 0.05);
    EXPECT_EQ(back.covariance.tangential.range_sq, 1e-6);
    EXPECT_EQ(back.camera.buffer_frames, 4U);
}

TEST(PerceptionConfig, KeepsDefaultsForMissingKeysAndRejectsUnknownOnes) {
    const auto partial = nlohmann::json::parse(R"({"ground": {"threshold": 0.06}})");
    const PerceptionConfig config = partial.get<PerceptionConfig>();
    EXPECT_EQ(config.ground.threshold, 0.06);
    EXPECT_EQ(config.ground.seed_cell_size, GroundConfig{}.seed_cell_size);
    EXPECT_EQ(config.camera.gate, CameraFusionConfig{}.gate);

    EXPECT_THROW((void)nlohmann::json::parse(R"({"grund": {}})").get<PerceptionConfig>(),
                 std::invalid_argument);
    EXPECT_THROW((void)nlohmann::json::parse(R"({"covariance": {"radial": {"konstant": 1}}})")
                     .get<PerceptionConfig>(),
                 std::invalid_argument);
}

TEST(PerceptionConfig, InvalidValuesThrowAtConstruction) {
    const SensorRig rig;
    PerceptionConfig config;
    config.covariance.radial.constant = 0.0;
    EXPECT_THROW(ConePerception(config, rig), std::invalid_argument);
    config = PerceptionConfig{};
    config.center_fit.range_sigma = -1.0;
    EXPECT_THROW(ConePerception(config, rig), std::invalid_argument);
    config = PerceptionConfig{};
    config.clustering.max_range = 0.0;
    EXPECT_THROW(ConePerception(config, rig), std::invalid_argument);
    EXPECT_NO_THROW(ConePerception(PerceptionConfig{}, rig));
}

}  // namespace
}  // namespace apex::perception
