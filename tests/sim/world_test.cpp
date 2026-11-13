#include "apex/sim/world.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::sim {
namespace {

// Field-by-field equality of sensor messages, bit for bit.
bool same(const SensorMessage& a, const SensorMessage& b) {
    if (a.index() != b.index()) return false;
    return std::visit(
        [&b](const auto& x) {
            using T = std::decay_t<decltype(x)>;
            const T& y = std::get<T>(b);
            if constexpr (std::is_same_v<T, ImuSample>) {
                return x.stamp == y.stamp && x.ax == y.ax && x.ay == y.ay &&
                       x.yaw_rate == y.yaw_rate;
            } else if constexpr (std::is_same_v<T, WheelSpeeds>) {
                return x.stamp == y.stamp && x.front == y.front && x.rear == y.rear;
            } else if constexpr (std::is_same_v<T, SteeringSample>) {
                return x.stamp == y.stamp && x.angle == y.angle;
            } else if constexpr (std::is_same_v<T, LidarScan>) {
                if (x.stamp != y.stamp || x.points.size() != y.points.size()) return false;
                for (std::size_t i = 0; i < x.points.size(); ++i) {
                    const LidarPoint& p = x.points[i];
                    const LidarPoint& q = y.points[i];
                    if (p.x != q.x || p.y != q.y || p.z != q.z || p.truth_id != q.truth_id) {
                        return false;
                    }
                }
                return true;
            } else {
                if (x.stamp != y.stamp || x.detections.size() != y.detections.size()) return false;
                for (std::size_t i = 0; i < x.detections.size(); ++i) {
                    const CameraDetection& p = x.detections[i];
                    const CameraDetection& q = y.detections[i];
                    if (p.bearing != q.bearing || p.range != q.range || p.color != q.color ||
                        p.confidence != q.confidence || p.truth_id != q.truth_id) {
                        return false;
                    }
                }
                return true;
            }
        },
        a);
}

// A simple truth-fed driver for closed-loop tests: pure pursuit on the centerline with a 6 m
// lookahead (Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm",
// CMU-RI-TR-92-01, 1992) and a proportional speed loop over a drag feed-forward.
ControlCommand drive(const World& world, double speed) {
    const VehicleState& s = world.truth();
    const Trajectory& center = world.track().centerline;
    const Trajectory::Projection here = center.project(s.pose.position());
    const Vec2 target = center.at(here.s + 6.0).position;
    const Vec2 local = s.pose.to_local(target);
    const double L = world.config().vehicle.wheelbase();
    const double steer = std::atan(2.0 * L * local.y() / local.squaredNorm());
    const VehicleParams& p = world.config().vehicle;
    const double feed_forward =
        (p.drag(s.vx) + p.rolling_resistance * (p.mass * kGravity + p.downforce(s.vx))) / p.mass;
    return {world.time(), steer, feed_forward + 2.0 * (speed - s.vx)};
}

TEST(World, StartsAtRestOnTheStagingPosition) {
    const Track track = make_oval_track();
    World world(track, WorldConfig{});
    EXPECT_EQ(world.tick(), 0);
    EXPECT_EQ(world.time(), 0.0);
    EXPECT_EQ(world.truth().pose.x, track.start_pose.x);
    EXPECT_EQ(world.truth().pose.y, track.start_pose.y);
    EXPECT_EQ(world.truth().pose.yaw, track.start_pose.yaw);
    EXPECT_EQ(world.truth().vx, 0.0);
    EXPECT_EQ(world.track().cones.size(), track.cones.size());
    EXPECT_NEAR(world.race().progress(), -6.0, 0.01);
    for (int i = 0; i < 1000; ++i) (void)world.step();
    EXPECT_EQ(world.truth().pose.x, track.start_pose.x);
    EXPECT_EQ(world.truth().vx, 0.0);
    EXPECT_FALSE(world.race().started());
}

TEST(World, RejectsInvalidConfiguration) {
    const Track track = make_oval_track();
    auto rejects = [&track](auto mutate) {
        WorldConfig config;
        mutate(config);
        EXPECT_THROW(World(track, config), std::invalid_argument);
    };
    rejects([](WorldConfig& c) { c.rig.imu_rate_hz = 300.0; });  // 3.33 ms period
    rejects([](WorldConfig& c) { c.rig.wheel_rate_hz = 0.0; });
    rejects([](WorldConfig& c) { c.rig.lidar.rate_hz = 3.0; });
    rejects([](WorldConfig& c) { c.rig.camera.rate_hz = 2000.0; });  // faster than the tick
    rejects([](WorldConfig& c) { c.rig.lidar.elevations.clear(); });
    rejects([](WorldConfig& c) { c.rig.lidar.azimuth_step = 0.0; });
    rejects([](WorldConfig& c) { c.rig.camera.max_range = -1.0; });
    rejects([](WorldConfig& c) { c.noise.imu.accel_stddev = -0.1; });
    rejects([](WorldConfig& c) { c.vehicle.mass = 0.0; });
    rejects([](WorldConfig& c) { c.friction_scale = 0.0; });
    Track shuffled = track;
    std::swap(shuffled.cones[0], shuffled.cones[1]);
    EXPECT_THROW(World(shuffled, WorldConfig{}), std::invalid_argument);
    WorldConfig slower;
    slower.rig.imu_rate_hz = 250.0;
    slower.rig.lidar.rate_hz = 20.0;
    EXPECT_NO_THROW(World(track, slower));
}

// Each sensor fires on the ticks its rate divides, stamped with the tick's time, in the order
// IMU, wheel speeds, steering, camera, LiDAR.
TEST(World, FiresSensorsOnExactTicksInFixedOrder) {
    World world(make_oval_track(), WorldConfig{});
    std::array<int, 5> counts{};
    for (int i = 1; i <= 1000; ++i) {
        const std::vector<SensorMessage> messages = world.step();
        ASSERT_EQ(world.tick(), i);
        std::vector<std::size_t> kinds;
        for (const SensorMessage& m : messages) {
            EXPECT_EQ(stamp_of(m), world.time());
            kinds.push_back(m.index());
            ++counts[m.index()];
        }
        std::vector<std::size_t> expected;
        if (i % 5 == 0) expected.push_back(0);    // ImuSample
        if (i % 10 == 0) expected.push_back(1);   // WheelSpeeds
        if (i % 10 == 0) expected.push_back(2);   // SteeringSample
        if (i % 50 == 0) expected.push_back(4);   // CameraFrame
        if (i % 100 == 0) expected.push_back(3);  // LidarScan
        ASSERT_EQ(kinds, expected) << "tick " << i;
    }
    EXPECT_EQ(counts, (std::array<int, 5>{200, 100, 100, 10, 20}));
    EXPECT_DOUBLE_EQ(world.time(), 1.0);
}

TEST(World, SameInputsGiveIdenticalMessages) {
    const Track track = generate_track(5);
    WorldConfig config;
    config.seed = 99;
    World a(track, config);
    World b(track, config);
    for (int i = 0; i < 4000; ++i) {
        if (i % 20 == 0) {
            const ControlCommand command = drive(a, 8.0);
            a.command(command);
            b.command(command);
        }
        const std::vector<SensorMessage> ma = a.step();
        const std::vector<SensorMessage> mb = b.step();
        ASSERT_EQ(ma.size(), mb.size());
        for (std::size_t k = 0; k < ma.size(); ++k) ASSERT_TRUE(same(ma[k], mb[k])) << i;
    }
    EXPECT_EQ(a.truth().pose.x, b.truth().pose.x);
    EXPECT_EQ(a.truth().pose.y, b.truth().pose.y);
    EXPECT_EQ(a.truth().vx, b.truth().vx);
    EXPECT_GT(a.truth().vx, 6.0);

    // Another seed, other noise: the first IMU samples differ.
    World c(track, config);
    config.seed = 100;
    World d(track, config);
    for (int i = 0; i < 4; ++i) {
        (void)c.step();
        (void)d.step();
    }
    const std::vector<SensorMessage> mc = c.step();
    const std::vector<SensorMessage> md = d.step();
    ASSERT_FALSE(mc.empty());
    ASSERT_EQ(mc.size(), md.size());
    EXPECT_FALSE(same(mc[0], md[0]));
}

// Every sensor draws from its own stream: changing one sensor's noise leaves the others'
// measurements bit-identical.
TEST(World, SensorsDrawFromIndependentStreams) {
    const Track track = make_oval_track();
    WorldConfig noisier;
    noisier.noise.wheels.stddev *= 3.0;
    noisier.noise.steering.stddev = 0.0;
    World a(track, WorldConfig{});
    World b(track, noisier);
    int compared = 0;
    int wheels_differ = 0;
    for (int i = 0; i < 500; ++i) {
        const std::vector<SensorMessage> ma = a.step();
        const std::vector<SensorMessage> mb = b.step();
        ASSERT_EQ(ma.size(), mb.size());
        for (std::size_t k = 0; k < ma.size(); ++k) {
            if (std::holds_alternative<ImuSample>(ma[k])) {
                EXPECT_TRUE(same(ma[k], mb[k]));
                ++compared;
            } else if (std::holds_alternative<WheelSpeeds>(ma[k]) && !same(ma[k], mb[k])) {
                ++wheels_differ;
            }
        }
    }
    EXPECT_EQ(compared, 100);
    // At rest, readings whose noise is negative clamp to zero in both worlds; the rest differ.
    EXPECT_GT(wheels_differ, 25);
}

// From the staging position the LiDAR and camera see the cones ahead, tagged with real ids.
TEST(World, LidarAndCameraSeeTheTrack) {
    const Track track = generate_track(8);
    World world(track, WorldConfig{});
    int cone_returns = 0;
    int detections = 0;
    for (int i = 0; i < 100; ++i) {
        for (const SensorMessage& m : world.step()) {
            if (const auto* scan = std::get_if<LidarScan>(&m)) {
                for (const LidarPoint& p : scan->points) {
                    if (p.truth_id < 0) continue;
                    ASSERT_LT(static_cast<std::size_t>(p.truth_id), track.cones.size());
                    ++cone_returns;
                }
            } else if (const auto* frame = std::get_if<CameraFrame>(&m)) {
                for (const CameraDetection& d : frame->detections) {
                    ASSERT_LT(d.truth_id, static_cast<std::int32_t>(track.cones.size()));
                    if (d.truth_id >= 0) ++detections;
                }
            }
        }
    }
    EXPECT_GT(cone_returns, 200);
    EXPECT_GT(detections, 10);
}

// Closed loop on the truth: a lap of the oval with a simple tracker, timed by the RaceMonitor,
// with the proprioceptive sensors agreeing with the truth.
TEST(World, DrivesALapOfTheOval) {
    const Track track = make_oval_track();
    World world(track, WorldConfig{});
    double max_wheel_error = 0.0;
    while (world.race().laps_completed() < 1 && world.time() < 60.0) {
        if (world.tick() % 20 == 0) world.command(drive(world, 10.0));
        for (const SensorMessage& m : world.step()) {
            if (const auto* wheels = std::get_if<WheelSpeeds>(&m)) {
                max_wheel_error =
                    std::max(max_wheel_error, std::abs(wheels->rear - world.truth().vx));
            }
        }
    }
    ASSERT_EQ(world.race().laps_completed(), 1);
    const LapRecord& lap = world.race().laps()[0];
    // About length / 10 m/s once at speed, a little longer for the corners' cut.
    EXPECT_GT(lap.time(), 13.0);
    EXPECT_LT(lap.time(), 17.0);
    EXPECT_EQ(world.race().penalty_seconds(), 0.0);
    EXPECT_LT(max_wheel_error, 0.4);  // noise (0.05 m/s) plus slip
    EXPECT_NEAR(world.race().progress(), track.centerline.length(), 1.0);
}

}  // namespace
}  // namespace apex::sim
