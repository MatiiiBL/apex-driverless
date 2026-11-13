#include "apex/sim/race.hpp"

#include <cmath>
#include <stdexcept>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

namespace apex::sim {
namespace {

constexpr double kTick = 1e-3;

// The car at arc length s of the centerline, `lateral` to its left, facing along it (or
// against it).
VehicleState at(const Track& track, double s, double lateral = 0.0, bool reversed = false) {
    const TrajectoryPoint p = track.centerline.at(s);
    const Vec2 position = p.position + lateral * left_normal(heading_vector(p.yaw));
    VehicleState state;
    state.pose = {position.x(), position.y(), reversed ? wrap_angle(p.yaw + kPi) : p.yaw};
    return state;
}

// Scripted motions on the 40 m x 12 m oval (155 m), one update per millisecond.
class RaceMonitorTest : public testing::Test {
protected:
    Track track_ = make_oval_track();
    VehicleParams vehicle_;
    RaceMonitor monitor_{track_, vehicle_};
    double length_ = track_.centerline.length();
    Time time_ = 0.0;

    // Moves the CoG along the centerline from s0 to s1 (unwrapped, either way) at `speed`.
    void drive(double s0, double s1, double speed, double lateral = 0.0) {
        const double distance = std::abs(s1 - s0);
        const int ticks = static_cast<int>(std::ceil(distance / (speed * kTick)));
        for (int i = 0; i <= ticks; ++i) {
            const double s = s0 + (s1 - s0) * i / ticks;
            monitor_.update(at(track_, s, lateral, s1 < s0), time_);
            time_ += distance / speed / ticks;
        }
    }
};

TEST_F(RaceMonitorTest, TimesLapsBetweenForwardCrossings) {
    const double start = length_ - 6.0;  // the staging position
    monitor_.update(at(track_, start), 0.0);
    EXPECT_FALSE(monitor_.started());
    EXPECT_NEAR(monitor_.progress(), -6.0, 1e-6);
    drive(start, start + 2.5 * length_, 10.0);
    EXPECT_TRUE(monitor_.started());
    ASSERT_EQ(monitor_.laps_completed(), 2);
    for (const LapRecord& lap : monitor_.laps()) {
        EXPECT_NEAR(lap.time(), length_ / 10.0, 1e-6);
    }
    EXPECT_EQ(monitor_.laps()[0].lap, 1);
    EXPECT_NEAR(monitor_.laps()[0].start, 0.6, 1e-6);
    EXPECT_EQ(monitor_.laps()[1].start, monitor_.laps()[0].end);
    EXPECT_NEAR(monitor_.progress(), 2.5 * length_ - 6.0, 1e-6);
    EXPECT_NEAR(monitor_.s(), track_.centerline.wrap_s(start + 2.5 * length_), 1e-6);
    EXPECT_EQ(monitor_.penalty_seconds(), 0.0);
}

// Rolling back and forth over the line starts the lap once; the lap still ends at the next
// real crossing, timed from the first.
TEST_F(RaceMonitorTest, HoveringOnTheLineCountsOnce) {
    drive(length_ - 3.0, length_ + 1.0, 5.0);  // crosses at 0.6 s
    for (int i = 0; i < 3; ++i) {
        drive(length_ + 1.0, length_ - 1.0, 2.0);
        drive(length_ - 1.0, length_ + 1.0, 2.0);
    }
    EXPECT_TRUE(monitor_.started());
    EXPECT_EQ(monitor_.laps_completed(), 0);
    drive(length_ + 1.0, 2.0 * length_ + 1.0, 10.0);
    ASSERT_EQ(monitor_.laps_completed(), 1);
    EXPECT_NEAR(monitor_.laps()[0].start, 0.6, 1e-6);
}

TEST_F(RaceMonitorTest, IgnoresCrossingsAgainstTheDrivingDirection) {
    drive(20.0, 20.0 - 2.0 * length_, 10.0);
    EXPECT_FALSE(monitor_.started());
    EXPECT_EQ(monitor_.laps_completed(), 0);
    EXPECT_NEAR(monitor_.progress(), 20.0 - 2.0 * length_, 1e-6);
}

// Through the plane of the timekeeping line but well beside the track: no light barrier there.
TEST_F(RaceMonitorTest, IgnoresCrossingsBesideTheLine) {
    drive(length_ - 5.0, length_ + 5.0, 10.0, 4.0);
    EXPECT_FALSE(monitor_.started());
}

TEST_F(RaceMonitorTest, CountsEachKnockedConeOnce) {
    const Cone& cone = track_.cones[10];
    const double radius = cone_shape(cone.color).base_radius;
    const double yaw = track_.centerline.at(track_.centerline.project(cone.position).s).yaw;
    auto with_nose_at = [&](double gap) {
        // The front of the body `gap` short of the cone's base, heading along the track.
        const Vec2 cog = cone.position - (vehicle_.body_front + radius + gap) * heading_vector(yaw);
        VehicleState state;
        state.pose = {cog.x(), cog.y(), yaw};
        return state;
    };
    monitor_.update(with_nose_at(0.01), 0.0);
    EXPECT_TRUE(monitor_.cone_hits().empty());
    for (int i = 1; i <= 100; ++i) monitor_.update(with_nose_at(-0.01), i * kTick);
    ASSERT_EQ(monitor_.cone_hits().size(), 1U);
    EXPECT_EQ(monitor_.cone_hits()[0].cone_id, 10);
    EXPECT_NEAR(monitor_.cone_hits()[0].time, kTick, 1e-12);
    EXPECT_EQ(monitor_.penalty_seconds(), 2.0);
}

TEST_F(RaceMonitorTest, DrivingTheCenterlineHitsNoCones) {
    drive(length_ - 6.0, 2.0 * length_, 15.0);
    EXPECT_TRUE(monitor_.cone_hits().empty());
    EXPECT_TRUE(monitor_.off_course().empty());
}

// With the 3.5 m track and 1.22 m wheel track, all four wheels are out once the CoG is more than
// 1.75 + 0.61 = 2.36 m from the centerline (the cone line it crosses is knocked down on the
// way, which is a separate penalty).
TEST_F(RaceMonitorTest, OffCourseNeedsAllFourWheelsOutside) {
    const double s = 20.0;
    monitor_.update(at(track_, s, 0.0), 0.0);
    monitor_.update(at(track_, s + 0.01, 2.2), kTick);
    EXPECT_TRUE(monitor_.off_course().empty());
    for (int i = 2; i < 50; ++i) monitor_.update(at(track_, s + 0.01 * i, 2.5), i * kTick);
    ASSERT_EQ(monitor_.off_course().size(), 1U);
    EXPECT_NEAR(monitor_.off_course()[0].time, 2.0 * kTick, 1e-12);
    // The offset point projects back onto the 0.5 m polyline of a curve only approximately.
    EXPECT_NEAR(monitor_.off_course()[0].s, s + 0.02, 0.05);
    EXPECT_NEAR(monitor_.lateral(), 2.5, 0.01);
    // Two wheels back on the track is not yet the end of the excursion; the CoG back is.
    monitor_.update(at(track_, s + 0.5, 1.8), 0.05);
    monitor_.update(at(track_, s + 0.51, 2.5), 0.051);
    EXPECT_EQ(monitor_.off_course().size(), 1U);
    monitor_.update(at(track_, s + 0.52, 1.0), 0.052);
    monitor_.update(at(track_, s + 0.53, -2.5), 0.053);
    EXPECT_EQ(monitor_.off_course().size(), 2U);
    EXPECT_EQ(monitor_.penalty_seconds(),
              20.0 + 2.0 * static_cast<double>(monitor_.cone_hits().size()));
}

TEST(RaceMonitor, RejectsATrackWithoutCenterline) {
    const Track empty;
    EXPECT_THROW(RaceMonitor(empty, VehicleParams{}), std::invalid_argument);
}

}  // namespace
}  // namespace apex::sim
