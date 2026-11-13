#include "apex/estimation/eval/synthetic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/LU>
#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/track.hpp"

namespace apex::estimation::eval {
namespace {

// Two laps of the oval, shared by the tests below (generating it is the slow part).
const SyntheticRun& oval_run() {
    static const SyntheticRun run = [] {
        DriveProfile profile;
        profile.laps = 2;
        return make_synthetic_run(make_oval_track(), VehicleParams{}, profile, SyntheticSensors{},
                                  11);
    }();
    return run;
}

TEST(SyntheticRun, TheCarFollowsTheCenterline) {
    const Track track = make_oval_track();
    const SyntheticRun& run = oval_run();
    ASSERT_GT(run.truth.size(), 1000U);
    double max_offset = 0.0;
    for (const TruthSample& t : run.truth) {
        max_offset = std::max(max_offset, track.centerline.project(t.pose.position()).distance);
    }
    EXPECT_LT(max_offset, 0.05);
    EXPECT_TRUE(std::isfinite(run.lap1_end));
    EXPECT_EQ(run.truth.front().lap, 1);
    EXPECT_EQ(run.truth.back().lap, 2);
    EXPECT_EQ(run.truth_at(run.lap1_end + 0.01).lap, 2);
}

TEST(SyntheticRun, VelocitiesAreTheDerivativesOfThePose) {
    const SyntheticRun& run = oval_run();
    for (std::size_t k = 1; k + 1 < run.truth.size(); k += 37) {
        const TruthSample& a = run.truth[k - 1];
        const TruthSample& b = run.truth[k + 1];
        const TruthSample& t = run.truth[k];
        const double dt = b.stamp - a.stamp;
        const Vec2 world_velocity = (b.pose.position() - a.pose.position()) / dt;
        const Vec2 body = t.pose.to_local(t.pose.position() + world_velocity);
        EXPECT_NEAR(body.x(), t.vx, 0.02) << t.stamp;
        EXPECT_NEAR(body.y(), t.vy, 0.02) << t.stamp;
        EXPECT_NEAR(angle_diff(b.pose.yaw, a.pose.yaw) / dt, t.yaw_rate, 1e-3) << t.stamp;
    }
}

TEST(SyntheticRun, IntegratingTheSpecificForceReproducesTheVelocity) {
    // vx' = ax + r vy, vy' = ay - r vx: the same kinematics the velocity filter predicts with.
    const SyntheticRun& run = oval_run();
    double vx = run.truth.front().vx;
    double vy = run.truth.front().vy;
    double max_error = 0.0;
    for (std::size_t k = 1; k < run.truth.size(); ++k) {
        const TruthSample& t0 = run.truth[k - 1];
        const TruthSample& t1 = run.truth[k];
        const double dt = t1.stamp - t0.stamp;
        const double ax = 0.5 * (t0.ax + t1.ax);
        const double ay = 0.5 * (t0.ay + t1.ay);
        const double r = 0.5 * (t0.yaw_rate + t1.yaw_rate);
        const double vx_next = vx + dt * (ax + r * vy);
        vy += dt * (ay - r * vx);
        vx = vx_next;
        max_error = std::max(max_error, std::hypot(vx - t1.vx, vy - t1.vy));
    }
    EXPECT_LT(max_error, 0.1);
}

TEST(SyntheticRun, MessagesArriveInDeliveryOrderWithTheLatency) {
    const SyntheticSensors sensors;
    const SyntheticRun& run = oval_run();
    std::size_t imu = 0;
    std::size_t scans = 0;
    for (std::size_t i = 0; i < run.messages.size(); ++i) {
        const TimedMessage& m = run.messages[i];
        if (i > 0) {
            EXPECT_GE(m.delivery, run.messages[i - 1].delivery);
        }
        if (const auto* cones = std::get_if<ConeObservations>(&m.message)) {
            ++scans;
            EXPECT_NEAR(m.delivery - cones->stamp, sensors.latency, 1e-9);
        } else if (std::holds_alternative<ImuSample>(m.message)) {
            ++imu;
        }
    }
    const double duration = run.truth.back().stamp;
    EXPECT_NEAR(static_cast<double>(imu), duration * 200.0, 2.0);
    EXPECT_NEAR(static_cast<double>(scans), duration * 10.0, 2.0);
}

TEST(SyntheticRun, ObservationsAreTheTrueConesWithTheReportedNoise) {
    const Track track = make_oval_track();
    const SyntheticSensors sensors;
    const SyntheticRun& run = oval_run();
    std::size_t true_cones = 0;
    std::size_t false_positives = 0;
    std::size_t scans = 0;
    double nis_sum = 0.0;
    for (const TimedMessage& m : run.messages) {
        const auto* cones = std::get_if<ConeObservations>(&m.message);
        if (cones == nullptr) continue;
        ++scans;
        const TruthSample& truth = run.truth_at(cones->stamp);
        for (const ConeObservation& obs : cones->cones) {
            EXPECT_LE(obs.position.norm(), sensors.cone_range + 2.0);
            if (obs.truth_id < 0) {
                ++false_positives;
                continue;
            }
            ++true_cones;
            const Vec2 expected =
                truth.pose.to_local(track.cones[static_cast<std::size_t>(obs.truth_id)].position);
            const Vec2 e = obs.position - expected;
            nis_sum += e.dot(obs.covariance.inverse() * e);
        }
    }
    ASSERT_GT(true_cones, 1000U);
    // The reported covariance is the one the noise is drawn from: the 2-D NIS averages 2.
    EXPECT_NEAR(nis_sum / static_cast<double>(true_cones), 2.0, 0.15);
    EXPECT_NEAR(static_cast<double>(false_positives) / static_cast<double>(scans),
                sensors.false_positive_rate, 0.1);
}

TEST(SyntheticRun, ConesBecomeMappableAfterThreeDetections) {
    const SyntheticRun& run = oval_run();
    std::size_t mappable = 0;
    for (const Time t : run.mappable_since) {
        if (std::isfinite(t)) ++mappable;
    }
    // Every cone of the oval is passed within sensor range on the first lap.
    EXPECT_EQ(mappable, make_oval_track().cones.size());
}

TEST(SyntheticRun, IsDeterministicInItsSeed) {
    const Track track = make_oval_track();
    DriveProfile profile;
    profile.hold_at_start = 0.2;
    const SyntheticRun a = make_synthetic_run(track, VehicleParams{}, profile, {}, 5);
    const SyntheticRun b = make_synthetic_run(track, VehicleParams{}, profile, {}, 5);
    const SyntheticRun c = make_synthetic_run(track, VehicleParams{}, profile, {}, 6);
    ASSERT_EQ(a.messages.size(), b.messages.size());
    bool differs = false;
    for (std::size_t i = 0; i < a.messages.size(); ++i) {
        const auto* ia = std::get_if<ImuSample>(&a.messages[i].message);
        const auto* ib = std::get_if<ImuSample>(&b.messages[i].message);
        const auto* ic =
            i < c.messages.size() ? std::get_if<ImuSample>(&c.messages[i].message) : nullptr;
        if (ia == nullptr) continue;
        ASSERT_NE(ib, nullptr);
        EXPECT_EQ(ia->yaw_rate, ib->yaw_rate);
        if (ic != nullptr && ic->yaw_rate != ia->yaw_rate) differs = true;
    }
    EXPECT_TRUE(differs);
}

TEST(SyntheticRun, RejectsAProfileThatCannotBeDriven) {
    DriveProfile profile;
    profile.max_brake = 0.0;
    EXPECT_THROW((void)make_synthetic_run(make_oval_track(), VehicleParams{}, profile, {}, 1),
                 std::invalid_argument);
    profile = {};
    profile.laps = 0;
    EXPECT_THROW((void)make_synthetic_run(make_oval_track(), VehicleParams{}, profile, {}, 1),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apex::estimation::eval
