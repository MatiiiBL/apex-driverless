#include "apex/estimation/odometry.hpp"

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/estimation/dead_reckoning.hpp"
#include "apex/estimation/eval/consistency.hpp"
#include "apex/estimation/eval/synthetic.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::estimation {
namespace {

Vec3 as_vector(const Pose2& p) {
    return {p.x, p.y, p.yaw};
}

void expect_pose_near(const Pose2& a, const Pose2& b, double tolerance) {
    EXPECT_NEAR(a.x, b.x, tolerance);
    EXPECT_NEAR(a.y, b.y, tolerance);
    EXPECT_NEAR(angle_diff(a.yaw, b.yaw), 0.0, tolerance);
}

TEST(Odometry, IntegrateTwistIsTheExactArcOfAConstantTwist) {
    // Fine Euler steps converge to the exponential map.
    for (const double yaw_rate : {0.0, 1e-8, 0.4, -1.3}) {
        const double vx = 12.0;
        const double vy = -0.6;
        const double dt = 0.8;
        Pose2 euler;
        const int steps = 200000;
        for (int i = 0; i < steps; ++i) {
            euler = euler.compose({vx * dt / steps, vy * dt / steps, yaw_rate * dt / steps});
        }
        expect_pose_near(integrate_twist(vx, vy, yaw_rate, dt), euler, 1e-4);
    }
    // A full circle comes back to the start.
    const double r = 2.0 * kPi;
    expect_pose_near(integrate_twist(5.0, 0.0, r, 1.0), Pose2{}, 1e-12);
}

TEST(Odometry, ComposeMatchesNumericalJacobians) {
    RelativeMotion a{{1.0, 0.3, 0.4}, Mat3::Zero()};
    RelativeMotion b{{2.0, -0.5, -0.2}, Mat3::Zero()};
    a.covariance << 0.04, 0.01, 0.002, 0.01, 0.09, -0.001, 0.002, -0.001, 0.003;
    b.covariance << 0.02, 0.0, 0.001, 0.0, 0.03, 0.0, 0.001, 0.0, 0.004;
    const RelativeMotion c = compose(a, b);
    expect_pose_near(c.delta, a.delta.compose(b.delta), 1e-15);

    // Central differences of the composition with respect to each argument.
    const double h = 1e-6;
    Mat3 ja;
    Mat3 jb;
    for (int i = 0; i < 3; ++i) {
        Vec3 d = Vec3::Zero();
        d[i] = h;
        const auto shift = [](const Pose2& p, const Vec3& v) {
            return Pose2{p.x + v.x(), p.y + v.y(), p.yaw + v.z()};
        };
        Vec3 da = as_vector(shift(a.delta, d).compose(b.delta)) -
                  as_vector(shift(a.delta, -d).compose(b.delta));
        Vec3 db = as_vector(a.delta.compose(shift(b.delta, d))) -
                  as_vector(a.delta.compose(shift(b.delta, -d)));
        da.z() = wrap_angle(da.z());
        db.z() = wrap_angle(db.z());
        ja.col(i) = da / (2.0 * h);
        jb.col(i) = db / (2.0 * h);
    }
    const Mat3 expected = ja * a.covariance * ja.transpose() + jb * b.covariance * jb.transpose();
    EXPECT_LT((c.covariance - expected).cwiseAbs().maxCoeff(), 1e-9);
}

OdometryIncrement increment(Time end, double vx, double vy, double yaw_rate) {
    OdometryIncrement inc;
    inc.end = end;
    inc.vx = vx;
    inc.vy = vy;
    inc.yaw_rate = yaw_rate;
    inc.noise_density = Vec3(0.01, 0.02, 0.001).asDiagonal();
    return inc;
}

TEST(OdometryBuffer, TakingASplitPrefixLeavesTheRestPending) {
    OdometryBuffer buffer;
    buffer.reset(1.0);
    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(buffer.start(), 1.0);
    for (int i = 1; i <= 10; ++i) {
        buffer.push(increment(1.0 + 0.005 * i, 10.0 + i, 0.1, 0.3 - 0.05 * i));
    }
    EXPECT_EQ(buffer.size(), 10U);
    EXPECT_NEAR(buffer.end(), 1.05, 1e-12);
    const RelativeMotion whole = buffer.pending();

    // 1.0225 splits the fifth increment.
    const RelativeMotion first = buffer.take_until(1.0225);
    EXPECT_NEAR(buffer.start(), 1.0225, 1e-12);
    EXPECT_EQ(buffer.size(), 6U);
    const RelativeMotion rejoined = compose(first, buffer.pending());
    expect_pose_near(rejoined.delta, whole.delta, 1e-12);
    // Splitting an increment rotates part of its noise; the difference is second order.
    EXPECT_LT((rejoined.covariance - whole.covariance).cwiseAbs().maxCoeff(),
              1e-3 * whole.covariance.norm());

    // Beyond the end everything is taken.
    const RelativeMotion rest = buffer.take_until(5.0);
    expect_pose_near(compose(first, rest).delta, whole.delta, 1e-12);
    EXPECT_TRUE(buffer.empty());
    EXPECT_NEAR(buffer.start(), buffer.end(), 1e-15);
    EXPECT_EQ(buffer.pending().covariance, Mat3::Zero());
}

TEST(OdometryBuffer, KeepsItsSpanContiguous) {
    OdometryBuffer buffer;
    buffer.reset(0.0);
    buffer.push(increment(0.01, 1.0, 0.0, 0.0));
    buffer.push(increment(0.01, 1.0, 0.0, 0.0));   // empty: ignored
    buffer.push(increment(0.005, 1.0, 0.0, 0.0));  // backwards: ignored
    EXPECT_EQ(buffer.size(), 1U);
    // A gap is closed: the increment starts where the buffer ends.
    OdometryIncrement late = increment(0.03, 2.0, 0.0, 0.0);
    late.start = 0.02;
    buffer.push(late);
    EXPECT_EQ(buffer.size(), 2U);
    EXPECT_NEAR(buffer.pending().delta.x, 0.01 + 0.04, 1e-12);
    // Taking before the start takes nothing.
    const RelativeMotion none = buffer.take_until(-1.0);
    EXPECT_EQ(none.delta.x, 0.0);
    EXPECT_EQ(buffer.size(), 2U);
}

struct Drift {
    eval::NeesSummary nees;  // of the motion over each one-second window
    double rms_position_error = 0.0;
    double rms_yaw_error = 0.0;
};

// Dead reckoning over a run, cut into one-second windows: the odometry's motion over each
// window against the true one.
Drift dead_reckon(const eval::SyntheticRun& run, const VelocityFilterConfig& velocity,
                  const SlamConfig& slam) {
    DeadReckoning motion(velocity, slam, VehicleParams{});
    eval::NeesAccumulator nees(3);
    Time next = 3.0;
    Pose2 window_start;
    double sum_position = 0.0;
    double sum_yaw = 0.0;
    for (const eval::TimedMessage& m : run.messages) {
        if (const auto* imu = std::get_if<ImuSample>(&m.message)) {
            EXPECT_TRUE(motion.on_imu(*imu));
            if (imu->stamp >= next) {
                const eval::TruthSample& t = run.truth_at(imu->stamp);
                const RelativeMotion step = motion.odometry().take_until(imu->stamp);
                if (next > 3.0) {
                    Vec3 e = as_vector(step.delta) - as_vector(window_start.between(t.pose));
                    e.z() = wrap_angle(e.z());
                    nees.add(eval::nees(e, step.covariance));
                    sum_position += e.head<2>().squaredNorm();
                    sum_yaw += e.z() * e.z();
                }
                window_start = t.pose;
                next += 1.0;
            }
        } else if (const auto* wheels = std::get_if<WheelSpeeds>(&m.message)) {
            motion.on_wheels(*wheels);
        } else if (const auto* steering = std::get_if<SteeringSample>(&m.message)) {
            motion.on_steering(*steering);
        }
    }
    Drift d;
    d.nees = nees.summary();
    const auto n = static_cast<double>(std::max<std::size_t>(d.nees.samples, 1));
    d.rms_position_error = std::sqrt(sum_position / n);
    d.rms_yaw_error = std::sqrt(sum_yaw / n);
    return d;
}

TEST(DeadReckoning, MotionOverEachSecondIsAccurateAndNeverOverconfident) {
    // The SLAM corrects the pose every scan, so what its prediction needs is the motion over
    // short spans and an honest covariance for it. The covariance is deliberately conservative
    // (its floors stand for what the synthetic sensors lack: accelerometer bias, IMU offset); the
    // test requires that it never claims more than the odometry delivers.
    const eval::SyntheticRun run =
        eval::make_synthetic_run(sim::generate_track(4), VehicleParams{}, {}, {}, 4);
    const Drift d = dead_reckon(run, {}, {});
    ASSERT_GT(d.nees.samples, 40U);
    EXPECT_LT(d.nees.mean, eval::nees_bounds(3, d.nees.samples).upper);
    EXPECT_LT(d.rms_position_error, 0.01);  // m per second of driving
    EXPECT_LT(d.rms_yaw_error, 0.002);      // rad per second
}

TEST(DeadReckoning, StartsAtRestWhenTheFirstMessageIsNotAnImu) {
    DeadReckoning motion({}, {}, VehicleParams{});
    EXPECT_FALSE(motion.started());
    motion.start(2.0);
    EXPECT_TRUE(motion.started());
    motion.start(3.0);  // only the first call sets the time base
    EXPECT_EQ(motion.odometry().start(), 2.0);
    motion.extrapolate_to(2.5);
    EXPECT_NEAR(motion.odometry().end(), 2.5, 1e-15);
    EXPECT_NEAR(motion.odometry().pending().delta.x, 0.0, 1e-15);  // at rest
    EXPECT_GT(motion.odometry().pending().covariance(0, 0), 0.0);
}

}  // namespace
}  // namespace apex::estimation
