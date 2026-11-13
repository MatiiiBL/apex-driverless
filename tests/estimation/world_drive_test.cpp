#include "apex/estimation/eval/world_drive.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "apex/estimation/estimator.hpp"
#include "apex/estimation/eval/consistency.hpp"
#include "apex/estimation/eval/map_metrics.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::estimation::eval {
namespace {

// A lap of exploration and a racing lap on a generated track, driven by the simulator's vehicle
// model, with its IMU, wheel and steering models. The racing limits are what the pure-pursuit
// test driver holds without sliding (10 m/s^2 lateral).
SyntheticRun simulated_drive(const Track& track, const sim::WorldConfig& config) {
    DriveProfile profile;
    profile.laps = 2;
    profile.racing_lateral_accel = 10.0;
    profile.max_brake = 8.0;
    profile.max_accel = 6.0;
    return make_world_run(track, config, profile, SyntheticSensors{});
}

TEST(StateEstimator, TracksTheSimulatedCarOnAGeneratedTrack) {
    const Track track = sim::generate_track(1);
    sim::WorldConfig config;
    config.seed = 1;
    const SyntheticRun run = simulated_drive(track, config);
    ASSERT_TRUE(std::isfinite(run.lap1_end));

    // The velocity filter alone, and the whole estimator, on the same messages.
    VelocityFilter filter({}, config.vehicle);
    StateEstimator estimator({}, config.vehicle, config.rig, track.start_pose);
    NeesAccumulator nees(3);
    double velocity_sum[3] = {};
    double samples = 0.0;
    Time next_nees = 2.0;
    double lap1_sum = 0.0;
    double racing_sum = 0.0;
    double lap1_scans = 0.0;
    double racing_scans = 0.0;
    ConeMap lap1_map;
    for (const TimedMessage& m : run.messages) {
        if (estimator.mapping() && m.delivery >= run.lap1_end + 1.0) {
            lap1_map = estimator.map();
            estimator.set_mapping(false);
        }
        deliver(estimator, m);
        if (const auto* imu = std::get_if<ImuSample>(&m.message)) {
            filter.predict(*imu);
            const TruthSample& t = run.truth_at(imu->stamp);
            if (imu->stamp < 2.0) continue;
            const Vec3 e(filter.vx() - t.vx, filter.vy() - t.vy, filter.yaw_rate() - t.yaw_rate);
            for (int i = 0; i < 3; ++i) velocity_sum[i] += e[i] * e[i];
            samples += 1.0;
            if (imu->stamp >= next_nees) {
                nees.add(eval::nees(e, filter.velocity_covariance()));
                next_nees += 2.0;
            }
        } else if (const auto* wheels = std::get_if<WheelSpeeds>(&m.message)) {
            filter.update_wheels(*wheels);
        } else if (const auto* steering = std::get_if<SteeringSample>(&m.message)) {
            filter.update_steering(*steering);
        } else {
            const StateEstimate e = estimator.estimate();
            const double error2 =
                (e.pose.position() - run.truth_at(e.stamp).pose.position()).squaredNorm();
            (estimator.mapping() ? lap1_sum : racing_sum) += error2;
            (estimator.mapping() ? lap1_scans : racing_scans) += 1.0;
        }
    }

    EXPECT_LT(std::sqrt(velocity_sum[0] / samples), 0.05);   // vx, m/s
    EXPECT_LT(std::sqrt(velocity_sum[1] / samples), 0.08);   // vy, m/s
    EXPECT_LT(std::sqrt(velocity_sum[2] / samples), 0.004);  // yaw rate, rad/s
    // On the simulator's sensors the default noise model is somewhat conservative; it must
    // never be overconfident.
    const NeesSummary s = nees.summary();
    EXPECT_LT(s.mean, nees_bounds(3, s.samples).upper);

    std::vector<std::int32_t> mappable;
    for (std::size_t i = 0; i < run.mappable_since.size(); ++i) {
        if (run.mappable_since[i] < run.lap1_end - 2.0) {
            mappable.push_back(static_cast<std::int32_t>(i));
        }
    }
    const MapMetrics map = evaluate_map(lap1_map, track, mappable);
    EXPECT_EQ(map.duplicates, 0U);
    EXPECT_EQ(map.spurious, 0U);
    EXPECT_EQ(map.missed, 0U);
    EXPECT_LT(map.rms_error, 0.13);
    EXPECT_LT(std::sqrt(lap1_sum / lap1_scans), 0.15);
    EXPECT_LT(std::sqrt(racing_sum / racing_scans), 0.15);
    EXPECT_EQ(estimator.map().cones.size(), lap1_map.cones.size());
}

}  // namespace
}  // namespace apex::estimation::eval
