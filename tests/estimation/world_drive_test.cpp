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
SyntheticRun simulated_drive(const Track& track, const sim::WorldConfig& config, ConeSource source,
                             double hold_at_start = 1.0) {
    DriveProfile profile;
    profile.laps = 2;
    profile.racing_lateral_accel = 10.0;
    profile.max_brake = 8.0;
    profile.max_accel = 6.0;
    profile.hold_at_start = hold_at_start;
    return make_world_run(track, config, profile, SyntheticSensors{}, source);
}

struct Evaluation {
    Vec3 velocity_rms = Vec3::Zero();  // vx, vy, yaw rate
    NeesSummary velocity_nees;
    MapMetrics map;  // of the map at the switch to localization
    double lap1_position_rms = 0.0;
    double racing_position_rms = 0.0;
    std::size_t final_landmarks = 0;
};

// The velocity filter alone and the whole estimator on the same messages; mapping is switched
// off one second into lap 2, as the stack does once it has closed the loop.
Evaluation evaluate(const SyntheticRun& run, const Track& track, const sim::WorldConfig& config,
                    const EstimatorConfig& estimator_config = {}) {
    VelocityFilter filter(estimator_config.velocity, config.vehicle);
    StateEstimator estimator(estimator_config, config.vehicle, config.rig, track.start_pose);
    NeesAccumulator nees(3);
    Vec3 velocity_sum = Vec3::Zero();
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
            velocity_sum += e.cwiseAbs2();
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
    std::vector<std::int32_t> mappable;
    for (std::size_t i = 0; i < run.mappable_since.size(); ++i) {
        if (run.mappable_since[i] < run.lap1_end - 2.0) {
            mappable.push_back(static_cast<std::int32_t>(i));
        }
    }
    Evaluation out;
    out.velocity_rms = (velocity_sum / samples).cwiseSqrt();
    out.velocity_nees = nees.summary();
    out.map = evaluate_map(lap1_map, track, mappable);
    out.lap1_position_rms = std::sqrt(lap1_sum / lap1_scans);
    out.racing_position_rms = std::sqrt(racing_sum / racing_scans);
    out.final_landmarks = estimator.map().cones.size();
    return out;
}

TEST(StateEstimator, KeepsMappingAtThreeTimesTheSensorNoise) {
    // At three times the simulator's sensor noise, perception no longer finds the big orange
    // cones beside the staging position, and only six of the cones within 20 m remain. A car at
    // rest sees nothing come nearer: when new landmarks were limited to 20 m the map stayed at
    // those six, and the stack's planner found no path to explore. The far cones must be mapped
    // too, and over the drive the map must keep growing, with larger errors than at the nominal
    // noise but no duplicates (docs/estimation.md).
    const double hold = 3.5;  // s, when the stack gave up
    const Track track = sim::generate_track(1);
    sim::WorldConfig config;
    config.seed = 1;
    config.noise = sim::scale_noise(sim::SensorNoise{}, 3.0);
    const SyntheticRun run = simulated_drive(track, config, ConeSource::Perception, hold);
    ASSERT_TRUE(std::isfinite(run.lap1_end));

    EstimatorConfig near_only;
    near_only.slam.max_new_landmark_range = 20.0;
    StateEstimator staged({}, config.vehicle, config.rig, track.start_pose);
    StateEstimator staged_near(near_only, config.vehicle, config.rig, track.start_pose);
    for (const TimedMessage& m : run.messages) {
        if (m.delivery >= hold) break;
        deliver(staged, m);
        deliver(staged_near, m);
    }
    std::size_t beyond_20m = 0;
    for (const MapCone& c : staged.map().cones) {
        if (track.start_pose.to_local(c.position).norm() > 20.0) {
            ++beyond_20m;
        }
    }
    // 20 landmarks, 14 of them beyond 20 m, against 6 with the 20 m limit, which turned away
    // the far cones' observations as too far to map.
    EXPECT_GE(staged.map().cones.size(), 15U);
    EXPECT_GE(beyond_20m, 10U);
    const MapMetrics at_rest = evaluate_map(staged.map(), track, {});
    EXPECT_EQ(at_rest.duplicates, 0U);
    EXPECT_EQ(at_rest.spurious, 0U);
    EXPECT_LT(at_rest.rms_error, 0.05);
    EXPECT_LT(staged_near.map().cones.size(), staged.map().cones.size());
    EXPECT_GT(staged_near.stats().too_far_to_map, 100U);

    // Map 5 cm rms, lap 1 9 cm, racing 7 cm.
    const Evaluation e = evaluate(run, track, config);
    EXPECT_EQ(e.map.duplicates, 0U);
    EXPECT_EQ(e.map.spurious, 0U);
    EXPECT_EQ(e.map.missed, 0U);
    EXPECT_LT(e.map.rms_error, 0.1);
    EXPECT_LT(e.lap1_position_rms, 0.2);
    EXPECT_LT(e.racing_position_rms, 0.15);
}

TEST(StateEstimator, TheYawRateBiasStateKeepsADriftingLapFromDoublingTheMap) {
    // World seed 5 with observations made from the truth: the velocity filter's yaw rate is off
    // by several 1e-4 rad/s for many seconds, and the yaw drifts over lap 1. Without the bias
    // state the covariance does not follow that drift. With new landmarks limited to 20 m, each
    // seen over a shorter stretch, the lap drifts so far that at the timing line the first cones
    // fall outside even the new-landmark gate and are mapped again. At the default 30 m the
    // drift stays within the gates either way, and the bias state halves it (docs/estimation.md).
    const Track track = sim::generate_track(5);
    sim::WorldConfig config;
    config.seed = 5;
    const SyntheticRun run = simulated_drive(track, config, ConeSource::Synthetic);

    EstimatorConfig near;
    near.slam.max_new_landmark_range = 20.0;
    EstimatorConfig near_without = near;
    near_without.slam.yaw_rate_bias_sigma = 0.0;
    const Evaluation near_off = evaluate(run, track, config, near_without);
    const Evaluation near_on = evaluate(run, track, config, near);
    EXPECT_GT(near_off.map.duplicates, 5U);
    EXPECT_EQ(near_on.map.duplicates, 0U);
    EXPECT_EQ(near_on.map.missed, 0U);
    EXPECT_LT(near_on.map.rms_error, 0.08);
    EXPECT_LT(near_on.racing_position_rms, 0.1);

    EstimatorConfig without;
    without.slam.yaw_rate_bias_sigma = 0.0;
    const Evaluation off = evaluate(run, track, config, without);
    const Evaluation on = evaluate(run, track, config);
    EXPECT_EQ(on.map.duplicates, 0U);
    EXPECT_EQ(on.map.missed, 0U);
    EXPECT_LT(on.map.rms_error, 0.08);
    EXPECT_LT(on.racing_position_rms, 0.1);
    EXPECT_LT(on.lap1_position_rms, 0.7 * off.lap1_position_rms);
}

TEST(StateEstimator, TracksTheSimulatedCarWithThePerceptionInTheLoop) {
    // World -> ConePerception -> StateEstimator: the simulator's vehicle model and its IMU, wheel
    // and steering signals, and the LiDAR's clusters with the camera's colors as observations.
    // (bench/estimation also runs the World with observations made from the truth.)
    const Track track = sim::generate_track(2);
    sim::WorldConfig config;
    config.seed = 2;
    const SyntheticRun run = simulated_drive(track, config, ConeSource::Perception);
    ASSERT_TRUE(std::isfinite(run.lap1_end));
    const Evaluation e = evaluate(run, track, config);

    EXPECT_LT(e.velocity_rms.x(), 0.05);   // m/s
    EXPECT_LT(e.velocity_rms.y(), 0.08);   // m/s
    EXPECT_LT(e.velocity_rms.z(), 0.004);  // rad/s
    // On the simulator's sensors the default noise model is somewhat conservative; it must
    // never be overconfident.
    EXPECT_LT(e.velocity_nees.mean, nees_bounds(3, e.velocity_nees.samples).upper);

    // Perception's centers are good to 1-2 cm, far better than the synthetic stand-in's.
    EXPECT_EQ(e.map.duplicates, 0U);
    EXPECT_EQ(e.map.spurious, 0U);
    EXPECT_EQ(e.map.missed, 0U);
    EXPECT_LT(e.map.rms_error, 0.04);
    EXPECT_LT(e.lap1_position_rms, 0.08);
    EXPECT_LT(e.racing_position_rms, 0.08);
    EXPECT_EQ(e.final_landmarks, e.map.landmarks);
}

}  // namespace
}  // namespace apex::estimation::eval
