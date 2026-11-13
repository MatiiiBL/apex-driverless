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
SyntheticRun simulated_drive(const Track& track, const sim::WorldConfig& config,
                             ConeSource source) {
    DriveProfile profile;
    profile.laps = 2;
    profile.racing_lateral_accel = 10.0;
    profile.max_brake = 8.0;
    profile.max_accel = 6.0;
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
Evaluation evaluate(const SyntheticRun& run, const Track& track, const sim::WorldConfig& config) {
    VelocityFilter filter({}, config.vehicle);
    StateEstimator estimator({}, config.vehicle, config.rig, track.start_pose);
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
