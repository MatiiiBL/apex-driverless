#include "apex/estimation/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/track.hpp"
#include "apex/estimation/eval/map_metrics.hpp"
#include "apex/estimation/eval/synthetic.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::estimation {
namespace {

using eval::DriveProfile;
using eval::SyntheticRun;
using eval::SyntheticSensors;

struct Outcome {
    StateEstimate final_estimate;
    ConeMap lap1_map;  // when mapping was switched off (or at the end, if it never was)
    ConeMap final_map;
    EstimatorStats stats;
    double lap1_position_rms = 0.0;
    double racing_position_rms = 0.0;
    double racing_yaw_rms = 0.0;
    bool covariances_spd = true;
};

bool spd(const Eigen::MatrixXd& p) {
    return (p - p.transpose()).cwiseAbs().maxCoeff() <= 1e-9 * std::max(1.0, p.norm()) &&
           Eigen::LLT<Eigen::MatrixXd>(p).info() == Eigen::Success;
}

// Runs an estimator over a synthetic drive, switching mapping off one second into lap 2 (as
// the stack does once it has closed the loop), and scores the pose against the truth.
Outcome run_estimator(const SyntheticRun& run, const EstimatorConfig& config = {},
                      bool localize_after_lap1 = true, bool check_covariances = false) {
    StateEstimator estimator(config, VehicleParams{}, SensorRig{}, run.truth.front().pose);
    Outcome out;
    double lap1_sum = 0.0;
    double racing_sum = 0.0;
    double racing_yaw = 0.0;
    std::size_t lap1_n = 0;
    std::size_t racing_n = 0;
    const Time switch_time = run.lap1_end + 1.0;
    bool switched = false;
    for (const eval::TimedMessage& m : run.messages) {
        if (localize_after_lap1 && !switched && m.delivery >= switch_time) {
            out.lap1_map = estimator.map();
            estimator.set_mapping(false);
            switched = true;
        }
        eval::deliver(estimator, m);
        if (std::holds_alternative<ConeObservations>(m.message)) {
            const StateEstimate e = estimator.estimate();
            const eval::TruthSample& t = run.truth_at(e.stamp);
            const double error2 = (e.pose.position() - t.pose.position()).squaredNorm();
            if (switched) {
                racing_sum += error2;
                const double yaw = angle_diff(e.pose.yaw, t.pose.yaw);
                racing_yaw += yaw * yaw;
                ++racing_n;
            } else {
                lap1_sum += error2;
                ++lap1_n;
            }
            if (check_covariances) {
                out.covariances_spd = out.covariances_spd && spd(e.pose_covariance);
                for (const MapCone& cone : estimator.map().cones) {
                    out.covariances_spd = out.covariances_spd && spd(cone.covariance);
                }
            }
        }
    }
    if (!switched) out.lap1_map = estimator.map();
    out.final_estimate = estimator.estimate();
    out.final_map = estimator.map();
    out.stats = estimator.stats();
    out.lap1_position_rms =
        std::sqrt(lap1_sum / static_cast<double>(std::max<std::size_t>(lap1_n, 1)));
    out.racing_position_rms =
        std::sqrt(racing_sum / static_cast<double>(std::max<std::size_t>(racing_n, 1)));
    out.racing_yaw_rms =
        std::sqrt(racing_yaw / static_cast<double>(std::max<std::size_t>(racing_n, 1)));
    return out;
}

std::vector<std::int32_t> mappable(const SyntheticRun& run, Time by) {
    std::vector<std::int32_t> ids;
    for (std::size_t i = 0; i < run.mappable_since.size(); ++i) {
        if (run.mappable_since[i] < by) ids.push_back(static_cast<std::int32_t>(i));
    }
    return ids;
}

SyntheticRun generated_run(std::uint64_t seed, int laps, const SyntheticSensors& sensors = {}) {
    DriveProfile profile;
    profile.laps = laps;
    return eval::make_synthetic_run(sim::generate_track(seed), VehicleParams{}, profile, sensors,
                                    seed);
}

// Landmark position errors against the covariance the map reports, averaged: about 2 (the
// dimension) for a consistent map.
double map_nees(const ConeMap& map, const Track& track) {
    double sum = 0.0;
    for (const MapCone& cone : map.cones) {
        const Vec2 e =
            cone.position - track.cones[static_cast<std::size_t>(cone.truth_id)].position;
        sum += e.dot(cone.covariance.llt().solve(e));
    }
    return sum / static_cast<double>(std::max<std::size_t>(map.cones.size(), 1));
}

// One lap of the oval: short, for the tests that compare runs with each other.
SyntheticRun oval_run(const SyntheticSensors& sensors = {}, std::uint64_t seed = 21) {
    return eval::make_synthetic_run(make_oval_track(), VehicleParams{}, DriveProfile{}, sensors,
                                    seed);
}

TEST(StateEstimator, MapsAGeneratedTrackAndLocalizesOnIt) {
    const Track track = sim::generate_track(3);
    const SyntheticRun run = generated_run(3, 2);
    const Outcome out = run_estimator(run);
    const std::vector<std::int32_t> ids = mappable(run, run.lap1_end - 2.0);
    const eval::MapMetrics map = eval::evaluate_map(out.lap1_map, track, ids);
    EXPECT_EQ(map.duplicates, 0U);
    EXPECT_EQ(map.spurious, 0U);
    EXPECT_LE(map.missed, ids.size() / 50);
    EXPECT_LT(map.rms_error, 0.12);
    // The map knows how good it is: neither overconfident nor uselessly conservative.
    const double nees = map_nees(out.lap1_map, track);
    EXPECT_GT(nees, 0.8);
    EXPECT_LT(nees, 3.0);
    EXPECT_LT(out.lap1_position_rms, 0.10);
    EXPECT_LT(out.racing_position_rms, 0.12);
    EXPECT_LT(out.racing_yaw_rms, deg_to_rad(0.5));
    EXPECT_TRUE(out.final_estimate.valid);
    EXPECT_EQ(out.stats.failed_updates, 0U);
}

TEST(StateEstimator, ClosesTheLoopOnTheOvalWithoutDuplicates) {
    // Re-observing the first cones at the end of lap 1 must pull the map together.
    const Track track = make_oval_track();
    DriveProfile profile;
    profile.laps = 2;
    const SyntheticRun run = eval::make_synthetic_run(track, VehicleParams{}, profile, {}, 21);
    const Outcome out = run_estimator(run, {}, false);
    std::vector<std::int32_t> all(track.cones.size());
    for (std::size_t i = 0; i < all.size(); ++i) all[i] = static_cast<std::int32_t>(i);
    const eval::MapMetrics map = eval::evaluate_map(out.final_map, track, all);
    EXPECT_EQ(map.duplicates, 0U);
    EXPECT_EQ(map.spurious, 0U);
    EXPECT_EQ(map.missed, 0U);
    EXPECT_EQ(map.landmarks, track.cones.size());
    EXPECT_LT(map.rms_error, 0.10);
}

TEST(StateEstimator, MappingOffNeverAddsLandmarks) {
    const SyntheticRun run = generated_run(5, 2);
    const Outcome out = run_estimator(run);
    ASSERT_GT(out.lap1_map.cones.size(), 50U);
    ASSERT_EQ(out.final_map.cones.size(), out.lap1_map.cones.size());
    for (std::size_t i = 0; i < out.final_map.cones.size(); ++i) {
        const MapCone& before = out.lap1_map.cones[i];
        const MapCone& after = out.final_map.cones[i];
        EXPECT_EQ(after.id, before.id);
        // Localization treats the landmarks as fixed (Schmidt update): nothing moves.
        EXPECT_EQ(after.position, before.position);
        EXPECT_EQ(after.observations, before.observations);
    }
    EXPECT_EQ(out.final_map.revision, out.lap1_map.revision);
}

TEST(StateEstimator, LateObservationsMatchOnTimeOnes) {
    SyntheticSensors on_time;
    on_time.latency = 0.0;
    SyntheticSensors late;  // the 50 ms default
    SyntheticSensors later;
    later.latency = 0.1;
    const Outcome a = run_estimator(oval_run(on_time), {}, false);
    const Outcome b = run_estimator(oval_run(late), {}, false);
    const Outcome c = run_estimator(oval_run(later), {}, false);
    ASSERT_GT(a.final_map.cones.size(), 40U);
    for (const Outcome* other : {&b, &c}) {
        ASSERT_EQ(other->final_map.cones.size(), a.final_map.cones.size());
        for (std::size_t i = 0; i < a.final_map.cones.size(); ++i) {
            EXPECT_EQ(other->final_map.cones[i].id, a.final_map.cones[i].id);
            EXPECT_LT((other->final_map.cones[i].position - a.final_map.cones[i].position).norm(),
                      1e-6);
        }
        EXPECT_LT((other->final_estimate.pose.position() - a.final_estimate.pose.position()).norm(),
                  1e-6);
        EXPECT_EQ(other->stats.late_dropped, 0U);
        EXPECT_EQ(other->stats.associated, a.stats.associated);
    }
}

TEST(StateEstimator, DropsObservationsOlderThanItsHistory) {
    SyntheticSensors sensors;
    sensors.latency = 0.7;  // beyond the 0.5 s of odometry kept
    SyntheticRun run = oval_run(sensors);
    // The drive ends with the IMU; scans still in flight then would be within the history.
    Time last_imu = 0.0;
    for (const eval::TimedMessage& m : run.messages) {
        if (std::holds_alternative<ImuSample>(m.message)) last_imu = m.delivery;
    }
    std::erase_if(run.messages, [&](const eval::TimedMessage& m) { return m.delivery > last_imu; });
    const Outcome out = run_estimator(run, {}, false);
    EXPECT_GT(out.stats.late_dropped, 0U);
    EXPECT_EQ(out.stats.late_dropped, out.stats.observations);
    EXPECT_TRUE(out.final_map.cones.empty());
}

TEST(StateEstimator, CovarianceStaysSymmetricPositiveDefinite) {
    const Outcome out = run_estimator(generated_run(1, 2), {}, true, true);
    EXPECT_TRUE(out.covariances_spd);
    EXPECT_EQ(out.stats.failed_updates, 0U);
}

TEST(StateEstimator, IsDeterministic) {
    const SyntheticRun run = oval_run();
    const Outcome a = run_estimator(run, {}, false);
    const Outcome b = run_estimator(run, {}, false);
    EXPECT_EQ(a.final_estimate.pose.x, b.final_estimate.pose.x);
    EXPECT_EQ(a.final_estimate.pose.y, b.final_estimate.pose.y);
    EXPECT_EQ(a.final_estimate.pose_covariance, b.final_estimate.pose_covariance);
    ASSERT_EQ(a.final_map.cones.size(), b.final_map.cones.size());
    for (std::size_t i = 0; i < a.final_map.cones.size(); ++i) {
        EXPECT_EQ(a.final_map.cones[i].position, b.final_map.cones[i].position);
        EXPECT_EQ(a.final_map.cones[i].covariance, b.final_map.cones[i].covariance);
    }
    EXPECT_EQ(a.final_map.revision, b.final_map.revision);
}

TEST(StateEstimator, ShufflingTruthIdsChangesOnlyTheTruthIds) {
    const Track track = sim::generate_track(6);
    SyntheticRun run = generated_run(6, 1);
    const Outcome original = run_estimator(run, {}, false);

    // A random bijection of the cone ids; -1 (clutter) stays -1.
    std::vector<std::int32_t> permutation(track.cones.size());
    for (std::size_t i = 0; i < permutation.size(); ++i) {
        permutation[i] = static_cast<std::int32_t>(i);
    }
    Rng rng(99);
    for (std::size_t i = permutation.size(); i > 1; --i) {
        std::swap(permutation[i - 1], permutation[rng.index(i)]);
    }
    for (eval::TimedMessage& m : run.messages) {
        if (auto* cones = std::get_if<ConeObservations>(&m.message)) {
            for (ConeObservation& obs : cones->cones) {
                if (obs.truth_id >= 0) {
                    obs.truth_id = permutation[static_cast<std::size_t>(obs.truth_id)];
                }
            }
        }
    }
    const Outcome shuffled = run_estimator(run, {}, false);
    EXPECT_EQ(shuffled.final_estimate.pose.x, original.final_estimate.pose.x);
    EXPECT_EQ(shuffled.final_estimate.pose.yaw, original.final_estimate.pose.yaw);
    ASSERT_EQ(shuffled.final_map.cones.size(), original.final_map.cones.size());
    for (std::size_t i = 0; i < original.final_map.cones.size(); ++i) {
        const MapCone& a = original.final_map.cones[i];
        const MapCone& b = shuffled.final_map.cones[i];
        EXPECT_EQ(b.id, a.id);
        EXPECT_EQ(b.position, a.position);
        EXPECT_EQ(b.color_votes, a.color_votes);
        EXPECT_EQ(b.truth_id,
                  a.truth_id < 0 ? -1 : permutation[static_cast<std::size_t>(a.truth_id)]);
    }
}

// Scans of the same three cones from a car at rest.
ConeObservations scan(Time stamp, std::vector<Vec2> cones) {
    ConeObservations obs;
    obs.stamp = stamp;
    std::int32_t id = 0;
    for (const Vec2& c : cones) {
        ConeObservation o;
        o.position = c;
        o.covariance = 0.01 * Mat2::Identity();
        o.color = ConeColor::Blue;
        o.color_confidence = 0.9;
        o.truth_id = id++;
        obs.cones.push_back(o);
    }
    return obs;
}

TEST(StateEstimator, ConfirmsLandmarksAfterRepeatedDetectionsWithStableIds) {
    StateEstimator estimator({}, VehicleParams{}, SensorRig{}, {});
    const std::vector<Vec2> cones = {{5.0, 2.0}, {5.0, -2.0}, {9.0, 2.0}};
    Time t = 0.0;
    for (int i = 0; i < 5; ++i) {
        for (int k = 0; k < 20; ++k) {
            t += 0.005;
            estimator.on_imu({t, 0.0, 0.0, 0.0});
        }
        estimator.on_cones(scan(t, cones));
        const std::vector<std::int32_t>& assoc = estimator.last_associations();
        ASSERT_EQ(assoc.size(), cones.size());
        if (i < 3) {
            // Seen once, twice, three times: tentative, then confirmed by this scan.
            EXPECT_EQ(assoc, (std::vector<std::int32_t>{-1, -1, -1})) << i;
        } else {
            EXPECT_EQ(assoc, (std::vector<std::int32_t>{0, 1, 2})) << i;
        }
        EXPECT_EQ(estimator.map().cones.size(), i < 2 ? 0U : 3U) << i;
    }
    const ConeMap& map = estimator.map();
    for (std::size_t i = 0; i < map.cones.size(); ++i) {
        EXPECT_EQ(map.cones[i].id, static_cast<std::int32_t>(i));
        EXPECT_EQ(map.cones[i].color, ConeColor::Blue);
        EXPECT_EQ(map.cones[i].observations, 5U);
        EXPECT_EQ(map.cones[i].truth_id, static_cast<std::int32_t>(i));
        EXPECT_LT((map.cones[i].position - cones[i]).norm(), 0.05);
    }
    EXPECT_EQ(estimator.stats().new_landmarks, 3U);
    EXPECT_EQ(estimator.stats().associated, 6U);
}

TEST(StateEstimator, DropsUnconfirmedClutter) {
    StateEstimator estimator({}, VehicleParams{}, SensorRig{}, {});
    Time t = 0.0;
    for (int i = 0; i < 10; ++i) {
        for (int k = 0; k < 20; ++k) {
            t += 0.005;
            estimator.on_imu({t, 0.0, 0.0, 0.0});
        }
        // A real cone every scan, and clutter that never appears twice in the same place.
        estimator.on_cones(scan(t, {{6.0, 1.5}, {4.0 + i, -3.0 + 0.7 * i}}));
    }
    EXPECT_EQ(estimator.map().cones.size(), 1U);
    // Each clutter tentative goes 0.35 s after it was seen; the last few are still pending.
    EXPECT_GE(estimator.stats().tentatives_dropped, 5U);
}

TEST(StateEstimator, IgnoresInvalidInputs) {
    StateEstimator estimator({}, VehicleParams{}, SensorRig{}, {1.0, 2.0, 0.5});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    estimator.on_imu({0.005, 0.0, 0.0, 0.0});
    estimator.on_imu({0.005, 0.0, 0.0, 0.0});  // not newer
    estimator.on_imu({0.010, nan, 0.0, 0.0});
    estimator.on_wheels({0.010, nan, 0.0});
    estimator.on_steering({0.010, nan});
    ConeObservations bad = scan(0.005, {{5.0, 1.0}});
    bad.cones[0].position.x() = nan;
    estimator.on_cones(bad);
    ConeObservations unstamped = scan(nan, {{5.0, 1.0}});
    estimator.on_cones(unstamped);
    EXPECT_EQ(estimator.stats().invalid_inputs, 6U);
    const StateEstimate e = estimator.estimate();
    EXPECT_TRUE(e.valid);
    EXPECT_NEAR(e.pose.x, 1.0, 1e-9);
    EXPECT_NEAR(e.pose.yaw, 0.5, 1e-9);
    EXPECT_TRUE(estimator.map().cones.empty());
}

TEST(StateEstimator, StartsAtItsStartPoseBeforeAnyMessage) {
    const StateEstimator estimator({}, VehicleParams{}, SensorRig{}, {3.0, -1.0, 2.0});
    const StateEstimate e = estimator.estimate();
    EXPECT_FALSE(e.valid);  // no time base yet
    EXPECT_EQ(e.pose.x, 3.0);
    EXPECT_EQ(e.pose.yaw, 2.0);
    EXPECT_TRUE(estimator.mapping());
}

TEST(StateEstimator, ConfigRoundTripsThroughJsonAndIsValidated) {
    EstimatorConfig config;
    config.slam.confirm_detections = 4;
    config.velocity.gyro_noise = 0.004;
    config.association.node_budget = 500;
    const auto back = nlohmann::json(config).get<EstimatorConfig>();
    EXPECT_EQ(back.slam.confirm_detections, 4U);
    EXPECT_EQ(back.velocity.gyro_noise, 0.004);
    EXPECT_EQ(back.association.node_budget, 500U);
    EXPECT_THROW((void)nlohmann::json({{"slm", {}}}).get<EstimatorConfig>(), std::invalid_argument);
    EXPECT_THROW((void)nlohmann::json({{"slam", {{"history", 1.0}}}}).get<EstimatorConfig>(),
                 std::invalid_argument);

    EstimatorConfig invalid;
    invalid.slam.history_duration = 0.0;
    EXPECT_THROW(StateEstimator(invalid, VehicleParams{}, SensorRig{}, {}), std::invalid_argument);
    invalid = {};
    invalid.slam.confirm_detections = 0;
    EXPECT_THROW(StateEstimator(invalid, VehicleParams{}, SensorRig{}, {}), std::invalid_argument);
}

}  // namespace
}  // namespace apex::estimation
