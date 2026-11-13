#include "apex/estimation/eval/batch_slam.hpp"

#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/track.hpp"
#include "apex/estimation/chi_square.hpp"
#include "apex/estimation/estimator.hpp"
#include "apex/estimation/eval/map_metrics.hpp"
#include "apex/estimation/eval/synthetic.hpp"

namespace apex::estimation::eval {
namespace {

struct Scenario {
    std::vector<Pose2> poses;
    std::vector<Vec2> landmarks;
    BatchProblem problem;
};

// A car driving a circle among landmarks, observing those within 15 m, with odometry and
// observations perturbed by noise of exactly their stated covariances (none if `noisy` is off).
Scenario circle(bool noisy, std::uint64_t seed = 1) {
    Rng rng(seed);
    Scenario s;
    for (int k = 0; k < 60; ++k) {
        const double a = 0.1 * k;
        s.poses.push_back({20.0 * std::cos(a), 20.0 * std::sin(a), wrap_angle(a + kPi / 2.0)});
    }
    for (int i = 0; i < 25; ++i) {
        const double a = rng.uniform(0.0, 6.0);
        const double r = rng.uniform(16.0, 24.0);
        s.landmarks.push_back({r * std::cos(a), r * std::sin(a)});
    }
    const Vec3 odometry_std(0.02, 0.01, 0.003);
    const double observation_std = 0.05;
    s.problem.prior = s.poses.front();
    s.problem.prior_covariance = 1e-6 * Mat3::Identity();
    for (std::size_t k = 0; k + 1 < s.poses.size(); ++k) {
        const Pose2 d = s.poses[k].between(s.poses[k + 1]);
        BatchOdometry o;
        o.delta = d;
        if (noisy) {
            o.delta = {d.x + odometry_std.x() * rng.normal(), d.y + odometry_std.y() * rng.normal(),
                       d.yaw + odometry_std.z() * rng.normal()};
        }
        o.covariance = odometry_std.cwiseAbs2().asDiagonal();
        s.problem.odometry.push_back(o);
    }
    for (std::size_t k = 0; k < s.poses.size(); ++k) {
        for (std::size_t i = 0; i < s.landmarks.size(); ++i) {
            const Vec2 z = s.poses[k].to_local(s.landmarks[i]);
            if (z.norm() > 15.0) continue;
            BatchObservation o;
            o.pose = k;
            o.landmark = static_cast<std::int32_t>(100 + i);  // keys need not be indices
            o.z = z;
            if (noisy) o.z += observation_std * Vec2{rng.normal(), rng.normal()};
            o.R = observation_std * observation_std * Mat2::Identity();
            s.problem.observations.push_back(o);
        }
    }
    return s;
}

TEST(BatchSlam, RecoversTheTruthFromNoiseFreeData) {
    const Scenario s = circle(false);
    const BatchSolution solution = solve_batch_slam(s.problem);
    EXPECT_TRUE(solution.converged);
    ASSERT_EQ(solution.poses.size(), s.poses.size());
    for (std::size_t k = 0; k < s.poses.size(); ++k) {
        EXPECT_NEAR(solution.poses[k].x, s.poses[k].x, 1e-9);
        EXPECT_NEAR(angle_diff(solution.poses[k].yaw, s.poses[k].yaw), 0.0, 1e-9);
    }
    for (const auto& [key, position] : solution.landmarks) {
        EXPECT_LT((position - s.landmarks[static_cast<std::size_t>(key - 100)]).norm(), 1e-9);
    }
    EXPECT_LT(solution.final_cost, 1e-12);
}

TEST(BatchSlam, ConvergesToAMinimumWithTheChiSquareCost) {
    // At the optimum of a correctly weighted problem the cost is chi^2 with
    // (residuals - unknowns) degrees of freedom; an over-weighted or wrongly solved one is not.
    const Scenario s = circle(true, 2);
    const BatchSolution solution = solve_batch_slam(s.problem);
    EXPECT_TRUE(solution.converged);
    EXPECT_LT(solution.iterations, 10);
    EXPECT_LT(solution.final_cost, solution.initial_cost);
    const int residuals = 3 + 3 * static_cast<int>(s.problem.odometry.size()) +
                          2 * static_cast<int>(s.problem.observations.size());
    const int unknowns = 3 * static_cast<int>(solution.poses.size()) +
                         2 * static_cast<int>(solution.landmarks.size());
    const int dof = residuals - unknowns;
    EXPECT_GT(solution.final_cost, chi_square_quantile(0.005, dof));
    EXPECT_LT(solution.final_cost, chi_square_quantile(0.995, dof));

    // And the map is better than the initial guess from dead reckoning.
    double before = 0.0;
    double after = 0.0;
    std::map<std::int32_t, Vec2> first;
    std::vector<Pose2> guess = {s.problem.prior};
    for (const BatchOdometry& o : s.problem.odometry)
        guess.push_back(guess.back().compose(o.delta));
    for (const BatchObservation& o : s.problem.observations) {
        first.emplace(o.landmark, guess[o.pose].to_parent(o.z));
    }
    for (const auto& [key, position] : solution.landmarks) {
        const Vec2& truth = s.landmarks[static_cast<std::size_t>(key - 100)];
        before += (first.at(key) - truth).squaredNorm();
        after += (position - truth).squaredNorm();
    }
    EXPECT_LT(after, 0.25 * before);
}

TEST(BatchSlam, RejectsMalformedProblems) {
    Scenario s = circle(false);
    s.problem.observations.back().pose = s.poses.size();
    EXPECT_THROW((void)solve_batch_slam(s.problem), std::invalid_argument);
    s = circle(false);
    s.problem.odometry[3].covariance = Mat3::Zero();
    EXPECT_THROW((void)solve_batch_slam(s.problem), std::invalid_argument);
}

TEST(BatchSlam, IsTheReferenceTheFilterMapIsCheckedAgainst) {
    // The same messages through the EKF and through the batch recorder: the batch solution,
    // with every observation re-linearized and the true associations, must be at least as good,
    // and the filter's map must sit within its own uncertainty of it.
    const Track track = make_oval_track();
    DriveProfile profile;
    profile.laps = 2;
    const SyntheticRun run = make_synthetic_run(track, VehicleParams{}, profile, {}, 8);
    StateEstimator estimator({}, VehicleParams{}, SensorRig{}, run.truth.front().pose);
    BatchRecorder recorder({}, VehicleParams{}, run.truth.front().pose);
    for (const TimedMessage& m : run.messages) {
        deliver(estimator, m);
        deliver(recorder, m);
    }
    ASSERT_EQ(recorder.stamps().size(), recorder.problem().odometry.size() + 1);
    const BatchSolution batch = solve_batch_slam(recorder.problem());
    EXPECT_TRUE(batch.converged);

    ConeMap batch_map;
    for (const auto& [key, position] : batch.landmarks) {
        MapCone cone;
        cone.id = key;
        cone.truth_id = key;
        cone.position = position;
        batch_map.cones.push_back(cone);
    }
    std::vector<std::int32_t> all(track.cones.size());
    for (std::size_t i = 0; i < all.size(); ++i) all[i] = static_cast<std::int32_t>(i);
    const MapMetrics filter = evaluate_map(estimator.map(), track, all);
    const MapMetrics reference = evaluate_map(batch_map, track, all);
    EXPECT_EQ(reference.missed, 0U);
    EXPECT_LE(reference.rms_error, filter.rms_error + 0.005);

    std::map<std::int32_t, Vec2> by_truth;
    for (const auto& [key, position] : batch.landmarks) by_truth[key] = position;
    double nees = 0.0;
    for (const MapCone& cone : estimator.map().cones) {
        const Vec2 d = cone.position - by_truth.at(cone.truth_id);
        nees += d.dot(cone.covariance.llt().solve(d));
    }
    nees /= static_cast<double>(estimator.map().cones.size());
    EXPECT_LT(nees, 2.0);  // the filter-to-reference gap is within the filter's 2-D uncertainty
}

}  // namespace
}  // namespace apex::estimation::eval
