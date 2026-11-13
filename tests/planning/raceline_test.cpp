#include "apex/planning/raceline.hpp"

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/track.hpp"
#include "apex/planning/planning.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::planning {
namespace {

std::vector<Vec2> circle(double radius, std::size_t n) {
    std::vector<Vec2> points;
    for (std::size_t i = 0; i < n; ++i) {
        const double angle = kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        points.push_back(radius * heading_vector(angle));
    }
    return points;
}

// A track model made from a true track: its centerline and cone lines.
TrackModel model_of(const Track& track) {
    return {track.centerline, track.left_boundary(), track.right_boundary()};
}

double centerline_lap_time(const TrackModel& track, const VehicleParams& car,
                           const RacelineConfig& config) {
    Trajectory center = track.centerline;
    apply_speed_profile(center, car, config.speed);
    return profile_time(center);
}

TEST(MinCurvatureQp, CurvatureOfACircleAndItsLinearization) {
    constexpr double kRadius = 20.0;
    const std::vector<Vec2> ring = circle(kRadius, 60);
    const std::vector<double> lower(ring.size(), -1.0);
    const std::vector<double> upper(ring.size(), 1.0);
    const MinCurvatureQp qp = min_curvature_qp(ring, lower, upper, 0.0, 0.0);
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const auto k = static_cast<Eigen::Index>(i);
        // The chord-parametrized spline overestimates by ~0.1% at 2 m knots on a 20 m circle.
        EXPECT_NEAR(qp.curvature[k], 1.0 / kRadius, 1e-4);
        // Counter-clockwise, so the left normals point at the center.
        EXPECT_GT(qp.normals[i].dot(-ring[i].normalized()), 0.9999);
    }
    // Moving every point 0.2 m inward gives a circle of radius 19.8: kappa_0 + M alpha matches
    // the spline's curvature of that circle up to second order in the offset.
    const Eigen::VectorXd inward = Eigen::VectorXd::Constant(60, 0.2);
    const Eigen::VectorXd predicted = qp.curvature + qp.jacobian * inward;
    const MinCurvatureQp moved = min_curvature_qp(circle(kRadius - 0.2, 60), lower, upper, 0, 0);
    for (Eigen::Index k = 0; k < predicted.size(); ++k) {
        EXPECT_NEAR(predicted[k], moved.curvature[k], 1e-5);
        EXPECT_GT(predicted[k], qp.curvature[k]);
    }
    EXPECT_EQ(qp.qp.G.rows(), 0);
    EXPECT_EQ(min_curvature_qp(ring, lower, upper, 0.2, 0.0).qp.G.rows(), 120);
}

TEST(MinCurvatureQp, RejectsBadInput) {
    const std::vector<Vec2> ring = circle(10.0, 8);
    const std::vector<double> bounds(8, 1.0);
    const std::vector<double> short_bounds(7, 1.0);
    EXPECT_THROW((void)min_curvature_qp(ring, short_bounds, bounds, 0.0, 0.0),
                 std::invalid_argument);
    EXPECT_THROW(
        (void)min_curvature_qp(std::span(ring).first(3), short_bounds, short_bounds, 0.0, 0.0),
        std::invalid_argument);
    EXPECT_THROW((void)min_curvature_qp(ring, bounds, bounds, -1.0, 0.0), std::invalid_argument);
}

// On a ring of constant width the least curved closed path is the widest circle the bounds
// allow: the outer cone line less the car's half width and the margin.
TEST(Raceline, RingTrackGivesTheWidestCircle) {
    constexpr double kRadius = 15.0;
    const TrackModel ring{Trajectory::from_waypoints(circle(kRadius, 120), true, 0.5),
                          circle(kRadius - 1.75, 400), circle(kRadius + 1.75, 400)};
    const VehicleParams car;
    const RacelineConfig config;
    const Raceline raceline = optimize_raceline(ring, car, config);
    const double expected = kRadius + 1.75 - 0.5 * car.body_width - config.safety_margin;
    for (const TrajectoryPoint& p : raceline.path.points()) {
        EXPECT_NEAR(p.position.norm(), expected, 0.02);
        EXPECT_NEAR(p.curvature, 1.0 / expected, 2e-4);
    }
    // Least curvature is not least time: on a ring the longer lap outweighs the higher speed.
    EXPECT_GT(raceline.predicted_lap_time, centerline_lap_time(ring, car, config));
}

TEST(Raceline, BeatsTheCenterlineOfTheOvalWithinTheTrack) {
    const TrackModel oval = model_of(make_oval_track());
    const VehicleParams car;
    const RacelineConfig config;
    RacelineTrace trace;
    const Raceline raceline = optimize_raceline(oval, car, config, &trace);

    ASSERT_EQ(trace.problems.size(), config.iterations);
    for (const QpResult& result : trace.results) EXPECT_EQ(result.status, QpStatus::Solved);
    EXPECT_TRUE(raceline.path.closed());
    const double keep_out = 0.5 * car.body_width + config.safety_margin;
    for (const TrajectoryPoint& p : raceline.path.points()) {
        EXPECT_GT(p.width_left, keep_out - 0.03);
        EXPECT_GT(p.width_right, keep_out - 0.03);
        EXPECT_GT(p.speed, 0.0);
        EXPECT_LT(std::abs(p.curvature), config.max_curvature);
    }
    // Using the 1.5 m of free width in the two 12 m half circles is worth several percent.
    const double centerline = centerline_lap_time(oval, car, config);
    EXPECT_LT(raceline.predicted_lap_time, 0.96 * centerline);
    EXPECT_GT(raceline.optimize_seconds, 0.0);
    EXPECT_LT(raceline.optimize_seconds, 2.0);
}

TEST(Raceline, ImprovesOnEveryReferenceTrack) {
    const VehicleParams car;
    const RacelineConfig config;
    for (const std::string& name : sim::reference_track_names()) {
        const TrackModel model = model_of(sim::reference_track(name));
        const Raceline raceline = optimize_raceline(model, car, config);
        EXPECT_LT(raceline.predicted_lap_time, 0.98 * centerline_lap_time(model, car, config))
            << name;
        EXPECT_LT(raceline.optimize_seconds, 2.0) << name;
    }
}

// A QP that cannot converge in its iteration budget leaves the line it was linearized around:
// here the centerline, still with a valid speed profile.
TEST(Raceline, FallsBackToTheReferenceWhenAQpFails) {
    const TrackModel oval = model_of(make_oval_track());
    RacelineConfig config;
    config.qp_max_iterations = 1;
    RacelineTrace trace;
    const Raceline raceline = optimize_raceline(oval, VehicleParams{}, config, &trace);
    ASSERT_EQ(trace.results.size(), 1U);
    EXPECT_NE(trace.results[0].status, QpStatus::Solved);
    for (const TrajectoryPoint& p : raceline.path.points()) {
        EXPECT_LT(oval.centerline.project(p.position).distance, 0.05);
    }
    EXPECT_TRUE(std::isfinite(raceline.predicted_lap_time));
}

TEST(Raceline, RejectsInvalidInput) {
    const TrackModel oval = model_of(make_oval_track());
    RacelineConfig config;
    config.step = 0.0;
    EXPECT_THROW((void)optimize_raceline(oval, VehicleParams{}, config), std::invalid_argument);
    EXPECT_THROW((void)optimize_raceline(TrackModel{}, VehicleParams{}, RacelineConfig{}),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apex::planning
