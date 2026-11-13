#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "apex/core/messages.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/estimation/dead_reckoning.hpp"
#include "apex/estimation/estimator.hpp"

// The reference for the online map: batch least-squares SLAM over every pose at which cones were
// observed and every landmark, given the associations (from the truth ids), solved by
// Gauss-Newton with a sparse Cholesky factorization of the normal equations (Dellaert and
// Kaess, "Square Root SAM", IJRR 25(12), 2006). With the same odometry and observations as the
// filter, it is the best map those measurements allow, without the filter's linearization and
// association errors.
namespace apex::estimation::eval {

struct BatchOdometry {
    Pose2 delta;  // pose k+1 in the frame of pose k
    Mat3 covariance = Mat3::Identity();
};

struct BatchObservation {
    std::size_t pose = 0;
    std::int32_t landmark = 0;  // any key; equal keys are the same landmark
    Vec2 z = Vec2::Zero();      // in the body frame of the pose
    Mat2 R = Mat2::Identity();
};

struct BatchProblem {
    Pose2 prior;  // of pose 0
    Mat3 prior_covariance = Mat3::Identity();
    std::vector<BatchOdometry> odometry;  // odometry[k] links pose k to pose k + 1
    std::vector<BatchObservation> observations;
};

struct BatchOptions {
    int max_iterations = 30;
    double step_tolerance = 1e-9;  // stop when the largest state change is below this
};

struct BatchSolution {
    std::vector<Pose2> poses;
    std::vector<std::pair<std::int32_t, Vec2>> landmarks;  // sorted by key
    int iterations = 0;
    double initial_cost = 0.0;  // sum of squared whitened residuals, at the initial guess
    double final_cost = 0.0;
    bool converged = false;
};

// Initial guess: poses chained from the prior along the odometry, each landmark at its first
// observation. Throws std::invalid_argument for an observation of a pose that does not exist.
[[nodiscard]] BatchSolution solve_batch_slam(const BatchProblem& problem,
                                             const BatchOptions& options = {});

// Records a run as a BatchProblem from the same messages, in the same order, as an estimator
// gets them: the same dead reckoning between scan stamps and the same observation covariances.
// Observations are associated by truth id; false positives (truth id -1) are left out.
class BatchRecorder {
public:
    BatchRecorder(const EstimatorConfig& config, const VehicleParams& nominal, const Pose2& start);

    void on_imu(const ImuSample& sample);
    void on_wheels(const WheelSpeeds& sample);
    void on_steering(const SteeringSample& sample);
    void on_cones(const ConeObservations& observations);

    [[nodiscard]] const BatchProblem& problem() const { return problem_; }
    // The stamp of each pose of the problem.
    [[nodiscard]] const std::vector<Time>& stamps() const { return stamps_; }

private:
    EstimatorConfig config_;
    DeadReckoning motion_;
    BatchProblem problem_;
    std::vector<Time> stamps_;
};

}  // namespace apex::estimation::eval
