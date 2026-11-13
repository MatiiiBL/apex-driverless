#include "apex/estimation/eval/batch_slam.hpp"

#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>

#include "apex/core/angles.hpp"
#include "apex/estimation/ekf_slam.hpp"
#include "apex/estimation/observation.hpp"

namespace apex::estimation::eval {

namespace {

using Triplet = Eigen::Triplet<double>;

// L^-1 for sigma = L L^T: premultiplying a factor's residual and Jacobian by it gives every row
// unit variance, so the cost is a plain sum of squares.
template <int N>
Eigen::Matrix<double, N, N> whitener(const Eigen::Matrix<double, N, N>& sigma) {
    const Eigen::LLT<Eigen::Matrix<double, N, N>> llt(sigma);
    if (llt.info() != Eigen::Success) {
        throw std::invalid_argument("batch slam: a covariance is not positive definite");
    }
    return llt.matrixL().solve(Eigen::Matrix<double, N, N>::Identity());
}

Vec3 pose_difference(const Pose2& a, const Pose2& b) {
    return {a.x - b.x, a.y - b.y, angle_diff(a.yaw, b.yaw)};
}

// The whitened residuals and their Jacobian, one block of rows per factor.
class Linearizer {
public:
    explicit Linearizer(Eigen::Index rows) : residual_(Eigen::VectorXd::Zero(rows)) {}

    template <int N>
    void residual(const Eigen::Matrix<double, N, 1>& r) {
        residual_.segment<N>(row_) = r;
    }
    void block(const Eigen::MatrixXd& j, Eigen::Index col) {
        for (Eigen::Index a = 0; a < j.rows(); ++a) {
            for (Eigen::Index b = 0; b < j.cols(); ++b) {
                if (j(a, b) != 0.0) triplets_.emplace_back(row_ + a, col + b, j(a, b));
            }
        }
    }
    void next(Eigen::Index rows) { row_ += rows; }

    [[nodiscard]] const Eigen::VectorXd& residual() const { return residual_; }
    [[nodiscard]] Eigen::SparseMatrix<double> jacobian(Eigen::Index cols) const {
        Eigen::SparseMatrix<double> j(residual_.size(), cols);
        j.setFromTriplets(triplets_.begin(), triplets_.end());
        return j;
    }

private:
    Eigen::VectorXd residual_;
    std::vector<Triplet> triplets_;
    Eigen::Index row_ = 0;
};

class BatchSolver {
public:
    BatchSolver(const BatchProblem& problem, BatchSolution& state) : problem_(problem), s_(state) {
        prior_whitener_ = whitener<3>(problem.prior_covariance);
        for (const BatchOdometry& o : problem.odometry) {
            odometry_whiteners_.push_back(whitener<3>(o.covariance));
        }
        for (const BatchObservation& o : problem.observations) {
            observation_whiteners_.push_back(whitener<2>(o.R));
        }
        for (std::size_t i = 0; i < s_.landmarks.size(); ++i) index_[s_.landmarks[i].first] = i;
        rows_ = 3 + 3 * static_cast<Eigen::Index>(problem.odometry.size()) +
                2 * static_cast<Eigen::Index>(problem.observations.size());
        cols_ = 3 * static_cast<Eigen::Index>(s_.poses.size()) +
                2 * static_cast<Eigen::Index>(s_.landmarks.size());
    }

    [[nodiscard]] Linearizer linearize() const {
        Linearizer lin(rows_);
        lin.residual<3>(prior_whitener_ * pose_difference(s_.poses[0], problem_.prior));
        lin.block(prior_whitener_, 0);
        lin.next(3);
        for (std::size_t k = 0; k < problem_.odometry.size(); ++k) {
            // r = between(p_k, p_k+1) - delta, and its Jacobians with respect to both poses.
            const Pose2& a = s_.poses[k];
            const Pose2& b = s_.poses[k + 1];
            const double c = std::cos(a.yaw);
            const double s = std::sin(a.yaw);
            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            Mat3 ja;
            ja << -c, -s, -s * dx + c * dy, s, -c, -c * dx - s * dy, 0.0, 0.0, -1.0;
            Mat3 jb;
            jb << c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0;
            const Mat3& w = odometry_whiteners_[k];
            lin.residual<3>(w * pose_difference(a.between(b), problem_.odometry[k].delta));
            lin.block(w * ja, pose_column(k));
            lin.block(w * jb, pose_column(k + 1));
            lin.next(3);
        }
        for (std::size_t i = 0; i < problem_.observations.size(); ++i) {
            const BatchObservation& o = problem_.observations[i];
            const std::size_t l = index_.at(o.landmark);
            const LandmarkMeasurement m =
                measure_landmark(s_.poses[o.pose], s_.landmarks[l].second);
            const Mat2& w = observation_whiteners_[i];
            lin.residual<2>(w * (m.predicted - o.z));
            lin.block(w * m.pose_jacobian, pose_column(o.pose));
            lin.block(w * m.landmark_jacobian, landmark_column(l));
            lin.next(2);
        }
        return lin;
    }

    // One Gauss-Newton step: solves J^T J dx = -J^T r with a sparse LDL^T factorization (the
    // pattern is analyzed once, with an approximate minimum degree ordering). Returns the
    // largest component of the step, or a negative value if the factorization failed.
    double step() {
        const Linearizer lin = linearize();
        const Eigen::SparseMatrix<double> j = lin.jacobian(cols_);
        const Eigen::SparseMatrix<double> h = (j.transpose() * j).pruned();
        const Eigen::VectorXd g = j.transpose() * lin.residual();
        if (!analyzed_) {
            solver_.analyzePattern(h);
            analyzed_ = true;
        }
        solver_.factorize(h);
        if (solver_.info() != Eigen::Success) return -1.0;
        const Eigen::VectorXd dx = solver_.solve(-g);
        if (solver_.info() != Eigen::Success || !dx.allFinite()) return -1.0;
        for (std::size_t k = 0; k < s_.poses.size(); ++k) {
            Pose2& p = s_.poses[k];
            const Eigen::Index c = pose_column(k);
            p = {p.x + dx[c], p.y + dx[c + 1], wrap_angle(p.yaw + dx[c + 2])};
        }
        for (std::size_t l = 0; l < s_.landmarks.size(); ++l) {
            s_.landmarks[l].second += dx.segment<2>(landmark_column(l));
        }
        return dx.cwiseAbs().maxCoeff();
    }

    [[nodiscard]] double cost() const { return linearize().residual().squaredNorm(); }

private:
    [[nodiscard]] static Eigen::Index pose_column(std::size_t k) {
        return 3 * static_cast<Eigen::Index>(k);
    }
    [[nodiscard]] Eigen::Index landmark_column(std::size_t l) const {
        return 3 * static_cast<Eigen::Index>(s_.poses.size()) + 2 * static_cast<Eigen::Index>(l);
    }

    const BatchProblem& problem_;
    BatchSolution& s_;
    Mat3 prior_whitener_;
    std::vector<Mat3> odometry_whiteners_;
    std::vector<Mat2> observation_whiteners_;
    std::map<std::int32_t, std::size_t> index_;  // landmark key -> index in s_.landmarks
    Eigen::Index rows_ = 0;
    Eigen::Index cols_ = 0;
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver_;
    bool analyzed_ = false;
};

}  // namespace

BatchSolution solve_batch_slam(const BatchProblem& problem, const BatchOptions& options) {
    BatchSolution s;
    s.poses.push_back(problem.prior);
    for (const BatchOdometry& o : problem.odometry)
        s.poses.push_back(s.poses.back().compose(o.delta));
    std::map<std::int32_t, Vec2> first_seen;
    for (const BatchObservation& o : problem.observations) {
        if (o.pose >= s.poses.size()) {
            throw std::invalid_argument("batch slam: an observation refers to a missing pose");
        }
        first_seen.emplace(o.landmark, s.poses[o.pose].to_parent(o.z));
    }
    s.landmarks.assign(first_seen.begin(), first_seen.end());

    BatchSolver solver(problem, s);
    s.initial_cost = solver.cost();
    for (s.iterations = 0; s.iterations < options.max_iterations;) {
        const double change = solver.step();
        ++s.iterations;
        if (change < 0.0) break;
        if (change < options.step_tolerance) {
            s.converged = true;
            break;
        }
    }
    s.final_cost = solver.cost();
    return s;
}

BatchRecorder::BatchRecorder(const EstimatorConfig& config, const VehicleParams& nominal,
                             const Pose2& start)
    : config_(config), motion_(config.velocity, config.slam, nominal) {
    problem_.prior = start;
    const double p = config.slam.initial_position_std;
    const double y = config.slam.initial_yaw_std;
    problem_.prior_covariance = Vec3(p * p, p * p, y * y).asDiagonal();
}

void BatchRecorder::on_imu(const ImuSample& sample) {
    motion_.on_imu(sample);
}

void BatchRecorder::on_wheels(const WheelSpeeds& sample) {
    motion_.on_wheels(sample);
}

void BatchRecorder::on_steering(const SteeringSample& sample) {
    motion_.on_steering(sample);
}

void BatchRecorder::on_cones(const ConeObservations& observations) {
    if (!std::isfinite(observations.stamp)) return;
    motion_.start(observations.stamp);
    OdometryBuffer& odometry = motion_.odometry();
    if (stamps_.empty()) stamps_.push_back(odometry.start());  // pose 0: the start pose
    // The estimator drops scans older than its odometry history, or than the previous scan.
    const Time stamp = observations.stamp;
    if (stamp < odometry.start() || stamp < odometry.end() - config_.slam.history_duration) return;
    motion_.extrapolate_to(stamp);
    const RelativeMotion delta = odometry.take_until(stamp);
    if (stamp > stamps_.back()) {
        problem_.odometry.push_back({delta.delta, delta.covariance});
        stamps_.push_back(stamp);
    }
    const std::size_t pose = stamps_.size() - 1;
    for (const ConeObservation& cone : observations.cones) {
        if (cone.truth_id < 0) continue;
        const PreparedObservation prepared = prepare_observation(cone, config_.slam);
        if (prepared.status != ObservationStatus::Usable) continue;
        problem_.observations.push_back({pose, cone.truth_id, prepared.model.z, prepared.model.R});
    }
}

}  // namespace apex::estimation::eval
