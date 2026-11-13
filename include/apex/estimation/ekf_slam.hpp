#pragma once

#include <cstddef>
#include <span>

#include <Eigen/Core>

#include "apex/core/types.hpp"
#include "apex/estimation/odometry.hpp"

namespace apex::estimation {

// The position of a landmark seen from a pose, z = R(yaw)^T (l - p), and its Jacobians with
// respect to the pose (x, y, yaw) and to the landmark.
struct LandmarkMeasurement {
    Vec2 predicted = Vec2::Zero();
    Eigen::Matrix<double, 2, 3> pose_jacobian = Eigen::Matrix<double, 2, 3>::Zero();
    Mat2 landmark_jacobian = Mat2::Zero();
};
[[nodiscard]] LandmarkMeasurement measure_landmark(const Pose2& pose, const Vec2& landmark);

// One observation assigned to one landmark, for an update.
struct Pairing {
    std::size_t landmark = 0;
    Vec2 z = Vec2::Zero();      // body frame (m)
    Mat2 R = Mat2::Identity();  // its covariance (m^2)
};

// EKF-SLAM over x = (x, y, yaw, l1x, l1y, ..., lnx, lny) with a dense covariance (Smith, Self and
// Cheeseman, "Estimating uncertain spatial relationships in robotics", in Autonomous Robot
// Vehicles, Springer, 1990; Dissanayake et al., "A solution to the simultaneous localization and
// map building (SLAM) problem", IEEE Trans. Robotics and Automation 17(3), 2001).
//
// Costs: predict O(n), update O(n^2 m) for m measurement rows, add_landmark O(n), where n is the
// state size. Storage grows by doubling so adding a landmark does not copy the matrix each time.
class EkfSlam {
public:
    using Covariance = Eigen::Block<const Eigen::MatrixXd>;

    EkfSlam(const Pose2& start, const Mat3& start_covariance);

    [[nodiscard]] Pose2 pose() const { return {x_[0], x_[1], x_[2]}; }
    [[nodiscard]] Mat3 pose_covariance() const { return P_.topLeftCorner<3, 3>(); }
    [[nodiscard]] std::size_t landmark_count() const {
        return static_cast<std::size_t>((n_ - 3) / 2);
    }
    [[nodiscard]] Vec2 landmark(std::size_t i) const { return x_.segment<2>(index(i)); }
    [[nodiscard]] Mat2 landmark_covariance(std::size_t i) const {
        return P_.block<2, 2>(index(i), index(i));
    }
    // Cross-covariances of the pose with landmark i, and of landmarks i and j.
    [[nodiscard]] Eigen::Matrix<double, 3, 2> pose_landmark_covariance(std::size_t i) const {
        return P_.block<3, 2>(0, index(i));
    }
    [[nodiscard]] Mat2 landmark_cross_covariance(std::size_t i, std::size_t j) const {
        return P_.block<2, 2>(index(i), index(j));
    }
    [[nodiscard]] std::size_t state_size() const { return static_cast<std::size_t>(n_); }
    [[nodiscard]] Covariance covariance() const { return P_.topLeftCorner(n_, n_); }

    // Moves the pose by `motion`, given in the body frame of the current pose:
    //   pose' = pose (+) delta,  P_pp' = Jx P_pp Jx^T + Jd Sigma Jd^T,  P_pl' = Jx P_pl.
    void predict(const RelativeMotion& motion);

    // One batch update with every pairing (distinct landmarks), Joseph form:
    //   P' = (I - K H) P (I - K H)^T + K R K^T = M - (M H^T - K R) K^T,  M = P - K W^T,
    // with W = P H^T. M H^T needs M only in the paired columns, so the n x n work is a single
    // product, P - [K T] [W K]^T with T = M H^T - K R, of which only the lower triangle is
    // formed and then mirrored: exactly symmetric, and positive definite where P - K S K^T can
    // lose it to round-off. With update_landmarks false it is a Schmidt-Kalman update: the
    // landmarks are
    // "consider" states whose estimates and covariance stay fixed while the pose and its
    // cross-covariance are updated consistently (Schmidt, "Application of state-space methods to
    // navigation problems", Advances in Control Systems 3, 1966), at O(n m) cost.
    // Returns false, changing nothing, if the innovation covariance is not positive definite.
    bool update(std::span<const Pairing> pairings, bool update_landmarks);

    // Appends the landmark p + R(yaw) z with the covariance and cross-covariances it inherits
    // from the pose and from R; returns its index.
    std::size_t add_landmark(const Vec2& z, const Mat2& R);

    // Marginalizes landmark i out of the state by dropping its rows and columns. The last
    // landmark moves into its slot (O(n) instead of shifting the whole matrix), so indices
    // above i are not stable: callers track their landmarks by their own ids.
    void remove_landmark(std::size_t i);

    [[nodiscard]] bool finite() const;

private:
    [[nodiscard]] static Eigen::Index index(std::size_t landmark) {
        return 3 + 2 * static_cast<Eigen::Index>(landmark);
    }
    void reserve(Eigen::Index size);
    void mirror_lower();  // copies the lower triangle of the active covariance onto the upper

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;  // capacity x capacity; the active part is the top-left n_ x n_
    Eigen::Index n_ = 3;
};

}  // namespace apex::estimation
