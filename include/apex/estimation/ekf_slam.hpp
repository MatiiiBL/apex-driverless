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

// EKF-SLAM over x = (x, y, yaw, beta, l1x, l1y, ..., lnx, lny) with a dense covariance (Smith,
// Self and Cheeseman, "Estimating uncertain spatial relationships in robotics", in Autonomous
// Robot Vehicles, Springer, 1990; Dissanayake et al., "A solution to the simultaneous
// localization and map building (SLAM) problem", IEEE Trans. Robotics and Automation 17(3),
// 2001).
//
// beta is the slow part of the error of the yaw rate the odometry integrates (rad/s): what the
// velocity filter's gyro bias estimate and its lateral model leave, which lasts for many seconds.
// White odometry noise would make the yaw error it causes grow like sqrt(T); it grows like T. In
// the state, the prediction yaw' = yaw + dyaw - (integral of beta) carries that growth into the
// covariance, and the landmarks observe beta through the drift they see. beta is a first-order
// Gauss-Markov process (a shaping filter for time-correlated noise, Maybeck, "Stochastic Models,
// Estimation, and Control", vol. 1, Academic Press, 1979, section 4.11):
//   beta' = phi beta + w,  phi = exp(-T / tau),  var(w) = sigma^2 (1 - phi^2),
// stationary at sigma, so a value learned in one corner fades over tau instead of being
// extrapolated for a lap. With sigma zero it stays exactly zero, which is the filter without it.
//
// Costs: predict O(n), update O(n^2 m) for m measurement rows, add_landmark O(n), where n is the
// state size. Storage grows by doubling so adding a landmark does not copy the matrix each time.
// The Gauss-Markov model of beta. sigma 0 switches it off.
struct YawRateBiasModel {
    double sigma = 0.0;              // rad/s, stationary standard deviation
    double correlation_time = 20.0;  // s
};

class EkfSlam {
public:
    using Covariance = Eigen::Block<const Eigen::MatrixXd>;
    using Mat4 = Eigen::Matrix4d;
    static constexpr Eigen::Index kVehicle = 4;  // x, y, yaw, beta

    // beta starts at zero with its stationary variance, uncorrelated with the pose. Throws
    // std::invalid_argument for a negative sigma or a non-positive correlation time.
    EkfSlam(const Pose2& start, const Mat3& start_covariance, const YawRateBiasModel& bias = {});

    [[nodiscard]] Pose2 pose() const { return {x_[0], x_[1], x_[2]}; }
    [[nodiscard]] Mat3 pose_covariance() const { return P_.topLeftCorner<3, 3>(); }
    [[nodiscard]] double yaw_rate_bias() const { return x_[3]; }
    // The yaw the bias adds up to over t from now, tau (1 - exp(-t / tau)) beta, and its
    // derivative with respect to beta: for the pose at a time ahead of the filter's.
    [[nodiscard]] double bias_yaw_gain(double t) const;
    // Covariance of (x, y, yaw, beta).
    [[nodiscard]] Mat4 vehicle_covariance() const { return P_.topLeftCorner<4, 4>(); }
    [[nodiscard]] std::size_t landmark_count() const {
        return static_cast<std::size_t>((n_ - kVehicle) / 2);
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

    // Moves the pose by `motion`, given in the body frame of the current pose, and corrects its
    // yaw by the bias integrated over the motion's duration T:
    //   pose' = pose (+) delta,  yaw' -= tau (1 - phi) beta,  beta' = phi beta,
    //   P_vv' = F P_vv F^T + Jd Sigma Jd^T + diag(0, 0, 0, sigma^2 (1 - phi^2)),  P_vl' = F P_vl,
    // where F is the pose composition's Jacobian with d yaw' / d beta = -tau (1 - phi) and
    // d beta' / d beta = phi. Both compose exactly, so splitting a motion changes nothing.
    // Within one motion, beta's effect on the position (of order v T^2 beta) is neglected: it is
    // far below 0.1 mm over a scan interval.
    void predict(const RelativeMotion& motion);

    // One batch update with every pairing (distinct landmarks), Joseph form:
    //   P' = (I - K H) P (I - K H)^T + K R K^T = M - (M H^T - K R) K^T,  M = P - K W^T,
    // with W = P H^T. M H^T needs M only in the paired columns, so the n x n work is a single
    // product, P - [K T] [W K]^T with T = M H^T - K R, of which only the lower triangle is
    // formed and then mirrored: exactly symmetric, and positive definite where P - K S K^T can
    // lose it to round-off. H is zero in beta's column; beta is updated through its correlation
    // with the yaw. With update_landmarks false it is a Schmidt-Kalman update: the landmarks are
    // "consider" states whose estimates and covariance stay fixed while the vehicle states and
    // their cross-covariance are updated consistently (Schmidt, "Application of state-space
    // methods to navigation problems", Advances in Control Systems 3, 1966), at O(n m) cost.
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
        return kVehicle + 2 * static_cast<Eigen::Index>(landmark);
    }
    void reserve(Eigen::Index size);
    void mirror_lower();  // copies the lower triangle of the active covariance onto the upper

    YawRateBiasModel bias_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;  // capacity x capacity; the active part is the top-left n_ x n_
    Eigen::Index n_ = kVehicle;
};

}  // namespace apex::estimation
