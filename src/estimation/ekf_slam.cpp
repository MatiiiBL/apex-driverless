#include "apex/estimation/ekf_slam.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Cholesky>

#include "apex/core/angles.hpp"

namespace apex::estimation {

namespace {

Mat2 rotation(double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Mat2 r;
    r << c, -s, s, c;
    return r;
}

}  // namespace

LandmarkMeasurement measure_landmark(const Pose2& pose, const Vec2& landmark) {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    const double dx = landmark.x() - pose.x;
    const double dy = landmark.y() - pose.y;
    LandmarkMeasurement m;
    m.predicted = {c * dx + s * dy, -s * dx + c * dy};
    m.landmark_jacobian << c, s, -s, c;
    m.pose_jacobian << -c, -s, -s * dx + c * dy, s, -c, -c * dx - s * dy;
    return m;
}

EkfSlam::EkfSlam(const Pose2& start, const Mat3& start_covariance) {
    reserve(3 + 2 * 64);
    x_.head<3>() << start.x, start.y, wrap_angle(start.yaw);
    P_.topLeftCorner<3, 3>() = start_covariance;
}

void EkfSlam::reserve(Eigen::Index size) {
    if (size <= x_.size()) return;
    const Eigen::Index capacity = std::max(size, 2 * x_.size());
    Eigen::VectorXd x = Eigen::VectorXd::Zero(capacity);
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(capacity, capacity);
    if (x_.size() > 0) {
        x.head(n_) = x_.head(n_);
        P.topLeftCorner(n_, n_) = P_.topLeftCorner(n_, n_);
    }
    x_.swap(x);
    P_.swap(P);
}

void EkfSlam::symmetrize() {
    for (Eigen::Index j = 0; j < n_; ++j) {
        for (Eigen::Index i = j + 1; i < n_; ++i) {
            const double mean = 0.5 * (P_(i, j) + P_(j, i));
            P_(i, j) = mean;
            P_(j, i) = mean;
        }
    }
}

void EkfSlam::predict(const RelativeMotion& motion) {
    const Pose2 pose = this->pose();
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    const double dx = motion.delta.x;
    const double dy = motion.delta.y;
    Mat3 jx = Mat3::Identity();
    jx(0, 2) = -s * dx - c * dy;
    jx(1, 2) = c * dx - s * dy;
    Mat3 jd = Mat3::Identity();
    jd.topLeftCorner<2, 2>() = rotation(pose.yaw);
    const Pose2 moved = pose.compose(motion.delta);
    x_.head<3>() << moved.x, moved.y, moved.yaw;
    const Mat3 ppp = P_.topLeftCorner<3, 3>();
    P_.topLeftCorner<3, 3>() = jx * ppp * jx.transpose() + jd * motion.covariance * jd.transpose();
    if (n_ > 3) {
        const Eigen::Index m = n_ - 3;
        const Eigen::Matrix<double, 3, Eigen::Dynamic> cross = jx * P_.block(0, 3, 3, m);
        P_.block(0, 3, 3, m) = cross;
        P_.block(3, 0, m, 3) = cross.transpose();
    }
}

bool EkfSlam::update(std::span<const Pairing> pairings, bool update_landmarks) {
    if (pairings.empty()) return true;
    const auto k = static_cast<Eigen::Index>(pairings.size());
    const Eigen::Index m = 2 * k;
    const Eigen::Index c = 3 + 2 * k;
    const Pose2 pose = this->pose();

    // H is zero outside the pose and the paired landmarks' columns: work on those c columns.
    std::vector<Eigen::Index> cols(static_cast<std::size_t>(c));
    Eigen::MatrixXd hc = Eigen::MatrixXd::Zero(m, c);
    Eigen::VectorXd nu(m);
    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(m, m);
    for (Eigen::Index i = 0; i < 3; ++i) cols[static_cast<std::size_t>(i)] = i;
    for (Eigen::Index a = 0; a < k; ++a) {
        const Pairing& pairing = pairings[static_cast<std::size_t>(a)];
        const Eigen::Index li = index(pairing.landmark);
        cols[static_cast<std::size_t>(3 + 2 * a)] = li;
        cols[static_cast<std::size_t>(4 + 2 * a)] = li + 1;
        const LandmarkMeasurement meas = measure_landmark(pose, x_.segment<2>(li));
        hc.block<2, 3>(2 * a, 0) = meas.pose_jacobian;
        hc.block<2, 2>(2 * a, 3 + 2 * a) = meas.landmark_jacobian;
        nu.segment<2>(2 * a) = pairing.z - meas.predicted;
        r.block<2, 2>(2 * a, 2 * a) = pairing.R;
    }

    auto P = P_.topLeftCorner(n_, n_);
    Eigen::MatrixXd pc(n_, c);
    for (Eigen::Index j = 0; j < c; ++j) pc.col(j) = P.col(cols[static_cast<std::size_t>(j)]);
    const Eigen::MatrixXd w = pc * hc.transpose();  // P H^T, n x m
    Eigen::MatrixXd wc(c, m);
    for (Eigen::Index j = 0; j < c; ++j) wc.row(j) = w.row(cols[static_cast<std::size_t>(j)]);
    Eigen::MatrixXd s = hc * wc + r;
    s = 0.5 * (s + s.transpose()).eval();
    const Eigen::LLT<Eigen::MatrixXd> llt(s);
    if (llt.info() != Eigen::Success) return false;

    if (update_landmarks) {
        const Eigen::MatrixXd gain = llt.solve(w.transpose()).transpose();  // n x m
        x_.head(n_) += gain * nu;
        P.noalias() -= gain * w.transpose();  // M = (I - K H) P
        for (Eigen::Index j = 0; j < c; ++j) pc.col(j) = P.col(cols[static_cast<std::size_t>(j)]);
        const Eigen::MatrixXd t = pc * hc.transpose() - gain * r;  // M H^T - K R
        P.noalias() -= t * gain.transpose();
        symmetrize();
    } else {
        const Eigen::MatrixXd gain = llt.solve(w.topRows<3>().transpose()).transpose();  // 3 x m
        x_.head<3>() += gain * nu;
        const Eigen::MatrixXd mp = P.topRows<3>() - gain * w.transpose();  // rows of (I - K H) P
        Eigen::MatrixXd apc = -gain * hc;                                  // pose rows of I - K H
        apc.leftCols<3>() += Mat3::Identity();
        Eigen::MatrixXd mpc(3, c);
        for (Eigen::Index j = 0; j < c; ++j) mpc.col(j) = mp.col(cols[static_cast<std::size_t>(j)]);
        Mat3 ppp = mpc * apc.transpose() + gain * r * gain.transpose();
        ppp = 0.5 * (ppp + ppp.transpose()).eval();
        P.topRows<3>() = mp;
        P.leftCols<3>() = mp.transpose();
        P.topLeftCorner<3, 3>() = ppp;
    }
    x_[2] = wrap_angle(x_[2]);
    return true;
}

std::size_t EkfSlam::add_landmark(const Vec2& z, const Mat2& R) {
    reserve(n_ + 2);
    const Pose2 pose = this->pose();
    const Mat2 rot = rotation(pose.yaw);
    Eigen::Matrix<double, 2, 3> gx;
    gx << 1.0, 0.0, -std::sin(pose.yaw) * z.x() - std::cos(pose.yaw) * z.y(), 0.0, 1.0,
        std::cos(pose.yaw) * z.x() - std::sin(pose.yaw) * z.y();
    const Eigen::Matrix<double, 2, Eigen::Dynamic> cross = gx * P_.topRows<3>().leftCols(n_);
    const Mat2 pll = gx * P_.topLeftCorner<3, 3>() * gx.transpose() + rot * R * rot.transpose();
    x_.segment<2>(n_) = pose.to_parent(z);
    P_.block(n_, 0, 2, n_) = cross;
    P_.block(0, n_, n_, 2) = cross.transpose();
    P_.block<2, 2>(n_, n_) = 0.5 * (pll + pll.transpose());
    const std::size_t landmark = landmark_count();
    n_ += 2;
    return landmark;
}

void EkfSlam::remove_landmark(std::size_t i) {
    const Eigen::Index target = index(i);
    const Eigen::Index last = n_ - 2;
    if (target != last) {
        x_.segment<2>(target) = x_.segment<2>(last);
        // Rows first, then columns: the column copy also moves the diagonal block into place.
        P_.block(target, 0, 2, n_) = P_.block(last, 0, 2, n_);
        P_.block(0, target, n_, 2) = P_.block(0, last, n_, 2);
    }
    n_ -= 2;
}

bool EkfSlam::finite() const {
    return x_.head(n_).allFinite() && P_.topLeftCorner(n_, n_).allFinite();
}

}  // namespace apex::estimation
