#include "apex/estimation/ekf_slam.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
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

EkfSlam::EkfSlam(const Pose2& start, const Mat3& start_covariance, const YawRateBiasModel& bias)
    : bias_(bias) {
    if (!(bias.sigma >= 0.0) || !(bias.correlation_time > 0.0)) {
        throw std::invalid_argument("EkfSlam: invalid yaw-rate bias model");
    }
    reserve(kVehicle + 2 * 64);
    x_.head<4>() << start.x, start.y, wrap_angle(start.yaw), 0.0;
    P_.topLeftCorner<3, 3>() = start_covariance;
    P_(3, 3) = bias.sigma * bias.sigma;
}

double EkfSlam::bias_yaw_gain(double t) const {
    return bias_.correlation_time * -std::expm1(-t / bias_.correlation_time);
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

void EkfSlam::mirror_lower() {
    for (Eigen::Index j = 1; j < n_; ++j) {
        for (Eigen::Index i = 0; i < j; ++i) P_(i, j) = P_(j, i);
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
    const double gain = bias_yaw_gain(motion.duration);  // tau (1 - phi)
    const double phi = 1.0 - gain / bias_.correlation_time;
    x_.head<4>() << moved.x, moved.y, wrap_angle(moved.yaw - gain * x_[3]), phi * x_[3];
    Mat4 f = Mat4::Identity();
    f.topLeftCorner<3, 3>() = jx;
    f(2, 3) = -gain;
    f(3, 3) = phi;
    Mat4 q = Mat4::Zero();
    q.topLeftCorner<3, 3>() = jd * motion.covariance * jd.transpose();
    q(3, 3) = bias_.sigma * bias_.sigma * (1.0 - phi * phi);
    const Mat4 pvv = P_.topLeftCorner<4, 4>();
    P_.topLeftCorner<4, 4>() = f * pvv * f.transpose() + q;
    if (n_ > kVehicle) {
        const Eigen::Index m = n_ - kVehicle;
        const Eigen::Matrix<double, 4, Eigen::Dynamic> cross = f * P_.block(0, kVehicle, 4, m);
        P_.block(0, kVehicle, 4, m) = cross;
        P_.block(kVehicle, 0, m, 4) = cross.transpose();
    }
}

bool EkfSlam::update(std::span<const Pairing> pairings, bool update_landmarks) {
    if (pairings.empty()) return true;
    const auto k = static_cast<Eigen::Index>(pairings.size());
    const Eigen::Index m = 2 * k;
    const Eigen::Index c = 3 + 2 * k;
    const Pose2 pose = this->pose();

    // H is zero outside the pose (x, y, yaw: not beta) and the paired landmarks' columns: work
    // on those c columns.
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
        const Eigen::MatrixXd gain = llt.solve(w.transpose()).transpose();  // K, n x m
        x_.head(n_) += gain * nu;
        // M = (I - K H) P = P - K W^T is needed only in the paired columns, for M H^T.
        const Eigen::MatrixXd mc = pc - gain * wc.transpose();     // n x c
        const Eigen::MatrixXd t = mc * hc.transpose() - gain * r;  // M H^T - K R, n x m
        // P' = M - (M H^T - K R) K^T = P - [K T] [W K]^T is symmetric: form its lower triangle
        // only, as one product of inner dimension 2m, and mirror it.
        Eigen::MatrixXd left(n_, 2 * m);
        left << gain, t;
        Eigen::MatrixXd right(n_, 2 * m);
        right << w, gain;
        P.triangularView<Eigen::Lower>() -= left * right.transpose();
        mirror_lower();
    } else {
        // Only the vehicle rows (v) have a gain. With M_v = P_v - K_v W^T, the vehicle rows of
        // (I - K H) P, the Joseph form's vehicle block is M_v (I - K H)_v^T + K_v R K_v^T
        // = M_vv - (M_v H^T) K_v^T + K_v R K_v^T, and M_v H^T needs M_v in the paired columns.
        const Eigen::MatrixXd gain = llt.solve(w.topRows<4>().transpose()).transpose();  // 4 x m
        x_.head<4>() += gain * nu;
        const Eigen::MatrixXd mp = P.topRows<4>() - gain * w.transpose();
        Eigen::MatrixXd mpc(4, c);
        for (Eigen::Index j = 0; j < c; ++j) mpc.col(j) = mp.col(cols[static_cast<std::size_t>(j)]);
        Mat4 pvv = mp.leftCols<4>() - (mpc * hc.transpose()) * gain.transpose() +
                   gain * r * gain.transpose();
        pvv = 0.5 * (pvv + pvv.transpose()).eval();
        P.topRows<4>() = mp;
        P.leftCols<4>() = mp.transpose();
        P.topLeftCorner<4, 4>() = pvv;
    }
    x_[2] = wrap_angle(x_[2]);
    return true;
}

std::size_t EkfSlam::add_landmark(const Vec2& z, const Mat2& R) {
    reserve(n_ + 2);
    const Pose2 pose = this->pose();
    const Mat2 rot = rotation(pose.yaw);
    // d l / d (x, y, yaw, beta); beta does not enter.
    Eigen::Matrix<double, 2, 4> gx;
    gx << 1.0, 0.0, -std::sin(pose.yaw) * z.x() - std::cos(pose.yaw) * z.y(), 0.0, 0.0, 1.0,
        std::cos(pose.yaw) * z.x() - std::sin(pose.yaw) * z.y(), 0.0;
    const Eigen::Matrix<double, 2, Eigen::Dynamic> cross = gx * P_.topRows<4>().leftCols(n_);
    const Mat2 pll = gx * P_.topLeftCorner<4, 4>() * gx.transpose() + rot * R * rot.transpose();
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
