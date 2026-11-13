#include "apex/estimation/ekf_slam.hpp"

#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"

namespace apex::estimation {
namespace {

Eigen::VectorXd state_of(const EkfSlam& slam) {
    Eigen::VectorXd x(static_cast<Eigen::Index>(slam.state_size()));
    const Pose2 p = slam.pose();
    x.head<3>() << p.x, p.y, p.yaw;
    for (std::size_t i = 0; i < slam.landmark_count(); ++i) {
        x.segment<2>(3 + 2 * static_cast<Eigen::Index>(i)) = slam.landmark(i);
    }
    return x;
}

Mat2 random_covariance(Rng& rng, double scale) {
    Mat2 a;
    a << rng.normal(), rng.normal(), rng.normal(), rng.normal();
    return scale * (a * a.transpose() + 0.2 * Mat2::Identity());
}

// A filter with correlated pose and landmarks: start, add landmarks, move, observe some.
EkfSlam make_filter(Rng& rng, std::size_t landmarks) {
    Mat3 p0 = Mat3::Zero();
    p0.diagonal() << 0.04, 0.02, 0.001;
    EkfSlam slam({2.0, -1.0, 0.3}, p0);
    for (std::size_t i = 0; i < landmarks; ++i) {
        slam.add_landmark({rng.uniform(2.0, 20.0), rng.uniform(-8.0, 8.0)},
                          random_covariance(rng, 0.01));
        RelativeMotion motion;
        motion.delta = {0.5, 0.02, 0.03};
        motion.covariance = Vec3(1e-3, 5e-4, 1e-4).asDiagonal();
        slam.predict(motion);
    }
    return slam;
}

TEST(EkfSlam, MeasurementJacobiansMatchFiniteDifferences) {
    const Pose2 pose{1.0, 2.0, 0.7};
    const Vec2 landmark{8.0, -3.0};
    const LandmarkMeasurement m = measure_landmark(pose, landmark);
    EXPECT_LT((m.predicted - pose.to_local(landmark)).norm(), 1e-12);
    const double h = 1e-6;
    for (int i = 0; i < 3; ++i) {
        Vec3 d = Vec3::Zero();
        d[i] = h;
        const Pose2 plus{pose.x + d.x(), pose.y + d.y(), pose.yaw + d.z()};
        const Pose2 minus{pose.x - d.x(), pose.y - d.y(), pose.yaw - d.z()};
        const Vec2 numeric = (plus.to_local(landmark) - minus.to_local(landmark)) / (2.0 * h);
        EXPECT_LT((m.pose_jacobian.col(i) - numeric).norm(), 1e-8) << i;
    }
    for (int i = 0; i < 2; ++i) {
        Vec2 d = Vec2::Zero();
        d[i] = h;
        const Vec2 numeric =
            (pose.to_local(landmark + d) - pose.to_local(landmark - d)) / (2.0 * h);
        EXPECT_LT((m.landmark_jacobian.col(i) - numeric).norm(), 1e-8) << i;
    }
}

TEST(EkfSlam, AddLandmarkPropagatesPoseAndObservationUncertainty) {
    Rng rng(1);
    EkfSlam slam = make_filter(rng, 3);
    const Eigen::MatrixXd before = slam.covariance();
    const Pose2 pose = slam.pose();
    const Vec2 z{6.0, 2.5};
    const Mat2 r = random_covariance(rng, 0.02);
    const std::size_t index = slam.add_landmark(z, r);
    EXPECT_EQ(index, 3U);
    EXPECT_LT((slam.landmark(index) - pose.to_parent(z)).norm(), 1e-12);

    // l = p + R(yaw) z: Jacobians with respect to the state and to z.
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(2, before.rows());
    g(0, 0) = 1.0;
    g(1, 1) = 1.0;
    g(0, 2) = -s * z.x() - c * z.y();
    g(1, 2) = c * z.x() - s * z.y();
    Mat2 rot;
    rot << c, -s, s, c;
    const Eigen::MatrixXd cross = g * before;
    const Mat2 own = g * before * g.transpose() + rot * r * rot.transpose();
    const Eigen::MatrixXd after = slam.covariance();
    const Eigen::Index n = before.rows();
    EXPECT_LT((after.topLeftCorner(n, n) - before).cwiseAbs().maxCoeff(), 1e-15);
    EXPECT_LT((after.block(n, 0, 2, n) - cross).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((after.block(0, n, n, 2) - cross.transpose()).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT((after.block<2, 2>(n, n) - own).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(EkfSlam, PredictMovesThePoseAndItsCorrelations) {
    Rng rng(2);
    EkfSlam slam = make_filter(rng, 4);
    const Eigen::MatrixXd p = slam.covariance();
    const Pose2 pose = slam.pose();
    RelativeMotion motion;
    motion.delta = {1.2, -0.1, 0.05};
    motion.covariance << 2e-3, 1e-4, 0.0, 1e-4, 1e-3, 0.0, 0.0, 0.0, 4e-4;
    slam.predict(motion);
    const Pose2 expected = pose.compose(motion.delta);
    EXPECT_NEAR(slam.pose().x, expected.x, 1e-12);
    EXPECT_NEAR(slam.pose().y, expected.y, 1e-12);
    EXPECT_NEAR(slam.pose().yaw, expected.yaw, 1e-12);

    // Dense: F P F^T + G Q G^T with F = diag(Jx, I) and G the motion's Jacobian on the pose.
    const Eigen::Index n = p.rows();
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    Eigen::MatrixXd f = Eigen::MatrixXd::Identity(n, n);
    f(0, 2) = -s * motion.delta.x - c * motion.delta.y;
    f(1, 2) = c * motion.delta.x - s * motion.delta.y;
    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(n, 3);
    g.topLeftCorner<3, 3>() << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
    const Eigen::MatrixXd expected_p =
        f * p * f.transpose() + g * motion.covariance * g.transpose();
    EXPECT_LT((slam.covariance() - expected_p).cwiseAbs().maxCoeff(), 1e-12);
}

// The textbook EKF update with the Joseph form on the dense state, as the oracle.
struct DenseUpdate {
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
};

DenseUpdate dense_update(const EkfSlam& slam, const std::vector<Pairing>& pairings,
                         bool update_landmarks) {
    const Eigen::VectorXd x = state_of(slam);
    const Eigen::MatrixXd p = slam.covariance();
    const Eigen::Index n = x.size();
    const auto m = static_cast<Eigen::Index>(2 * pairings.size());
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(m, n);
    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(m, m);
    Eigen::VectorXd nu(m);
    for (std::size_t a = 0; a < pairings.size(); ++a) {
        const auto row = static_cast<Eigen::Index>(2 * a);
        const auto col = static_cast<Eigen::Index>(3 + 2 * pairings[a].landmark);
        const LandmarkMeasurement meas =
            measure_landmark(slam.pose(), slam.landmark(pairings[a].landmark));
        h.block<2, 3>(row, 0) = meas.pose_jacobian;
        h.block<2, 2>(row, col) = meas.landmark_jacobian;
        r.block<2, 2>(row, row) = pairings[a].R;
        nu.segment<2>(row) = pairings[a].z - meas.predicted;
    }
    const Eigen::MatrixXd s = h * p * h.transpose() + r;
    Eigen::MatrixXd k = p * h.transpose() * s.inverse();
    if (!update_landmarks) k.bottomRows(n - 3).setZero();  // Schmidt: landmarks are considered
    const Eigen::MatrixXd a = Eigen::MatrixXd::Identity(n, n) - k * h;
    DenseUpdate out;
    out.x = x + k * nu;
    out.x[2] = wrap_angle(out.x[2]);
    out.P = a * p * a.transpose() + k * r * k.transpose();
    return out;
}

std::vector<Pairing> observe(Rng& rng, const EkfSlam& slam, std::vector<std::size_t> landmarks) {
    std::vector<Pairing> pairings;
    for (const std::size_t i : landmarks) {
        Pairing pairing;
        pairing.landmark = i;
        pairing.R = random_covariance(rng, 0.005);
        pairing.z =
            slam.pose().to_local(slam.landmark(i)) + Vec2(0.1 * rng.normal(), 0.1 * rng.normal());
        pairings.push_back(pairing);
    }
    return pairings;
}

TEST(EkfSlam, UpdateMatchesTheDenseJosephForm) {
    Rng rng(3);
    EkfSlam slam = make_filter(rng, 7);
    const std::vector<Pairing> pairings = observe(rng, slam, {5, 1, 3});
    const DenseUpdate expected = dense_update(slam, pairings, true);
    ASSERT_TRUE(slam.update(pairings, true));
    EXPECT_LT((state_of(slam) - expected.x).cwiseAbs().maxCoeff(), 1e-10);
    EXPECT_LT((slam.covariance() - expected.P).cwiseAbs().maxCoeff(), 1e-12);
    const Eigen::MatrixXd p = slam.covariance();
    EXPECT_EQ(p, p.transpose());
    EXPECT_EQ(Eigen::LLT<Eigen::MatrixXd>(p).info(), Eigen::Success);
}

TEST(EkfSlam, SchmidtUpdateLeavesTheLandmarksAndCorrectsThePose) {
    Rng rng(4);
    EkfSlam slam = make_filter(rng, 6);
    const std::vector<Pairing> pairings = observe(rng, slam, {0, 4});
    const Eigen::VectorXd x_before = state_of(slam);
    const Eigen::MatrixXd p_before = slam.covariance();
    const DenseUpdate expected = dense_update(slam, pairings, false);
    ASSERT_TRUE(slam.update(pairings, false));
    const Eigen::Index n = x_before.size();
    EXPECT_EQ(state_of(slam).tail(n - 3), x_before.tail(n - 3));
    EXPECT_EQ(slam.covariance().bottomRightCorner(n - 3, n - 3),
              p_before.bottomRightCorner(n - 3, n - 3));
    EXPECT_LT((state_of(slam) - expected.x).cwiseAbs().maxCoeff(), 1e-10);
    EXPECT_LT((slam.covariance() - expected.P).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LT(slam.pose_covariance().trace(), (p_before.topLeftCorner<3, 3>().trace()));
    EXPECT_EQ(Eigen::LLT<Eigen::MatrixXd>(slam.covariance()).info(), Eigen::Success);
}

TEST(EkfSlam, RefusesAnUpdateWithoutAPositiveDefiniteInnovationCovariance) {
    Rng rng(5);
    EkfSlam slam = make_filter(rng, 3);
    std::vector<Pairing> pairings = observe(rng, slam, {1});
    pairings[0].R = -10.0 * Mat2::Identity();
    const Eigen::VectorXd x = state_of(slam);
    const Eigen::MatrixXd p = slam.covariance();
    EXPECT_FALSE(slam.update(pairings, true));
    EXPECT_EQ(state_of(slam), x);
    EXPECT_EQ(slam.covariance(), p);
    EXPECT_TRUE(slam.update({}, true));  // nothing to do
}

TEST(EkfSlam, RemoveLandmarkMovesTheLastIntoItsSlot) {
    Rng rng(6);
    EkfSlam slam = make_filter(rng, 5);
    const Eigen::VectorXd x = state_of(slam);
    const Eigen::MatrixXd p = slam.covariance();
    slam.remove_landmark(1);
    ASSERT_EQ(slam.landmark_count(), 4U);
    // Order after removal: pose, l0, l4, l2, l3.
    const std::vector<Eigen::Index> keep = {0, 1, 2, 3, 4, 11, 12, 7, 8, 9, 10};
    for (std::size_t i = 0; i < keep.size(); ++i) {
        const auto a = static_cast<Eigen::Index>(i);
        EXPECT_EQ(state_of(slam)[a], x[keep[i]]);
        for (std::size_t j = 0; j < keep.size(); ++j) {
            EXPECT_EQ(slam.covariance()(a, static_cast<Eigen::Index>(j)), p(keep[i], keep[j]));
        }
    }
    slam.remove_landmark(3);  // the last one: no move
    EXPECT_EQ(slam.landmark_count(), 3U);
    EXPECT_EQ(slam.landmark(2), x.segment<2>(7));
}

TEST(EkfSlam, GrowsPastItsInitialCapacity) {
    Rng rng(7);
    EkfSlam slam = make_filter(rng, 150);
    EXPECT_EQ(slam.landmark_count(), 150U);
    EXPECT_EQ(slam.state_size(), 303U);
    const std::vector<Pairing> pairings = observe(rng, slam, {0, 70, 149});
    const DenseUpdate expected = dense_update(slam, pairings, true);
    ASSERT_TRUE(slam.update(pairings, true));
    EXPECT_LT((slam.covariance() - expected.P).cwiseAbs().maxCoeff(), 1e-10);
    EXPECT_TRUE(slam.finite());
}

}  // namespace
}  // namespace apex::estimation
