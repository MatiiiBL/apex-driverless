#include "apex/estimation/association.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "apex/core/angles.hpp"
#include "apex/core/rng.hpp"

namespace apex::estimation {
namespace {

// A map of precisely known landmarks and a pose estimate with covariance `pose_covariance`.
EkfSlam make_map(const std::vector<Vec2>& landmarks, const Pose2& estimate,
                 const Mat3& pose_covariance) {
    EkfSlam slam(estimate, 1e-10 * Mat3::Identity());
    for (const Vec2& l : landmarks)
        slam.add_landmark(estimate.to_local(l), 1e-4 * Mat2::Identity());
    RelativeMotion uncertainty;
    uncertainty.covariance = pose_covariance;
    slam.predict(uncertainty);  // no motion: the pose only becomes uncertain
    return slam;
}

ObservationModel observe(const Pose2& truth, const Vec2& cone, double sigma, Rng& rng) {
    return {truth.to_local(cone) + Vec2(sigma * rng.normal(), sigma * rng.normal()),
            sigma * sigma * Mat2::Identity()};
}

Mat3 pose_covariance(double position_std, double yaw_std) {
    return Vec3(position_std * position_std, position_std * position_std, yaw_std * yaw_std)
        .asDiagonal();
}

// Individual compatibility nearest neighbour (ICNN in Neira and Tardos, 2001): each observation
// independently takes the landmark nearest by Mahalanobis distance. The baseline joint
// compatibility must beat.
std::vector<std::int64_t> nearest_neighbour(const EkfSlam& slam,
                                            const std::vector<ObservationModel>& observations,
                                            double gate) {
    std::vector<std::int64_t> result;
    const Eigen::MatrixXd p = slam.covariance();
    for (const ObservationModel& obs : observations) {
        double best = gate;
        std::int64_t best_landmark = -1;
        for (std::size_t j = 0; j < slam.landmark_count(); ++j) {
            const LandmarkMeasurement m = measure_landmark(slam.pose(), slam.landmark(j));
            Eigen::MatrixXd h = Eigen::MatrixXd::Zero(2, p.rows());
            h.leftCols<3>() = m.pose_jacobian;
            h.block<2, 2>(0, 3 + 2 * static_cast<Eigen::Index>(j)) = m.landmark_jacobian;
            const Mat2 s = h * p * h.transpose() + obs.R;
            const Vec2 nu = obs.z - m.predicted;
            const double d2 = nu.dot(s.llt().solve(nu));
            if (d2 < best) {
                best = d2;
                best_landmark = static_cast<std::int64_t>(j);
            }
        }
        result.push_back(best_landmark);
    }
    return result;
}

TEST(Association, PairsClearObservationsWithTheirLandmarks) {
    const std::vector<Vec2> cones = {{5.0, 2.0}, {5.0, -2.0}, {10.0, 2.0}, {10.0, -2.0}};
    const Pose2 truth{0.0, 0.0, 0.0};
    const EkfSlam slam = make_map(cones, truth, pose_covariance(0.05, 0.005));
    Rng rng(1);
    std::vector<ObservationModel> observations;
    for (const std::size_t i : {2U, 0U, 3U})
        observations.push_back(observe(truth, cones[i], 0.03, rng));
    Associator associator({});
    const AssociationResult result = associator.associate(slam, observations);
    EXPECT_EQ(result.landmark, (std::vector<std::int64_t>{2, 0, 3}));
    EXPECT_FALSE(result.ambiguous);
    EXPECT_FALSE(result.budget_exhausted);
    for (const double d2 : result.min_distance2) EXPECT_LT(d2, associator.gate());
}

TEST(Association, JointCompatibilityResolvesWhatNearestNeighbourGetsWrong) {
    // Two rows of cones 3 m apart, seen from a pose 2 m ahead of its estimate (1.3 sigma). Each
    // observation then lands nearer the cone behind its own, and nearest neighbour takes that
    // one. Shifting every pairing by one cone is no explanation, though: the rows bend, so the
    // shifted pairs disagree on the pose error, and only the true pairing passes the joint test.
    Rng rng(4);
    std::vector<Vec2> cones;
    for (int row = 0; row < 2; ++row) {
        for (int i = 0; i < 6; ++i) {
            cones.push_back({4.0 + 3.0 * i + rng.uniform(-0.1, 0.1),
                             (row == 0 ? 1.8 : -1.8) + 0.08 * i * i + rng.uniform(-0.1, 0.1)});
        }
    }
    const Pose2 estimate{0.0, 0.0, 0.0};
    const Pose2 truth{2.0, 0.0, 0.0};
    const EkfSlam slam = make_map(cones, estimate, pose_covariance(1.5, deg_to_rad(1.0)));
    std::vector<ObservationModel> observations;
    std::vector<std::int64_t> expected;
    for (std::size_t i = 0; i < cones.size(); ++i) {
        if (truth.to_local(cones[i]).x() < 1.0) continue;
        observations.push_back(observe(truth, cones[i], 0.03, rng));
        expected.push_back(static_cast<std::int64_t>(i));
    }
    ASSERT_GE(observations.size(), 8U);

    Associator associator({});
    const AssociationResult joint = associator.associate(slam, observations);
    EXPECT_EQ(joint.landmark, expected);
    EXPECT_TRUE(joint.ambiguous);
    EXPECT_FALSE(joint.budget_exhausted);

    const std::vector<std::int64_t> nn = nearest_neighbour(slam, observations, associator.gate());
    std::size_t nn_correct = 0;
    for (std::size_t k = 0; k < nn.size(); ++k) nn_correct += nn[k] == expected[k] ? 1U : 0U;
    EXPECT_LT(nn_correct, expected.size() / 2);  // the scenario defeats the individual rule
}

TEST(Association, TwoLandmarksInsideTheGateGoToTheConsistentObservations) {
    // Two cones 1.5 m apart, both inside each observation's gate. A third, unambiguous cone pins
    // the pose error, which decides which observation belongs to which.
    const std::vector<Vec2> cones = {{8.0, 1.0}, {8.0, 2.5}, {14.0, -2.0}};
    const Pose2 estimate{0.0, 0.0, 0.0};
    const Pose2 truth{0.0, 0.7, 0.0};  // the estimate is 0.7 m to the right of the truth
    const EkfSlam slam = make_map(cones, estimate, pose_covariance(0.6, 0.01));
    Rng rng(2);
    const std::vector<ObservationModel> observations = {observe(truth, cones[0], 0.03, rng),
                                                        observe(truth, cones[1], 0.03, rng),
                                                        observe(truth, cones[2], 0.03, rng)};
    Associator associator({});
    const AssociationResult result = associator.associate(slam, observations);
    EXPECT_EQ(result.landmark, (std::vector<std::int64_t>{0, 1, 2}));
    EXPECT_TRUE(result.ambiguous);
}

TEST(Association, AFalsePositiveBetweenConesStaysUnpaired) {
    // Clutter halfway between two cones, inside both gates when the pose is uncertain: the true
    // observations of those cones claim them, and the clutter is left over.
    const std::vector<Vec2> cones = {{6.0, 1.75}, {9.0, 1.75}, {6.0, -1.75}, {9.0, -1.75}};
    const Pose2 truth{0.0, 0.0, 0.0};
    const EkfSlam slam = make_map(cones, truth, pose_covariance(0.5, 0.02));
    Rng rng(3);
    std::vector<ObservationModel> observations;
    observations.push_back(observe(truth, {7.5, 1.75}, 0.03, rng));  // the false positive
    for (const Vec2& cone : cones) observations.push_back(observe(truth, cone, 0.03, rng));
    Associator associator({});
    const AssociationResult result = associator.associate(slam, observations);
    EXPECT_EQ(result.landmark, (std::vector<std::int64_t>{-1, 0, 1, 2, 3}));
    EXPECT_LT(result.min_distance2[0], associator.gate());  // it was a real contender
}

TEST(Association, AnExhaustedBudgetStillReturnsAJointlyCompatibleAnswer) {
    Rng rng(5);
    std::vector<Vec2> cones;
    for (int i = 0; i < 12; ++i) cones.push_back({3.0 + 2.0 * i, i % 2 == 0 ? 1.75 : -1.75});
    const Pose2 truth{0.4, 0.2, 0.02};
    const EkfSlam slam = make_map(cones, {}, pose_covariance(1.0, 0.05));
    std::vector<ObservationModel> observations;
    for (const Vec2& cone : cones) observations.push_back(observe(truth, cone, 0.05, rng));
    AssociationConfig config;
    config.node_budget = 3;
    Associator limited(config);
    const AssociationResult result = limited.associate(slam, observations);
    EXPECT_TRUE(result.budget_exhausted);
    EXPECT_LE(result.nodes, 3U + observations.size());  // the greedy start plus the budget
    std::size_t paired = 0;
    for (const std::int64_t l : result.landmark) paired += l >= 0 ? 1U : 0U;
    EXPECT_GT(paired, 0U);
}

TEST(Association, ReportsTheNearestDistanceForLandmarkInitialization) {
    const std::vector<Vec2> cones = {{5.0, 2.0}};
    const EkfSlam slam = make_map(cones, {}, pose_covariance(0.05, 0.005));
    Rng rng(6);
    Associator associator({});
    const std::vector<ObservationModel> observations = {observe({}, {5.0, -2.0}, 0.03, rng),
                                                        observe({}, {40.0, 0.0}, 0.03, rng)};
    const AssociationResult result = associator.associate(slam, observations);
    EXPECT_EQ(result.landmark, (std::vector<std::int64_t>{-1, -1}));
    EXPECT_GT(result.min_distance2[0], associator.new_landmark_gate());
    EXPECT_TRUE(std::isinf(result.min_distance2[1]));  // culled before the Mahalanobis test

    const EkfSlam empty({}, Mat3::Identity());
    const AssociationResult none = associator.associate(empty, observations);
    EXPECT_EQ(none.landmark, (std::vector<std::int64_t>{-1, -1}));
}

TEST(Association, GatesAreChiSquareQuantiles) {
    Associator associator({});
    EXPECT_NEAR(associator.gate(), -2.0 * std::log(1.0 - 0.999), 1e-9);
    EXPECT_NEAR(associator.new_landmark_gate(), -2.0 * std::log(1.0 - 0.99999), 1e-8);
}

TEST(Association, ConfigIsValidatedAndRoundTripsThroughJson) {
    AssociationConfig config;
    config.new_landmark_probability = 0.9;  // below the gate probability
    EXPECT_THROW(Associator{config}, std::invalid_argument);
    config = {};
    config.max_candidates = 0;
    EXPECT_THROW(Associator{config}, std::invalid_argument);
    config = {};
    config.gate_probability = 1.0;
    EXPECT_THROW(Associator{config}, std::invalid_argument);

    config = {};
    config.node_budget = 77;
    const auto back = nlohmann::json(config).get<AssociationConfig>();
    EXPECT_EQ(back.node_budget, 77U);
    EXPECT_EQ(back.gate_probability, AssociationConfig{}.gate_probability);
    EXPECT_THROW((void)nlohmann::json({{"gate", 0.9}}).get<AssociationConfig>(),
                 std::invalid_argument);
}

}  // namespace
}  // namespace apex::estimation
