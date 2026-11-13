#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/types.hpp"
#include "apex/estimation/chi_square.hpp"
#include "apex/estimation/ekf_slam.hpp"

namespace apex::estimation {

// Thresholds of the data association. docs/estimation.md explains the choices.
struct AssociationConfig {
    // Individual and joint compatibility gates: an innovation of d dimensions is accepted when
    // its Mahalanobis distance is below the chi^2_d quantile of this probability.
    double gate_probability = 0.999;
    // An unmatched observation may start a new landmark only if it is outside every
    // landmark's gate at this, stricter, level. Between the two gates it is ambiguous and
    // dropped, which keeps a mis-predicted re-observation from becoming a duplicate.
    double new_landmark_probability = 0.99999;
    std::size_t max_candidates = 4;  // per observation, the nearest by Mahalanobis distance
    // Hypotheses the joint compatibility search may evaluate per scan before it returns the best
    // so far: bounds the worst case deterministically.
    std::size_t node_budget = 2000;
};

void to_json(nlohmann::json& j, const AssociationConfig& config);
void from_json(const nlohmann::json& j, AssociationConfig& config);

struct ObservationModel {
    Vec2 z = Vec2::Zero();      // body frame (m)
    Mat2 R = Mat2::Identity();  // covariance (m^2)
};

struct AssociationResult {
    // Per observation: the landmark index it is paired with, or -1.
    std::vector<std::int64_t> landmark;
    // Per observation: the smallest Mahalanobis distance to any landmark (+inf if none).
    std::vector<double> min_distance2;
    std::size_t nodes = 0;          // hypotheses evaluated
    bool ambiguous = false;         // some observation or landmark had competing candidates
    bool budget_exhausted = false;  // the search stopped at node_budget
};

// Individual gating, then joint compatibility branch and bound (JCBB) over the gated pairings:
// the largest set of pairings whose stacked innovation passes the chi^2_{2k} test with its full
// covariance, ties broken by the smaller joint Mahalanobis distance (Neira and Tardos, "Data
// association in stochastic mapping using the joint compatibility test", IEEE Trans. Robotics
// and Automation 17(6), 2001). Because the joint covariance carries the pose uncertainty that
// all innovations share, a hypothesis that explains one observation with a shifted landmark
// must explain the others with the same shift: the look-alike neighbours a nearest-neighbour
// rule picks when the pose is uncertain fail the joint test.
//
// The search starts from a greedy hypothesis (each observation, most constrained first, takes
// its nearest compatible landmark), so the common unambiguous scan costs two passes and an
// exhausted budget still returns a jointly compatible answer. Each extension updates a
// Cholesky factor of the joint innovation covariance, O(k^2) for a hypothesis of k pairings.
class Associator {
public:
    explicit Associator(const AssociationConfig& config);

    [[nodiscard]] AssociationResult associate(const EkfSlam& slam,
                                              std::span<const ObservationModel> observations);

    [[nodiscard]] double gate() const { return gate_; }  // chi^2_2 of gate_probability
    [[nodiscard]] double new_landmark_gate() const { return new_landmark_gate_; }

private:
    AssociationConfig config_;
    double gate_;
    double new_landmark_gate_;
    ChiSquareTable joint_;
};

}  // namespace apex::estimation
