#include "apex/estimation/association.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"

namespace apex::estimation {

void to_json(nlohmann::json& j, const AssociationConfig& config) {
    j = {{"gate_probability", config.gate_probability},
         {"new_landmark_probability", config.new_landmark_probability},
         {"max_candidates", config.max_candidates},
         {"node_budget", config.node_budget}};
}

void from_json(const nlohmann::json& j, AssociationConfig& config) {
    reject_unknown_keys(
        j, {"gate_probability", "new_landmark_probability", "max_candidates", "node_budget"},
        "association");
    read_optional(j, "gate_probability", config.gate_probability);
    read_optional(j, "new_landmark_probability", config.new_landmark_probability);
    read_optional(j, "max_candidates", config.max_candidates);
    read_optional(j, "node_budget", config.node_budget);
}

namespace {

using Mat23 = Eigen::Matrix<double, 2, 3>;

// One individually compatible observation-landmark pairing, with what the joint test needs.
struct Candidate {
    std::size_t landmark = 0;
    double d2 = 0.0;
    Vec2 innovation = Vec2::Zero();
    Mat2 S = Mat2::Zero();  // H P H^T + R
    Mat23 hp = Mat23::Zero();
    Mat2 hl = Mat2::Zero();
    Mat23 u = Mat23::Zero();  // hp P_pp + hl P_lp: shared factor of the cross-covariances
};

// Depth-first branch and bound over the observations in `order` (each has candidates). The
// hypothesis is a stack of pairings with the lower Cholesky factor L of its joint innovation
// covariance and y = L^-1 nu, so its Mahalanobis distance is |y|^2 and adding a pairing only
// appends two rows to L.
class JointCompatibilitySearch {
public:
    JointCompatibilitySearch(const EkfSlam& slam,
                             const std::vector<std::vector<Candidate>>& candidates,
                             std::vector<std::size_t> order, ChiSquareTable& joint,
                             std::size_t budget)
        : slam_(slam),
          candidates_(candidates),
          order_(std::move(order)),
          joint_(joint),
          budget_(budget),
          used_(slam.landmark_count(), 0),
          current_(candidates.size(), -1),
          best_(candidates.size(), -1) {
        const auto dim = static_cast<Eigen::Index>(2 * order_.size());
        L_ = Eigen::MatrixXd::Zero(dim, dim);
        y_ = Eigen::VectorXd::Zero(dim);
        nis_.push_back(0.0);
    }

    void run() {
        for (const std::size_t obs : order_) {
            for (const Candidate& c : candidates_[obs]) {
                if (!used_[c.landmark] && extend(obs, c)) break;
            }
        }
        record();
        while (!pairs_.empty()) retract();
        recurse(0);
    }

    [[nodiscard]] const std::vector<std::int64_t>& best() const { return best_; }
    [[nodiscard]] std::size_t nodes() const { return nodes_; }
    [[nodiscard]] bool exhausted() const { return exhausted_; }

private:
    // H_a P H_b^T for two pairings on different observations and landmarks (their measurement
    // noises are independent, so R does not appear).
    [[nodiscard]] Mat2 cross(const Candidate& a, const Candidate& b) const {
        return a.u * b.hp.transpose() +
               (a.hp * slam_.pose_landmark_covariance(b.landmark) +
                a.hl * slam_.landmark_cross_covariance(a.landmark, b.landmark)) *
                   b.hl.transpose();
    }

    // Appends (obs, c) if the extended hypothesis passes the chi^2_{2(k+1)} joint test.
    bool extend(std::size_t obs, const Candidate& c) {
        ++nodes_;
        const std::size_t k = pairs_.size();
        const auto dim = static_cast<Eigen::Index>(2 * k);
        Mat2 d = c.S;
        Vec2 r = c.innovation;
        Eigen::Matrix<double, Eigen::Dynamic, 2> w(dim, 2);
        if (k > 0) {
            for (std::size_t a = 0; a < k; ++a) {
                w.block<2, 2>(static_cast<Eigen::Index>(2 * a), 0) = cross(*pairs_[a], c);
            }
            L_.topLeftCorner(dim, dim).triangularView<Eigen::Lower>().solveInPlace(w);
            d -= w.transpose() * w;
            r -= w.transpose() * y_.head(dim);
        }
        // 2 x 2 Cholesky of the Schur complement D = S - W^T W.
        if (!(d(0, 0) > 0.0)) return false;
        const double l11 = std::sqrt(d(0, 0));
        const double l21 = d(1, 0) / l11;
        const double l22_sq = d(1, 1) - l21 * l21;
        if (!(l22_sq > 0.0)) return false;
        const double l22 = std::sqrt(l22_sq);
        const double y1 = r.x() / l11;
        const double y2 = (r.y() - l21 * y1) / l22;
        const double nis = nis_.back() + y1 * y1 + y2 * y2;
        if (!(nis < joint_(static_cast<int>(2 * (k + 1))))) return false;
        if (k > 0) L_.block(dim, 0, 2, dim) = w.transpose();
        L_(dim, dim) = l11;
        L_(dim + 1, dim) = l21;
        L_(dim, dim + 1) = 0.0;
        L_(dim + 1, dim + 1) = l22;
        y_(dim) = y1;
        y_(dim + 1) = y2;
        nis_.push_back(nis);
        pairs_.push_back(&c);
        pair_obs_.push_back(obs);
        used_[c.landmark] = 1;
        current_[obs] = static_cast<std::int64_t>(c.landmark);
        return true;
    }

    void retract() {
        used_[pairs_.back()->landmark] = 0;
        current_[pair_obs_.back()] = -1;
        pairs_.pop_back();
        pair_obs_.pop_back();
        nis_.pop_back();
    }

    void record() {
        const std::size_t k = pairs_.size();
        if (k > best_pairs_ || (k == best_pairs_ && nis_.back() < best_nis_)) {
            best_pairs_ = k;
            best_nis_ = nis_.back();
            best_ = current_;
        }
    }

    void recurse(std::size_t level) {
        if (nodes_ >= budget_) {
            exhausted_ = true;
            return;
        }
        // Prune what cannot beat the incumbent: more pairings, or as many with a smaller
        // distance (which only grows as pairings are added).
        const std::size_t reachable = pairs_.size() + order_.size() - level;
        if (reachable < best_pairs_ || (reachable == best_pairs_ && nis_.back() >= best_nis_)) {
            return;
        }
        if (level == order_.size()) {
            record();
            return;
        }
        const std::size_t obs = order_[level];
        for (const Candidate& c : candidates_[obs]) {
            if (used_[c.landmark] || !extend(obs, c)) continue;
            recurse(level + 1);
            retract();
            if (exhausted_) return;
        }
        recurse(level + 1);  // the observation left unpaired
    }

    const EkfSlam& slam_;
    const std::vector<std::vector<Candidate>>& candidates_;
    std::vector<std::size_t> order_;
    ChiSquareTable& joint_;
    std::size_t budget_;
    std::size_t nodes_ = 0;
    bool exhausted_ = false;

    std::vector<const Candidate*> pairs_;
    std::vector<std::size_t> pair_obs_;
    std::vector<double> nis_;  // nis_[k]: joint distance of the first k pairings
    Eigen::MatrixXd L_;
    Eigen::VectorXd y_;
    std::vector<char> used_;  // by landmark
    std::vector<std::int64_t> current_;
    std::vector<std::int64_t> best_;
    std::size_t best_pairs_ = 0;
    double best_nis_ = std::numeric_limits<double>::infinity();
};

}  // namespace

Associator::Associator(const AssociationConfig& config)
    : config_(config),
      gate_(chi_square_quantile(config.gate_probability, 2)),
      new_landmark_gate_(chi_square_quantile(config.new_landmark_probability, 2)),
      joint_(config.gate_probability) {
    if (config.new_landmark_probability < config.gate_probability) {
        throw std::invalid_argument(
            "association: new_landmark_probability must not be below gate_probability");
    }
    if (config.max_candidates < 1) {
        throw std::invalid_argument("association: max_candidates must be at least 1");
    }
}

AssociationResult Associator::associate(const EkfSlam& slam,
                                        std::span<const ObservationModel> observations) {
    AssociationResult result;
    result.landmark.assign(observations.size(), -1);
    result.min_distance2.assign(observations.size(), std::numeric_limits<double>::infinity());
    const std::size_t landmarks = slam.landmark_count();
    if (landmarks == 0 || observations.empty()) return result;

    const Pose2 pose = slam.pose();
    const Mat3 ppp = slam.pose_covariance();
    const double sigma_position = std::sqrt(ppp(0, 0) + ppp(1, 1));
    const double sigma_yaw = std::sqrt(ppp(2, 2));
    std::vector<std::vector<Candidate>> candidates(observations.size());
    std::vector<std::size_t> claims(landmarks, 0);
    for (std::size_t i = 0; i < observations.size(); ++i) {
        const ObservationModel& obs = observations[i];
        const Vec2 world = pose.to_parent(obs.z);
        const double range = obs.z.norm();
        for (std::size_t j = 0; j < landmarks; ++j) {
            const Vec2 landmark = slam.landmark(j);
            const double dist2 = (landmark - world).squaredNorm();
            // |nu| equals the world-frame distance and d^2 >= |nu|^2 / trace(S); trace(S) is
            // bounded from the pose and landmark spreads (Cauchy-Schwarz on the cross term), so
            // landmarks beyond this radius are outside even the new-landmark gate.
            const Mat2 pll = slam.landmark_covariance(j);
            const double lever = sigma_position + (range + std::sqrt(dist2)) * sigma_yaw;
            const double trace_bound = 2.0 * lever * lever + 2.0 * pll.trace() + obs.R.trace();
            if (dist2 > new_landmark_gate_ * trace_bound) continue;

            const LandmarkMeasurement m = measure_landmark(pose, landmark);
            Candidate c;
            c.landmark = j;
            c.hp = m.pose_jacobian;
            c.hl = m.landmark_jacobian;
            const Eigen::Matrix<double, 3, 2> ppl = slam.pose_landmark_covariance(j);
            c.u = c.hp * ppp + c.hl * ppl.transpose();
            c.S = c.u * c.hp.transpose() + (c.hp * ppl + c.hl * pll) * c.hl.transpose() + obs.R;
            c.S = 0.5 * (c.S + c.S.transpose()).eval();
            c.innovation = obs.z - m.predicted;
            const Eigen::LLT<Mat2> llt(c.S);
            if (llt.info() != Eigen::Success) continue;
            c.d2 = c.innovation.dot(llt.solve(c.innovation));
            result.min_distance2[i] = std::min(result.min_distance2[i], c.d2);
            if (c.d2 < gate_) candidates[i].push_back(c);
        }
        auto& list = candidates[i];
        std::sort(list.begin(), list.end(), [](const Candidate& a, const Candidate& b) {
            return a.d2 < b.d2 || (a.d2 == b.d2 && a.landmark < b.landmark);
        });
        if (list.size() > config_.max_candidates) list.resize(config_.max_candidates);
        if (list.size() > 1) result.ambiguous = true;
        for (const Candidate& c : list) {
            if (++claims[c.landmark] > 1) result.ambiguous = true;
        }
    }

    // Most constrained observations first: fewer branches near the root, and the greedy start
    // settles the unambiguous pairings before the contested ones.
    std::vector<std::size_t> order;
    for (std::size_t i = 0; i < observations.size(); ++i) {
        if (!candidates[i].empty()) order.push_back(i);
    }
    if (order.empty()) return result;
    std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return candidates[a].size() < candidates[b].size();
    });
    JointCompatibilitySearch search(slam, candidates, std::move(order), joint_,
                                    config_.node_budget);
    search.run();
    result.landmark = search.best();
    result.nodes = search.nodes();
    result.budget_exhausted = search.exhausted();
    return result;
}

}  // namespace apex::estimation
