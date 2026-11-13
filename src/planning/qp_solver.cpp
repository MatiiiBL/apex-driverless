#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Cholesky>

#include "apex/planning/qp.hpp"

namespace apex::planning {

namespace {

using Eigen::Index;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Fraction of the distance to the boundary of the positive orthant taken by each step.
constexpr double kStepFraction = 0.99;

// The constraints in the form C x >= d, stacked as: finite lower bounds (x_i >= l_i), finite
// upper bounds (-x_i >= -u_i), then the general rows (-G x >= -h). The bound rows of C are
// never formed: they only index into x.
class Constraints {
public:
    explicit Constraints(const QpProblem& problem) : G_(problem.G), n_(problem.q.size()) {
        for (Index i = 0; i < n_; ++i) {
            if (problem.lower.size() > 0 && std::isfinite(problem.lower[i])) lower_.push_back(i);
            if (problem.upper.size() > 0 && std::isfinite(problem.upper[i])) upper_.push_back(i);
        }
        d_.resize(rows());
        Index r = 0;
        for (const Index i : lower_) d_[r++] = problem.lower[i];
        for (const Index i : upper_) d_[r++] = -problem.upper[i];
        d_.tail(G_.rows()) = -problem.h;
    }

    [[nodiscard]] Index rows() const {
        return static_cast<Index>(lower_.size() + upper_.size()) + G_.rows();
    }
    [[nodiscard]] const VectorXd& d() const { return d_; }

    [[nodiscard]] VectorXd times(const VectorXd& x) const {  // C x
        VectorXd result(rows());
        Index r = 0;
        for (const Index i : lower_) result[r++] = x[i];
        for (const Index i : upper_) result[r++] = -x[i];
        if (G_.rows() > 0) result.tail(G_.rows()).noalias() = -(G_ * x);
        return result;
    }

    [[nodiscard]] VectorXd transpose_times(const VectorXd& y) const {  // C' y
        VectorXd result = VectorXd::Zero(n_);
        Index r = 0;
        for (const Index i : lower_) result[i] += y[r++];
        for (const Index i : upper_) result[i] -= y[r++];
        if (G_.rows() > 0) result.noalias() -= G_.transpose() * y.tail(G_.rows());
        return result;
    }

    // K += C' diag(w) C.
    void add_normal_matrix(const VectorXd& w, MatrixXd& K) const {
        Index r = 0;
        for (const Index i : lower_) K(i, i) += w[r++];
        for (const Index i : upper_) K(i, i) += w[r++];
        if (G_.rows() > 0) {
            K.noalias() += G_.transpose() * w.tail(G_.rows()).asDiagonal() * G_;
        }
    }

private:
    const MatrixXd& G_;
    Index n_ = 0;
    std::vector<Index> lower_;
    std::vector<Index> upper_;
    VectorXd d_;
};

double inf_norm(const VectorXd& v) {
    return v.size() > 0 ? v.lpNorm<Eigen::Infinity>() : 0.0;
}

// The largest alpha in (0, 1] keeping s + alpha ds >= 0 and z + alpha dz >= 0.
double max_step(const VectorXd& s, const VectorXd& ds, const VectorXd& z, const VectorXd& dz) {
    double alpha = 1.0;
    for (Index i = 0; i < s.size(); ++i) {
        if (ds[i] < 0.0) alpha = std::min(alpha, -s[i] / ds[i]);
        if (dz[i] < 0.0) alpha = std::min(alpha, -z[i] / dz[i]);
    }
    return alpha;
}

void check_dimensions(const QpProblem& p) {
    const Index n = p.q.size();
    const bool ok = p.P.rows() == n && p.P.cols() == n &&
                    (p.lower.size() == 0 || p.lower.size() == n) &&
                    (p.upper.size() == 0 || p.upper.size() == n) && p.G.rows() == p.h.size() &&
                    (p.G.rows() == 0 || p.G.cols() == n);
    if (!ok) throw std::invalid_argument("solve_qp: inconsistent problem dimensions");
}

// Cholesky of K, with a growing diagonal shift if K is only semidefinite (a singular P with
// unbounded variables). False if even that fails.
bool factor(MatrixXd& K, Eigen::LLT<MatrixXd>& llt) {
    llt.compute(K);
    double shift = 1e-12 * std::max(1.0, K.diagonal().cwiseAbs().maxCoeff());
    for (int attempt = 0; attempt < 6 && llt.info() != Eigen::Success; ++attempt) {
        K.diagonal().array() += shift;
        llt.compute(K);
        shift *= 100.0;
    }
    return llt.info() == Eigen::Success;
}

struct Step {
    VectorXd dx;
    VectorXd ds;
    VectorXd dz;
};

}  // namespace

QpResult solve_qp(const QpProblem& problem, const QpSettings& settings) {
    check_dimensions(problem);
    const Index n = problem.q.size();
    constexpr double kInf = std::numeric_limits<double>::infinity();
    QpResult result;

    // Start inside the box where possible; the slacks absorb any infeasibility.
    VectorXd x = VectorXd::Zero(n);
    for (Index i = 0; i < n; ++i) {
        const double lo = problem.lower.size() > 0 ? problem.lower[i] : -kInf;
        const double hi = problem.upper.size() > 0 ? problem.upper[i] : kInf;
        if (!(lo <= hi)) {
            result.x = x;
            result.status = QpStatus::InvalidBounds;
            return result;
        }
        x[i] = std::clamp(0.0, lo, hi);
    }

    const Constraints C(problem);
    const Index m = C.rows();
    const auto rows = static_cast<double>(m);
    VectorXd s = (C.times(x) - C.d()).cwiseMax(1.0);  // slacks of C x - d >= 0
    VectorXd z = VectorXd::Ones(m);                   // their multipliers
    const double tol = settings.tolerance;
    const double dual_scale = 1.0 + inf_norm(problem.q);
    const double primal_scale = 1.0 + inf_norm(C.d());
    const auto objective = [&problem](const VectorXd& v) {
        return 0.5 * v.dot(problem.P * v) + problem.q.dot(v);
    };

    MatrixXd K(n, n);
    Eigen::LLT<MatrixXd> llt;
    for (std::size_t iteration = 0;; ++iteration) {
        // KKT residuals: stationarity P x + q - C' z = 0, primal C x - s - d = 0, and the
        // complementarity gap s' z.
        const VectorXd r_dual = problem.P * x + problem.q - C.transpose_times(z);
        const VectorXd r_primal = C.times(x) - s - C.d();
        const double gap = s.dot(z);
        result.x = x;
        result.iterations = iteration;
        result.objective = objective(x);
        if (inf_norm(r_dual) <= tol * dual_scale && inf_norm(r_primal) <= tol * primal_scale &&
            gap <= tol * (1.0 + std::abs(result.objective))) {
            result.status = QpStatus::Solved;
            return result;
        }
        if (iteration == settings.max_iterations) break;

        // Newton step on the perturbed KKT system for a complementarity target r_c (s z = r_c
        // after the step), with ds and dz eliminated:
        //   (P + C' S^-1 Z C) dx = -r_dual - C' S^-1 (r_c + Z r_primal)
        //   ds = C dx + r_primal,   dz = -S^-1 (r_c + Z ds).
        K = problem.P;
        C.add_normal_matrix(z.cwiseQuotient(s), K);
        if (!factor(K, llt)) break;
        const auto solve = [&](const VectorXd& r_c) {
            const VectorXd t = (r_c + z.cwiseProduct(r_primal)).cwiseQuotient(s);
            Step step;
            step.dx = llt.solve(-r_dual - C.transpose_times(t));
            step.ds = C.times(step.dx) + r_primal;
            step.dz = -(r_c + z.cwiseProduct(step.ds)).cwiseQuotient(s);
            return step;
        };

        // Predictor: the affine-scaling direction, which aims straight at s z = 0.
        const Step affine = solve(s.cwiseProduct(z));
        if (m == 0) {  // unconstrained: a plain Newton step
            x += affine.dx;
            continue;
        }
        if (iteration == 0) {
            // Starting-point heuristic (Nocedal and Wright, section 16.6): keep x, move the
            // slacks and multipliers to where the affine step would take them, at least 1.
            s = (s + affine.ds).cwiseAbs().cwiseMax(1.0);
            z = (z + affine.dz).cwiseAbs().cwiseMax(1.0);
            continue;
        }
        // Corrector: center by sigma = (mu_affine / mu)^3 (Mehrotra's heuristic) and cancel the
        // affine step's second-order complementarity error ds_aff dz_aff.
        const double mu = gap / rows;
        const double alpha_affine = max_step(s, affine.ds, z, affine.dz);
        const double mu_affine =
            (s + alpha_affine * affine.ds).dot(z + alpha_affine * affine.dz) / rows;
        const double sigma = std::pow(mu_affine / mu, 3);
        const Step step = solve(s.cwiseProduct(z) + affine.ds.cwiseProduct(affine.dz) -
                                VectorXd::Constant(m, sigma * mu));
        const double alpha = std::min(1.0, kStepFraction * max_step(s, step.ds, z, step.dz));
        x += alpha * step.dx;
        s += alpha * step.ds;
        z += alpha * step.dz;
    }
    result.status = QpStatus::NotConverged;
    return result;
}

}  // namespace apex::planning
