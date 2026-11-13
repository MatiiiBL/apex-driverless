#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "apex/control/qp/ocp_qp.hpp"
#include "apex/control/qp/riccati.hpp"

// A primal-dual interior-point method for OCP-structured QPs (ocp_qp.hpp), with Mehrotra's
// predictor-corrector and a Riccati recursion (riccati.hpp) for every Newton system:
//   S. Mehrotra, "On the implementation of a primal-dual interior point method", SIAM Journal
//   on Optimization 2(4), 1992;
//   C. V. Rao, S. J. Wright, J. B. Rawlings, "Application of interior-point methods to model
//   predictive control", JOTA 99(3), 1998;
//   G. Frison, M. Diehl, "HPIPM: a high-performance quadratic programming framework for model
//   predictive control", IFAC World Congress, 2020.
//
// Every finite bound becomes a one-sided inequality g_i(z) = sign_i (c' x + d' u) <= h_i with a
// slack t_i >= 0 and a multiplier lambda_i >= 0. With W = diag(lambda / t), a Newton step on the
// perturbed KKT conditions eliminates the slacks and multipliers,
//   dt = -r_p - G dz,   dlambda = (-r_c - lambda dt) / t,
// and leaves an LQ problem in dz with the stage Hessians augmented by G' W G and the gradient by
// G' (lambda r_p - r_c) / t, whose dynamics offsets are the current dynamics residuals: exactly
// the problem the Riccati recursion solves in O(N). The cost per iteration is one factorization
// and two solves (predictor and corrector), so it grows linearly with the horizon, where a
// general sparse solver such as OSQP pays for a sparse LDL' factorization of the whole KKT system
// every time the matrices change, which an LTV MPC does at every step.
//
// Termination: Solved when the stationarity, dynamics and inequality residuals are below
// `tolerance` and every complementarity product lambda_i t_i below
// `complementarity_tolerance` (absolute, infinity norms). Infeasibility is detected from the
// multipliers: on an infeasible problem they diverge along a Farkas certificate (pi, lambda >= 0)
// with E' pi + G' lambda = 0 and pi' b + lambda' h < 0, which the solver checks once they are
// large.
namespace apex::control::qp {

struct InteriorPointSettings {
    int max_iterations = 50;
    double tolerance = 1e-6;
    double complementarity_tolerance = 1e-6;
    // At the iteration limit, residuals within this factor of the tolerances count as
    // SolvedInaccurate (usable) rather than MaxIterations.
    double inaccurate_factor = 100.0;
    double fraction_to_boundary = 0.995;  // keeps slacks and multipliers strictly positive
    double regularization = 1e-10;        // added to the Riccati recursion's R_bar
    // Slacks start at their constraint's distance from the bound but at least this, and
    // multipliers at 1, whether the start is cold or warm (see solve()).
    double initial_slack = 1.0;
    // Multipliers beyond this trigger the infeasibility certificate check.
    double infeasibility_threshold = 1e6;
    double infeasibility_tolerance = 1e-6;
};

template <int Nx, int Nu>
class InteriorPointSolver {
public:
    using T = Types<Nx, Nu>;
    using StateVector = typename T::StateVector;
    using InputVector = typename T::InputVector;
    using StateMatrix = typename T::StateMatrix;
    using InputMatrix = typename T::InputMatrix;
    using InputStateMatrix = typename T::InputStateMatrix;

    explicit InteriorPointSolver(const InteriorPointSettings& settings = {})
        : settings_(settings) {}

    [[nodiscard]] const InteriorPointSettings& settings() const { return settings_; }
    void set_settings(const InteriorPointSettings& settings) { settings_ = settings; }

    // Solves `problem` into `solution` and returns its status (also in solution.status).
    // `warm_start`, if given, seeds the primal iterate with its x and u (it must have the same
    // horizon). Never throws on numerical trouble; allocates only when the problem's size
    // changes.
    //
    // The warm start is primal only: slacks and multipliers start centred, as in a cold start.
    // Interior-point methods are notoriously hard to warm start (E. A. Yildirim, S. J. Wright,
    // "Warm-start strategies in interior-point methods for linear programming", SIAM J.
    // Optimization 12(3), 2002): slacks and multipliers taken from the previous solution sit
    // near its active bounds, and when the active set has moved the iterate is badly centred.
    // On the MPC's QPs, whose late-stage inputs are weakly penalized, the method then crossed a
    // whole input box per iteration and stopped at its iteration limit on problems a cold
    // start solves in 7 iterations; centring costs one or two iterations when the warm start is
    // good.
    QpStatus solve(const OcpQp<Nx, Nu>& problem, OcpSolution<Nx, Nu>& solution,
                   const OcpSolution<Nx, Nu>* warm_start = nullptr) {
        solution.iterations = 0;
        const QpStatus setup_status = setup(problem);
        if (setup_status == QpStatus::InvalidProblem) {
            return finish(problem, solution, setup_status);
        }
        initialize(problem, warm_start);
        if (setup_status == QpStatus::PrimalInfeasible) {
            compute_residuals(problem);
            return finish(problem, solution, setup_status);
        }
        for (int iteration = 0;; ++iteration) {
            const double mu = compute_residuals(problem);
            solution.iterations = iteration;
            if (!std::isfinite(mu) || !std::isfinite(residuals_.stationarity)) {
                return finish(problem, solution, QpStatus::NumericalError);
            }
            if (converged(1.0)) return finish(problem, solution, QpStatus::Solved);
            if (multipliers_norm() > settings_.infeasibility_threshold &&
                infeasibility_certificate(problem)) {
                return finish(problem, solution, QpStatus::PrimalInfeasible);
            }
            if (iteration >= settings_.max_iterations) {
                if (converged(settings_.inaccurate_factor)) {
                    return finish(problem, solution, QpStatus::SolvedInaccurate);
                }
                return finish(problem, solution,
                              infeasibility_certificate(problem) ? QpStatus::PrimalInfeasible
                                                                 : QpStatus::MaxIterations);
            }
            // A failed step leaves the iterate untouched: if it is already close, it is usable.
            if (!newton_step(problem, mu)) {
                return finish(problem, solution,
                              converged(settings_.inaccurate_factor) ? QpStatus::SolvedInaccurate
                                                                     : QpStatus::NumericalError);
            }
        }
    }

private:
    // sign * (c' x_k + d' u_k) <= bound, for row `row` of stage `stage`.
    struct Inequality {
        std::size_t stage = 0;
        std::size_t row = 0;
        double sign = 1.0;
        double bound = 0.0;
    };

    QpStatus setup(const OcpQp<Nx, Nu>& problem) {
        const std::size_t n = problem.horizon();
        if (n == 0 || !problem.x0.allFinite()) return QpStatus::InvalidProblem;
        inequalities_.clear();
        stage_begin_.assign(n + 2, 0);
        bool crossed_bounds = false;
        for (std::size_t k = 0; k <= n; ++k) {
            const OcpStage<Nx, Nu>& stage = problem.stages[k];
            if (!finite_stage(stage)) return QpStatus::InvalidProblem;
            stage_begin_[k] = inequalities_.size();
            for (std::size_t j = 0; j < stage.constraints.size(); ++j) {
                const LinearConstraint<Nx, Nu>& c = stage.constraints[j];
                if (std::isnan(c.lower) || std::isnan(c.upper)) return QpStatus::InvalidProblem;
                crossed_bounds = crossed_bounds || c.lower > c.upper;
                if (c.upper < kInfinity) inequalities_.push_back({k, j, 1.0, c.upper});
                if (c.lower > -kInfinity) inequalities_.push_back({k, j, -1.0, -c.lower});
            }
        }
        stage_begin_[n + 1] = inequalities_.size();
        resize(n);
        return crossed_bounds ? QpStatus::PrimalInfeasible : QpStatus::Solved;
    }

    static bool finite_stage(const OcpStage<Nx, Nu>& s) {
        return s.A.allFinite() && s.B.allFinite() && s.b.allFinite() && s.Q.allFinite() &&
               s.S.allFinite() && s.R.allFinite() && s.q.allFinite() && s.r.allFinite();
    }

    void resize(std::size_t n) {
        const std::size_t stages = n + 1;
        if (x_.size() != stages) {
            for (auto* v : {&x_, &dx_, &rd_x_, &q_hat_}) v->resize(stages);
            for (auto* v : {&u_, &du_, &rd_u_, &r_hat_}) v->resize(stages);
            for (auto* v : {&pi_, &dpi_, &r_eq_}) v->resize(n);
            Q_hat_.resize(stages);
            S_hat_.resize(stages);
            R_hat_.resize(stages);
            riccati_.resize(n);
        }
        const std::size_t m = inequalities_.size();
        for (auto* v :
             {&t_, &lambda_, &g_, &r_p_, &r_c_, &dt_, &dlambda_, &dt_aff_, &dlambda_aff_}) {
            v->resize(m);
        }
    }

    double row_value(const OcpQp<Nx, Nu>& problem, const Inequality& in, const StateVector& x,
                     const InputVector& u) const {
        const LinearConstraint<Nx, Nu>& c = problem.stages[in.stage].constraints[in.row];
        return in.sign * (c.c.dot(x) + c.d.dot(u));
    }

    void initialize(const OcpQp<Nx, Nu>& problem, const OcpSolution<Nx, Nu>* warm) {
        const std::size_t n = problem.horizon();
        const bool use_warm = warm != nullptr && warm->x.size() == n + 1 &&
                              warm->u.size() == n + 1 && all_finite(*warm);
        x_[0] = problem.x0;
        for (std::size_t k = 0; k <= n; ++k) {
            u_[k] = use_warm ? warm->u[k] : InputVector::Zero();
            if (k < n) {
                const OcpStage<Nx, Nu>& stage = problem.stages[k];
                // A cold start rolls the dynamics out, so they hold exactly from the first
                // iterate; a warm start keeps its own states, which are closer to the optimum.
                x_[k + 1] = use_warm ? warm->x[k + 1]
                                     : StateVector(stage.A * x_[k] + stage.B * u_[k] + stage.b);
            }
        }
        for (std::size_t k = 0; k < n; ++k) pi_[k] = StateVector::Zero();
        for (std::size_t i = 0; i < inequalities_.size(); ++i) {
            const Inequality& in = inequalities_[i];
            const double distance = in.bound - row_value(problem, in, x_[in.stage], u_[in.stage]);
            t_[i] = std::max(distance, settings_.initial_slack);
            lambda_[i] = 1.0;
        }
    }

    static bool all_finite(const OcpSolution<Nx, Nu>& s) {
        for (const auto& x : s.x) {
            if (!x.allFinite()) return false;
        }
        for (const auto& u : s.u) {
            if (!u.allFinite()) return false;
        }
        return true;
    }

    // Residuals of the KKT conditions at the current iterate; returns the mean complementarity.
    double compute_residuals(const OcpQp<Nx, Nu>& problem) {
        const std::size_t n = problem.horizon();
        residuals_ = {};
        for (std::size_t k = 0; k <= n; ++k) {
            const OcpStage<Nx, Nu>& stage = problem.stages[k];
            rd_x_[k].noalias() = stage.Q * x_[k] + stage.S.transpose() * u_[k];
            rd_x_[k] += stage.q;
            rd_u_[k].noalias() = stage.R * u_[k] + stage.S * x_[k];
            rd_u_[k] += stage.r;
            if (k < n) {
                rd_x_[k].noalias() += stage.A.transpose() * pi_[k];
                rd_u_[k].noalias() += stage.B.transpose() * pi_[k];
                r_eq_[k].noalias() = stage.A * x_[k] + stage.B * u_[k];
                r_eq_[k] += stage.b - x_[k + 1];
                residuals_.dynamics =
                    std::max(residuals_.dynamics, r_eq_[k].template lpNorm<Eigen::Infinity>());
            }
            if (k > 0) rd_x_[k] -= pi_[k - 1];
            for (std::size_t i = stage_begin_[k]; i < stage_begin_[k + 1]; ++i) {
                const Inequality& in = inequalities_[i];
                const LinearConstraint<Nx, Nu>& c = stage.constraints[in.row];
                g_[i] = row_value(problem, in, x_[k], u_[k]);
                r_p_[i] = g_[i] + t_[i] - in.bound;
                rd_x_[k] += (in.sign * lambda_[i]) * c.c;
                rd_u_[k] += (in.sign * lambda_[i]) * c.d;
            }
            if (k > 0) {
                residuals_.stationarity =
                    std::max(residuals_.stationarity, rd_x_[k].template lpNorm<Eigen::Infinity>());
            }
            residuals_.stationarity =
                std::max(residuals_.stationarity, rd_u_[k].template lpNorm<Eigen::Infinity>());
        }
        double gap = 0.0;
        for (std::size_t i = 0; i < inequalities_.size(); ++i) {
            residuals_.inequality = std::max(residuals_.inequality, std::abs(r_p_[i]));
            const double product = lambda_[i] * t_[i];
            residuals_.complementarity = std::max(residuals_.complementarity, product);
            gap += product;
        }
        return inequalities_.empty() ? 0.0 : gap / static_cast<double>(inequalities_.size());
    }

    bool converged(double factor) const {
        const double tol = factor * settings_.tolerance;
        return residuals_.stationarity <= tol && residuals_.dynamics <= tol &&
               residuals_.inequality <= tol &&
               residuals_.complementarity <= factor * settings_.complementarity_tolerance;
    }

    double multipliers_norm() const {
        double norm = 0.0;
        for (const double l : lambda_) norm = std::max(norm, l);
        for (const auto& p : pi_) norm = std::max(norm, p.template lpNorm<Eigen::Infinity>());
        return norm;
    }

    // Checks whether the current multipliers, normalized, certify infeasibility: for every
    // primal z, sum_k pi_k' (A x_k + B u_k + b_k - x_{k+1}) + sum_i lambda_i (g_i(z) - h_i) is
    // then a positive constant, impossible when the dynamics hold and every g_i <= h_i.
    bool infeasibility_certificate(const OcpQp<Nx, Nu>& problem) const {
        const double scale = multipliers_norm();
        if (!(scale > 0.0) || !std::isfinite(scale)) return false;
        const std::size_t n = problem.horizon();
        double stationarity = 0.0;
        double constant = 0.0;  // the Lagrangian term's value, independent of z
        for (std::size_t k = 0; k <= n; ++k) {
            const OcpStage<Nx, Nu>& stage = problem.stages[k];
            StateVector cx = StateVector::Zero();
            InputVector cu = InputVector::Zero();
            if (k < n) {
                cx.noalias() += stage.A.transpose() * pi_[k];
                cu.noalias() += stage.B.transpose() * pi_[k];
                constant += pi_[k].dot(stage.b);
                if (k == 0) constant += pi_[k].dot(stage.A * problem.x0);
            }
            if (k > 0) cx -= pi_[k - 1];
            for (std::size_t i = stage_begin_[k]; i < stage_begin_[k + 1]; ++i) {
                const Inequality& in = inequalities_[i];
                const LinearConstraint<Nx, Nu>& c = stage.constraints[in.row];
                cx += (in.sign * lambda_[i]) * c.c;
                cu += (in.sign * lambda_[i]) * c.d;
                constant -= lambda_[i] * in.bound;
                if (k == 0) constant += lambda_[i] * in.sign * c.c.dot(problem.x0);
            }
            if (k > 0) stationarity = std::max(stationarity, cx.template lpNorm<Eigen::Infinity>());
            stationarity = std::max(stationarity, cu.template lpNorm<Eigen::Infinity>());
        }
        const double tol = settings_.infeasibility_tolerance;
        return stationarity / scale <= tol && constant / scale > tol;
    }

    // One predictor-corrector step. False on a failed factorization or non-finite direction.
    bool newton_step(const OcpQp<Nx, Nu>& problem, double mu) {
        const std::size_t n = problem.horizon();
        for (std::size_t k = 0; k <= n; ++k) {
            const OcpStage<Nx, Nu>& stage = problem.stages[k];
            Q_hat_[k] = stage.Q;
            S_hat_[k] = stage.S;
            R_hat_[k] = stage.R;
            for (std::size_t i = stage_begin_[k]; i < stage_begin_[k + 1]; ++i) {
                const LinearConstraint<Nx, Nu>& c = stage.constraints[inequalities_[i].row];
                const double w = lambda_[i] / t_[i];
                Q_hat_[k].noalias() += (w * c.c) * c.c.transpose();
                S_hat_[k].noalias() += (w * c.d) * c.c.transpose();
                R_hat_[k].noalias() += (w * c.d) * c.d.transpose();
            }
        }
        if (!riccati_.factorize(problem, Q_hat_, S_hat_, R_hat_, settings_.regularization)) {
            return false;
        }
        const std::size_t m = inequalities_.size();
        // Predictor (affine scaling): aim straight at complementarity zero.
        for (std::size_t i = 0; i < m; ++i) r_c_[i] = lambda_[i] * t_[i];
        solve_direction(problem);
        double alpha = max_step();
        if (m > 0) {
            double mu_affine = 0.0;
            for (std::size_t i = 0; i < m; ++i) {
                mu_affine += (lambda_[i] + alpha * dlambda_[i]) * (t_[i] + alpha * dt_[i]);
                dt_aff_[i] = dt_[i];
                dlambda_aff_[i] = dlambda_[i];
            }
            mu_affine /= static_cast<double>(m);
            const double ratio = mu > 0.0 ? mu_affine / mu : 0.0;
            const double sigma = std::clamp(ratio * ratio * ratio, 0.0, 1.0);
            // The centering target never goes below a tenth of the complementarity tolerance.
            // Products far below it buy nothing, but they inflate the barrier weights
            // lambda / t of the active constraints, and the round-off of the Newton direction
            // grows with those weights until it, not the method, limits the stationarity.
            const double target = std::max(sigma * mu, 0.1 * settings_.complementarity_tolerance);
            // Corrector: the predictor's second-order term and Mehrotra's centering.
            for (std::size_t i = 0; i < m; ++i) {
                r_c_[i] = lambda_[i] * t_[i] + dlambda_aff_[i] * dt_aff_[i] - target;
            }
            solve_direction(problem);
            alpha = std::min(1.0, settings_.fraction_to_boundary * max_step());
        }
        if (!std::isfinite(alpha) || !finite_direction()) return false;
        for (std::size_t k = 0; k <= n; ++k) {
            if (k > 0) x_[k] += alpha * dx_[k];
            u_[k] += alpha * du_[k];
            if (k < n) pi_[k] += alpha * dpi_[k];
        }
        for (std::size_t i = 0; i < m; ++i) {
            t_[i] += alpha * dt_[i];
            lambda_[i] += alpha * dlambda_[i];
        }
        return true;
    }

    bool finite_direction() const {
        for (std::size_t k = 0; k < dx_.size(); ++k) {
            if (!dx_[k].allFinite() || !du_[k].allFinite()) return false;
        }
        for (const auto& p : dpi_) {
            if (!p.allFinite()) return false;
        }
        for (std::size_t i = 0; i < dt_.size(); ++i) {
            if (!std::isfinite(dt_[i]) || !std::isfinite(dlambda_[i])) return false;
        }
        return true;
    }

    // The Newton direction for the current r_c (the factorization is already done).
    void solve_direction(const OcpQp<Nx, Nu>& problem) {
        const std::size_t n = problem.horizon();
        for (std::size_t k = 0; k <= n; ++k) {
            q_hat_[k] = rd_x_[k];
            r_hat_[k] = rd_u_[k];
            const OcpStage<Nx, Nu>& stage = problem.stages[k];
            for (std::size_t i = stage_begin_[k]; i < stage_begin_[k + 1]; ++i) {
                const Inequality& in = inequalities_[i];
                const LinearConstraint<Nx, Nu>& c = stage.constraints[in.row];
                const double scale = in.sign * (lambda_[i] * r_p_[i] - r_c_[i]) / t_[i];
                q_hat_[k] += scale * c.c;
                r_hat_[k] += scale * c.d;
            }
        }
        riccati_.solve(problem, q_hat_, r_hat_, r_eq_, StateVector::Zero(), dx_, du_, dpi_);
        for (std::size_t i = 0; i < inequalities_.size(); ++i) {
            const Inequality& in = inequalities_[i];
            const double dg = row_value(problem, in, dx_[in.stage], du_[in.stage]);
            dt_[i] = -r_p_[i] - dg;
            dlambda_[i] = (-r_c_[i] - lambda_[i] * dt_[i]) / t_[i];
        }
    }

    // The largest step in [0, 1] that keeps every slack and multiplier non-negative.
    double max_step() const {
        double alpha = 1.0;
        for (std::size_t i = 0; i < inequalities_.size(); ++i) {
            if (dt_[i] < 0.0) alpha = std::min(alpha, -t_[i] / dt_[i]);
            if (dlambda_[i] < 0.0) alpha = std::min(alpha, -lambda_[i] / dlambda_[i]);
        }
        return alpha;
    }

    QpStatus finish(const OcpQp<Nx, Nu>& problem, OcpSolution<Nx, Nu>& solution,
                    QpStatus status) const {
        solution.status = status;
        const std::size_t n = problem.horizon();
        if (status == QpStatus::InvalidProblem || x_.size() != n + 1) {
            solution.x.clear();
            solution.u.clear();
            solution.pi.clear();
            solution.y.clear();
            solution.residuals = {};
            solution.objective = 0.0;
            return status;
        }
        solution.x = x_;
        solution.u = u_;
        solution.pi = pi_;
        solution.y.resize(n + 1);
        for (std::size_t k = 0; k <= n; ++k) {
            solution.y[k].assign(problem.stages[k].constraints.size(), 0.0);
            for (std::size_t i = stage_begin_[k]; i < stage_begin_[k + 1]; ++i) {
                solution.y[k][inequalities_[i].row] += inequalities_[i].sign * lambda_[i];
            }
        }
        solution.residuals = residuals_;
        solution.objective = objective(problem, solution);
        return status;
    }

    InteriorPointSettings settings_;
    std::vector<Inequality> inequalities_;
    std::vector<std::size_t> stage_begin_;  // inequalities of stage k: [begin[k], begin[k + 1])
    // Primal-dual iterate and Newton direction.
    std::vector<StateVector> x_, dx_, pi_, dpi_;
    std::vector<InputVector> u_, du_;
    std::vector<double> t_, lambda_, dt_, dlambda_, dt_aff_, dlambda_aff_;
    // Residuals: stationarity per stage, dynamics, inequalities (r_p = g + t - h), and the
    // complementarity target of the current direction.
    std::vector<StateVector> rd_x_, r_eq_, q_hat_;
    std::vector<InputVector> rd_u_, r_hat_;
    std::vector<double> g_, r_p_, r_c_;
    QpResiduals residuals_;
    // Barrier-augmented stage Hessians.
    std::vector<StateMatrix> Q_hat_;
    std::vector<InputStateMatrix> S_hat_;
    std::vector<InputMatrix> R_hat_;
    RiccatiRecursion<Nx, Nu> riccati_;
};

}  // namespace apex::control::qp
