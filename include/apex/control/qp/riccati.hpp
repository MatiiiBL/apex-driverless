#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include "apex/control/qp/ocp_qp.hpp"

// The Riccati recursion that solves an equality-constrained linear-quadratic (LQ) problem,
//
//   min   sum_k 1/2 x_k' Q_k x_k + u_k' S_k x_k + 1/2 u_k' R_k u_k + q_k' x_k + r_k' u_k
//   s.t.  x_{k+1} = A_k x_k + B_k u_k + b_k,  x_0 given,
//
// in O(N (nx + nu)^3) operations instead of the O(N^3 (nx + nu)^3) of a dense factorization of
// the KKT system. It is the linear-algebra core of the interior-point method (interior_point.hpp)
// that solves the MPC's QPs: each Newton step is one such LQ problem, as in
//   C. V. Rao, S. J. Wright, J. B. Rawlings, "Application of interior-point methods to model
//   predictive control", Journal of Optimization Theory and Applications 99(3), 1998, and
//   G. Frison, M. Diehl, "HPIPM: a high-performance quadratic programming framework for model
//   predictive control", IFAC World Congress, 2020.
//
// The backward pass computes the cost-to-go V_k(x) = 1/2 x' P_k x + p_k' x:
//   R_bar = R + B' P B,  S_bar = S + B' P A,  Q_bar = Q + A' P A          (P = P_{k+1})
//   K = -R_bar^-1 S_bar,  P_k = Q_bar + S_bar' K
//   v = P b + p,  k = -R_bar^-1 (r + B' v),  p_k = q + A' v + S_bar' k   (p = p_{k+1})
// and the forward pass rolls out u_k = K_k x_k + k_k. The multiplier of the dynamics from stage
// k to k+1 is the cost-to-go gradient there, pi_k = P_{k+1} x_{k+1} + p_{k+1}.
//
// The matrix part (factorize) depends only on the Hessians and the dynamics; the vector part
// (solve) on the gradients and offsets. The interior-point method factorizes once per iteration
// and solves twice (Mehrotra's predictor and corrector), so the split halves its cost.
namespace apex::control::qp {

template <int Nx, int Nu>
class RiccatiRecursion {
public:
    using T = Types<Nx, Nu>;
    using StateVector = typename T::StateVector;
    using InputVector = typename T::InputVector;
    using StateMatrix = typename T::StateMatrix;
    using InputMatrix = typename T::InputMatrix;
    using InputStateMatrix = typename T::InputStateMatrix;

    // Sizes the workspace for N + 1 stages. Only factorize() and solve() with the same horizon
    // may follow.
    void resize(std::size_t horizon) {
        const std::size_t stages = horizon + 1;
        P_.resize(stages);
        K_.resize(stages);
        S_bar_.resize(stages);
        chol_.resize(stages);
        k_.resize(stages);
        p_.resize(stages);
    }

    // Factorizes with the dynamics of `problem` and the stage Hessians Q, S, R (which the
    // interior-point method builds from the problem's plus its barrier terms). Returns false if
    // some R_bar is not numerically positive definite, i.e. the problem is not strictly convex
    // in the inputs. `regularization` is added to R_bar's diagonal.
    bool factorize(const OcpQp<Nx, Nu>& problem, const std::vector<StateMatrix>& Q,
                   const std::vector<InputStateMatrix>& S, const std::vector<InputMatrix>& R,
                   double regularization) {
        const std::size_t n = problem.horizon();
        if (!factor_stage(n, R[n], S[n], regularization)) return false;
        P_[n].noalias() = Q[n] + S_bar_[n].transpose() * K_[n];
        for (std::size_t k = n; k-- > 0;) {
            const OcpStage<Nx, Nu>& stage = problem.stages[k];
            const StateMatrix& P = P_[k + 1];
            PA_.noalias() = P * stage.A;
            PB_.noalias() = P * stage.B;
            R_bar_.noalias() = R[k] + stage.B.transpose() * PB_;
            S_bar_[k].noalias() = S[k] + stage.B.transpose() * PA_;
            if (!factor_stage(k, R_bar_, S_bar_[k], regularization)) return false;
            if (k == 0) break;  // x_0 is fixed: P_0 is never used
            StateMatrix& P_k = P_[k];
            P_k.noalias() = Q[k] + stage.A.transpose() * PA_;
            P_k.noalias() += S_bar_[k].transpose() * K_[k];
            P_k = 0.5 * (P_k + P_k.transpose()).eval();
        }
        return true;
    }

    // Solves for the stage gradients q, r and dynamics offsets b (the problem's own b_k are not
    // used, so the interior-point method can pass its residuals), from x0. Fills x (N + 1),
    // u (N + 1) and pi (N), which must already have those sizes.
    void solve(const OcpQp<Nx, Nu>& problem, const std::vector<StateVector>& q,
               const std::vector<InputVector>& r, const std::vector<StateVector>& b,
               const StateVector& x0, std::vector<StateVector>& x, std::vector<InputVector>& u,
               std::vector<StateVector>& pi) {
        const std::size_t n = problem.horizon();
        k_[n] = -chol_[n].solve(r[n]);
        p_[n].noalias() = q[n] + S_bar_[n].transpose() * k_[n];
        for (std::size_t k = n; k-- > 0;) {
            const OcpStage<Nx, Nu>& stage = problem.stages[k];
            v_.noalias() = P_[k + 1] * b[k];
            v_ += p_[k + 1];
            r_bar_.noalias() = r[k] + stage.B.transpose() * v_;
            k_[k] = -chol_[k].solve(r_bar_);
            if (k == 0) break;
            p_[k].noalias() = q[k] + stage.A.transpose() * v_;
            p_[k].noalias() += S_bar_[k].transpose() * k_[k];
        }
        x[0] = x0;
        for (std::size_t k = 0; k < n; ++k) {
            const OcpStage<Nx, Nu>& stage = problem.stages[k];
            u[k].noalias() = K_[k] * x[k];
            u[k] += k_[k];
            x[k + 1].noalias() = stage.A * x[k] + stage.B * u[k];
            x[k + 1] += b[k];
            pi[k].noalias() = P_[k + 1] * x[k + 1];
            pi[k] += p_[k + 1];
        }
        u[n].noalias() = K_[n] * x[n];
        u[n] += k_[n];
    }

private:
    bool factor_stage(std::size_t k, const InputMatrix& R_bar, const InputStateMatrix& S_bar,
                      double regularization) {
        InputMatrix regularized = R_bar;
        regularized.diagonal().array() += regularization;
        chol_[k].compute(regularized);
        if (chol_[k].info() != Eigen::Success) return false;
        S_bar_[k] = S_bar;
        K_[k] = -chol_[k].solve(S_bar);
        return K_[k].allFinite();
    }

    std::vector<StateMatrix> P_;
    std::vector<InputStateMatrix> K_;
    std::vector<InputStateMatrix> S_bar_;
    std::vector<Eigen::LLT<InputMatrix>> chol_;
    std::vector<InputVector> k_;
    std::vector<StateVector> p_;
    // Scratch, kept as members so the hot loop does not touch the heap.
    StateMatrix PA_;
    typename T::StateInputMatrix PB_;
    InputMatrix R_bar_;
    StateVector v_;
    InputVector r_bar_;
};

}  // namespace apex::control::qp
