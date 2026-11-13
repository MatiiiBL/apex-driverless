#pragma once

#include <cstddef>

#include <Eigen/Core>
#include <Eigen/SVD>

#include "apex/control/qp/ocp_qp.hpp"
#include "apex/core/rng.hpp"

// Random OCP-QPs with the structure and conditioning of an MPC problem, feasible by
// construction: a random input sequence is rolled out through the dynamics and every bound is
// placed around that trajectory, at random distances, so some bind at the optimum and some do
// not.
namespace apex::control::testing {

template <int Rows, int Cols>
Eigen::Matrix<double, Rows, Cols> random_matrix(Rng& rng, double stddev) {
    Eigen::Matrix<double, Rows, Cols> m;
    for (Eigen::Index i = 0; i < Rows; ++i) {
        for (Eigen::Index j = 0; j < Cols; ++j) m(i, j) = rng.normal(0.0, stddev);
    }
    return m;
}

struct RandomQpOptions {
    std::size_t horizon = 20;
    std::size_t state_rows = 2;  // random polytopic rows on (x_k, u_k) per stage k >= 1
    double one_sided_probability = 0.3;
    double min_margin = 0.05;  // distance of each bound from the feasible trajectory
    double max_margin = 1.0;
};

template <int Nx, int Nu>
qp::OcpQp<Nx, Nu> random_ocp_qp(Rng& rng, const RandomQpOptions& options) {
    using T = qp::Types<Nx, Nu>;
    qp::OcpQp<Nx, Nu> problem;
    problem.stages.resize(options.horizon + 1);
    problem.x0 = random_matrix<Nx, 1>(rng, 1.0);
    typename T::StateVector x = problem.x0;
    auto margin = [&] { return rng.uniform(options.min_margin, options.max_margin); };
    auto bound_around = [&](double value, qp::LinearConstraint<Nx, Nu>& row) {
        row.lower = value - margin();
        row.upper = value + margin();
        if (rng.bernoulli(options.one_sided_probability)) {
            if (rng.bernoulli(0.5)) {
                row.lower = -qp::kInfinity;
            } else {
                row.upper = qp::kInfinity;
            }
        }
    };
    for (std::size_t k = 0; k <= options.horizon; ++k) {
        qp::OcpStage<Nx, Nu>& stage = problem.stages[k];
        // A discretized system like an MPC's: identity plus a random perturbation, scaled so its
        // largest singular value stays at most 1.02 and long horizons stay well conditioned.
        using StateMatrix = typename T::StateMatrix;
        stage.A = StateMatrix::Identity() + random_matrix<Nx, Nx>(rng, 0.2);
        const double norm = Eigen::JacobiSVD<StateMatrix>(stage.A).singularValues()(0);
        if (norm > 1.02) stage.A *= 1.02 / norm;
        stage.B = random_matrix<Nx, Nu>(rng, 0.5);
        stage.b = random_matrix<Nx, 1>(rng, 0.05);
        // [Q S'; S R] = M M' + 0.1 I: positive definite, with cross terms.
        const Eigen::Matrix<double, Nx + Nu, Nx + Nu> m = random_matrix<Nx + Nu, Nx + Nu>(rng, 0.5);
        Eigen::Matrix<double, Nx + Nu, Nx + Nu> h = m * m.transpose();
        h.diagonal().array() += 0.1;
        stage.Q = h.template topLeftCorner<Nx, Nx>();
        stage.S = h.template bottomLeftCorner<Nu, Nx>();
        stage.R = h.template bottomRightCorner<Nu, Nu>();
        stage.q = random_matrix<Nx, 1>(rng, 1.0);
        stage.r = random_matrix<Nu, 1>(rng, 1.0);

        typename T::InputVector u;
        for (Eigen::Index i = 0; i < Nu; ++i) u(i) = rng.uniform(-0.5, 0.5);
        for (Eigen::Index i = 0; i < Nu; ++i) {
            qp::LinearConstraint<Nx, Nu> row;
            row.d(i) = 1.0;
            bound_around(u(i), row);
            stage.constraints.push_back(row);
        }
        if (k > 0) {
            for (std::size_t j = 0; j < options.state_rows; ++j) {
                qp::LinearConstraint<Nx, Nu> row;
                row.c = random_matrix<Nx, 1>(rng, 1.0);
                if (rng.bernoulli(0.5)) row.d = random_matrix<Nu, 1>(rng, 0.5);
                bound_around(row.c.dot(x) + row.d.dot(u), row);
                stage.constraints.push_back(row);
            }
        }
        x = stage.A * x + stage.B * u + stage.b;
    }
    return problem;
}

}  // namespace apex::control::testing
