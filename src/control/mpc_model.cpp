#include "apex/control/mpc_model.hpp"

#include "apex/control/matrix_exponential.hpp"

namespace apex::control::mpc {

DiscreteModel discretize(const NominalModel& model, const State& x, const Input& u,
                         double curvature, double dt) {
    // Independent variables: the six states, then steering rate and acceleration.
    constexpr int kVariables = kStates + 2;
    using D = Dual<kVariables>;
    StateArray<D> xs;
    for (int i = 0; i < kStates; ++i) xs[static_cast<std::size_t>(i)] = D::variable(x(i), i);
    const D steer_rate = D::variable(u(kSteerRate), kStates);
    const D accel = D::variable(u(kAccel), kStates + 1);
    const StateArray<D> f = error_dynamics(model, xs, steer_rate, accel, curvature);

    Eigen::Matrix<double, kVariables, 1> point;
    point << x, u(kSteerRate), u(kAccel);
    // d/dt [x; u; 1] = M [x; u; 1], with the affine term in the last column.
    Eigen::Matrix<double, kVariables + 1, kVariables + 1> m;
    m.setZero();
    for (int i = 0; i < kStates; ++i) {
        const D& fi = f[static_cast<std::size_t>(i)];
        m.block<1, kVariables>(i, 0) = dt * fi.grad.transpose();
        m(i, kVariables) = dt * (fi.value - fi.grad.dot(point));
    }
    const Eigen::Matrix<double, kVariables + 1, kVariables + 1> e = matrix_exponential(m);
    DiscreteModel out;
    out.A = e.topLeftCorner<kStates, kStates>();
    out.B.setZero();
    out.B.leftCols<2>() = e.block<kStates, 2>(0, kStates);
    out.c = e.block<kStates, 1>(0, kVariables);
    return out;
}

State simulate(const NominalModel& model, const State& x, const Input& u, double curvature,
               double dt, int substeps) {
    auto derivative = [&](const State& s) {
        StateArray<double> a;
        for (int i = 0; i < kStates; ++i) a[static_cast<std::size_t>(i)] = s(i);
        const StateArray<double> f = error_dynamics(model, a, u(kSteerRate), u(kAccel), curvature);
        return State(Eigen::Map<const State>(f.data()));
    };
    const double h = dt / static_cast<double>(substeps);
    State s = x;
    for (int i = 0; i < substeps; ++i) {
        const State k1 = derivative(s);
        const State k2 = derivative(s + 0.5 * h * k1);
        const State k3 = derivative(s + 0.5 * h * k2);
        const State k4 = derivative(s + h * k3);
        s += (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
    return s;
}

}  // namespace apex::control::mpc
