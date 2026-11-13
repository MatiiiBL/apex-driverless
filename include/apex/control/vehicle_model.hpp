#pragma once

#include <algorithm>
#include <cmath>

#include "apex/control/dual.hpp"
#include "apex/core/angles.hpp"
#include "apex/core/vehicle.hpp"

// The controllers' model of the car: the dynamic single-track (bicycle) model of
// docs/design.md, section 5.1, with the *nominal* VehicleParams the car's software is given
// (which experiments may perturb away from the simulator's truth). Pure pursuit and Stanley use
// its steady-state slip angles and resistance; the MPC linearizes its equations of motion.
//
// Sign conventions: body frame x forward, y left. Slip angles alpha_f = atan2(vy + lf r, vx) -
// delta and alpha_r = atan2(vy - lr r, vx); an axle's lateral force opposes its slip, so a left
// turn has negative slip angles.
namespace apex::control {

template <class T>
struct BodyRates {
    T vx_dot;     // m/s^2, body frame
    T vy_dot;     // m/s^2
    T yaw_accel;  // rad/s^2
};

struct SlipAngles {
    double front = 0.0;  // rad
    double rear = 0.0;
};

class NominalModel {
public:
    // Below kinematic_below the model is the kinematic bicycle, above dynamic_above the dynamic
    // one, linearly blended in between (as in J. Kabzan et al., "AMZ Driverless: The full
    // autonomous racing system", Journal of Field Robotics 37(7), 2020).
    explicit NominalModel(const VehicleParams& params, double kinematic_below = 2.5,
                          double dynamic_above = 4.0);

    [[nodiscard]] const VehicleParams& params() const { return params_; }

    // Drag plus rolling resistance on weight and downforce, as a deceleration (m/s^2).
    template <class T>
    [[nodiscard]] T resistance(const T& vx) const;

    // The largest drive acceleration at speed v (power and drive-force limits) and the largest
    // braking deceleration, both as total tire force over mass (m/s^2, positive).
    [[nodiscard]] double max_drive_accel(double speed) const;
    [[nodiscard]] double max_brake_decel() const;

    // Slip angles of steady cornering on curvature `curvature` (1/m, positive left) at
    // `speed`, where each axle carries its share m v^2 kappa of the centripetal force. Beyond an
    // axle's grip the slip is its peak slip angle.
    [[nodiscard]] SlipAngles steady_state_slip(double curvature, double speed) const;

    // Body-frame accelerations for velocities (vx, vy, r), front wheel angle delta, its rate
    // and the commanded acceleration (total longitudinal tire force over mass). The kinematic
    // part needs delta_rate: there the lateral velocity and yaw rate follow the steering.
    template <class T>
    [[nodiscard]] BodyRates<T> body_rates(const T& vx, const T& vy, const T& r, const T& delta,
                                          const T& delta_rate, const T& accel) const;

private:
    // Pacejka lateral force of an axle with normal load fz carrying longitudinal force fx: the
    // friction ellipse leaves sqrt((mu fz)^2 - fx^2) for the lateral force, which scales the
    // magic formula's peak.
    template <class T>
    [[nodiscard]] T lateral_force(const T& alpha, const T& fz, const T& fx,
                                  const TireParams& tire) const;
    // The slip angle (negative for a positive force) at which an axle with normal load fz makes
    // lateral force fy, by bisection on the rising part of the magic formula.
    [[nodiscard]] double inverse_lateral_force(double fy, double fz, const TireParams& tire) const;

    VehicleParams params_;
    double kinematic_below_;
    double dynamic_above_;
    double static_front_;  // N
    double static_rear_;   // N
};

template <class T>
T NominalModel::resistance(const T& vx) const {
    using std::tanh;
    const double half_rho = 0.5 * params_.air_density;
    const T v2 = vx * vx;
    // tanh: rolling resistance fades out at standstill instead of pushing the car backwards.
    const T rolling = params_.rolling_resistance *
                      (params_.mass * kGravity + half_rho * params_.downforce_area * v2) *
                      tanh(vx / 0.5);
    return (half_rho * params_.drag_area * v2 + rolling) / params_.mass;
}

template <class T>
T NominalModel::lateral_force(const T& alpha, const T& fz, const T& fx,
                              const TireParams& tire) const {
    using std::atan;
    using std::sin;
    using std::sqrt;
    const T cap = tire.mu * fz;
    const T available2 = cap * cap - fx * fx;
    // At least a tenth of the grip stays available, which keeps the model smooth and its
    // linearization informative when a command asks for more longitudinal force than the axle
    // has.
    const T peak =
        value_of(available2) > 0.01 * value_of(cap) * value_of(cap) ? sqrt(available2) : 0.1 * cap;
    const T b_alpha = tire.B * alpha;
    return -peak * sin(tire.C * atan(b_alpha - tire.E * (b_alpha - atan(b_alpha))));
}

template <class T>
BodyRates<T> NominalModel::body_rates(const T& vx, const T& vy, const T& r, const T& delta,
                                      const T& delta_rate, const T& accel) const {
    using std::atan2;
    using std::cos;
    using std::sin;
    using std::tan;
    const VehicleParams& p = params_;
    const double wheelbase = p.wheelbase();
    const T drag = resistance(vx);

    // Kinematic bicycle: vy = lr r and r = vx tan(delta) / L hold at all times, so both
    // derivatives follow d/dt (vx tan delta).
    const T vx_dot_kinematic = accel - drag;
    const T tan_delta = tan(delta);
    const T turning =
        vx_dot_kinematic * tan_delta + vx * (1.0 + tan_delta * tan_delta) * delta_rate;
    BodyRates<T> kinematic{vx_dot_kinematic, turning * (p.lr / wheelbase), turning / wheelbase};

    const double speed = value_of(vx);
    if (speed <= kinematic_below_) return kinematic;

    const double split = value_of(accel) >= 0.0 ? p.drive_split_front : p.brake_split_front;
    const T fx_total = p.mass * accel;
    const T fx_front = split * fx_total;
    const T fx_rear = (1.0 - split) * fx_total;
    const T downforce = 0.5 * p.air_density * p.downforce_area * vx * vx;
    const T transfer = fx_total * (p.cg_height / wheelbase);
    const T fz_front = static_front_ + p.aero_balance_front * downforce - transfer;
    const T fz_rear = static_rear_ + (1.0 - p.aero_balance_front) * downforce + transfer;
    const T alpha_front = atan2(vy + p.lf * r, vx) - delta;
    const T alpha_rear = atan2(vy - p.lr * r, vx);
    const T fy_front = lateral_force(alpha_front, fz_front, fx_front, p.front_tire);
    const T fy_rear = lateral_force(alpha_rear, fz_rear, fx_rear, p.rear_tire);
    const T cos_delta = cos(delta);
    const T sin_delta = sin(delta);
    // Front tire forces rotated from the wheel frame into the body frame.
    const T front_x = fx_front * cos_delta - fy_front * sin_delta;
    const T front_y = fy_front * cos_delta + fx_front * sin_delta;
    BodyRates<T> dynamic{(front_x + fx_rear) / p.mass - drag + vy * r,
                         (front_y + fy_rear) / p.mass - vx * r,
                         (p.lf * front_y - p.lr * fy_rear) / p.yaw_inertia};
    if (speed >= dynamic_above_) return dynamic;

    const T w = (vx - kinematic_below_) / (dynamic_above_ - kinematic_below_);
    return {w * dynamic.vx_dot + (1.0 - w) * kinematic.vx_dot,
            w * dynamic.vy_dot + (1.0 - w) * kinematic.vy_dot,
            w * dynamic.yaw_accel + (1.0 - w) * kinematic.yaw_accel};
}

}  // namespace apex::control
