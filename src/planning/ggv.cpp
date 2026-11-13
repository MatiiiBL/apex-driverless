#include "apex/planning/ggv.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "apex/core/angles.hpp"

namespace apex::planning {

namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

bool share(double x) {
    return x >= 0.0 && x <= 1.0;
}

}  // namespace

GgvEnvelope::GgvEnvelope(const VehicleParams& nominal, const SpeedProfileConfig& config) {
    const double f = config.grip_factor;
    if (!(f > 0.0) || !std::isfinite(f)) {
        throw std::invalid_argument("SpeedProfileConfig: grip_factor must be positive");
    }
    if (!(config.max_speed > 0.0)) {
        throw std::invalid_argument("SpeedProfileConfig: max_speed must be positive");
    }
    const double m = nominal.mass;
    const double wheelbase = nominal.wheelbase();
    if (!(m > 0.0) || !(nominal.lf > 0.0) || !(nominal.lr > 0.0) || !(nominal.cg_height >= 0.0)) {
        throw std::invalid_argument("GgvEnvelope: the vehicle needs a mass and a wheelbase");
    }
    if (!(nominal.front_tire.mu > 0.0) || !(nominal.rear_tire.mu > 0.0)) {
        throw std::invalid_argument("GgvEnvelope: the tires need grip");
    }
    if (!(nominal.max_speed > 0.0) || nominal.max_drive_force < 0.0 || nominal.max_power < 0.0 ||
        nominal.max_brake_force < 0.0 || nominal.drag_area < 0.0 ||
        nominal.rolling_resistance < 0.0 || !share(nominal.aero_balance_front) ||
        !share(nominal.drive_split_front) || !share(nominal.brake_split_front)) {
        throw std::invalid_argument(
            "GgvEnvelope: invalid powertrain, aero or resistance parameters");
    }

    const double q = 0.5 * nominal.air_density * nominal.downforce_area;  // N per (m/s)^2
    const double transfer = nominal.cg_height / wheelbase;
    const double share_front = nominal.lr / wheelbase;
    const double share_rear = nominal.lf / wheelbase;
    front_ = {.grip = f * nominal.front_tire.mu,
              .weight = m * kGravity * share_front,
              .aero = nominal.aero_balance_front * q,
              .yaw_share = share_front,
              .drive_share = nominal.drive_split_front,
              .brake_share = nominal.brake_split_front,
              .transfer = -transfer};
    rear_ = {.grip = f * nominal.rear_tire.mu,
             .weight = m * kGravity * share_rear,
             .aero = (1.0 - nominal.aero_balance_front) * q,
             .yaw_share = share_rear,
             .drive_share = 1.0 - nominal.drive_split_front,
             .brake_share = 1.0 - nominal.brake_split_front,
             .transfer = transfer};
    mass_ = m;
    max_drive_force_ = nominal.max_drive_force;
    max_power_ = nominal.max_power;
    max_brake_force_ = nominal.max_brake_force;
    drag_coefficient_ = 0.5 * nominal.air_density * nominal.drag_area / m;
    rolling_static_ = nominal.rolling_resistance * kGravity;
    rolling_aero_ = nominal.rolling_resistance * q / m;
    speed_cap_ = std::min(nominal.max_speed, config.max_speed);
}

double GgvEnvelope::lateral_limit(double v) const {
    const double v2 = v * v;
    double limit = kInfinity;
    for (const Axle& axle : {front_, rear_}) {
        limit =
            std::min(limit, axle.grip * (axle.weight + axle.aero * v2) / (axle.yaw_share * mass_));
    }
    return limit;
}

double GgvEnvelope::resistance(double v) const {
    return rolling_static_ + (drag_coefficient_ + rolling_aero_) * v * v;
}

// With F the total force's magnitude, the axle takes s F of it and Y sideways on a normal load
// N0 + t F (t its load transfer in this direction), so its ellipse reads
//   s^2 F^2 + Y^2 <= k^2 (N0 + t F)^2,   that is   a F^2 + b F + c >= 0 with
//   a = k^2 t^2 - s^2,  b = 2 k^2 N0 t,  c = k^2 N0^2 - Y^2.
// It holds at F = 0 when c >= 0. The largest F that keeps it is the root 2c / (sqrt(D) - b),
// D = b^2 - 4ac, the form that stays accurate as a goes to zero: for a < 0 it is the positive
// root, for a > 0 and b < 0 (an axle losing load) the smaller one; for a >= 0 and b >= 0 the
// axle gains load faster than it uses grip and F is unbounded. Returns -1 when the axle is
// saturated laterally already.
double GgvEnvelope::axle_force(const Axle& axle, double v, double curvature, bool braking) const {
    const double v2 = v * v;
    const double lateral = axle.yaw_share * mass_ * v2 * std::abs(curvature);
    const double n0 = axle.weight + axle.aero * v2;
    const double s = braking ? axle.brake_share : axle.drive_share;
    const double t = braking ? -axle.transfer : axle.transfer;
    const double k = axle.grip;
    const double c = k * k * n0 * n0 - lateral * lateral;
    if (c < 0.0) return -1.0;
    const double a = k * k * t * t - s * s;
    const double b = 2.0 * k * k * n0 * t;
    const double denominator = std::sqrt(std::max(0.0, b * b - 4.0 * a * c)) - b;
    double limit = denominator > 0.0 ? 2.0 * c / denominator : kInfinity;
    if (t < 0.0) limit = std::min(limit, -n0 / t);  // the normal load stays positive
    return limit;
}

double GgvEnvelope::tire_drive_force(double v, double curvature) const {
    return std::max(0.0, std::min(axle_force(front_, v, curvature, false),
                                  axle_force(rear_, v, curvature, false)));
}

double GgvEnvelope::tire_brake_force(double v, double curvature) const {
    return std::max(0.0, std::min(axle_force(front_, v, curvature, true),
                                  axle_force(rear_, v, curvature, true)));
}

double GgvEnvelope::drive_limit(double v) const {
    const double power_force = v > 0.0 ? max_power_ / v : kInfinity;
    return std::min(max_drive_force_, power_force);
}

double GgvEnvelope::max_acceleration(double v, double curvature) const {
    return std::min(drive_limit(v), tire_drive_force(v, curvature)) / mass_ - resistance(v);
}

double GgvEnvelope::max_deceleration(double v, double curvature) const {
    return std::min(max_brake_force_, tire_brake_force(v, curvature)) / mass_ + resistance(v);
}

bool GgvEnvelope::steady_state_feasible(double v, double curvature) const {
    const double needed = mass_ * resistance(v);
    return drive_limit(v) >= needed && axle_force(front_, v, curvature, false) >= needed &&
           axle_force(rear_, v, curvature, false) >= needed;
}

double GgvEnvelope::cornering_speed(double curvature) const {
    const double k = std::abs(curvature);
    double hi = speed_cap_;
    // Pure lateral limit of each axle: w m v^2 k = f mu (m g w + beta q v^2). Below this
    // curvature, downforce grows faster than the demand and the axle never saturates.
    for (const Axle& axle : {front_, rear_}) {
        const double demand = axle.yaw_share * mass_ * k - axle.grip * axle.aero;
        if (demand > 0.0) hi = std::min(hi, std::sqrt(axle.grip * axle.weight / demand));
    }
    if (steady_state_feasible(hi, curvature)) return hi;
    // Feasibility is monotone in v (lateral and resistance demands grow faster than the grip,
    // drive force shrinks), so bisect for the boundary.
    double lo = 0.0;
    if (!steady_state_feasible(lo, curvature)) return 0.0;
    for (int i = 0; i < 100 && hi - lo > 1e-12 * hi; ++i) {
        const double mid = 0.5 * (lo + hi);
        (steady_state_feasible(mid, curvature) ? lo : hi) = mid;
    }
    return lo;
}

}  // namespace apex::planning
