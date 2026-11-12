#include "apex/planning/ggv.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "apex/core/angles.hpp"

namespace apex::planning {

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
    if (!(m > 0.0) || !(nominal.lf > 0.0) || !(nominal.lr > 0.0)) {
        throw std::invalid_argument("GgvEnvelope: the vehicle needs a mass and a wheelbase");
    }
    if (!(nominal.front_tire.mu > 0.0) || !(nominal.rear_tire.mu > 0.0)) {
        throw std::invalid_argument("GgvEnvelope: the tires need grip");
    }
    if (!(nominal.max_speed > 0.0) || nominal.max_drive_force < 0.0 || nominal.max_power < 0.0 ||
        nominal.max_brake_force < 0.0 || nominal.drag_area < 0.0 ||
        nominal.rolling_resistance < 0.0) {
        throw std::invalid_argument("GgvEnvelope: invalid powertrain or resistance parameters");
    }

    const double q = 0.5 * nominal.air_density * nominal.downforce_area;  // N per (m/s)^2
    const double share_front = nominal.lr / wheelbase;
    const double share_rear = nominal.lf / wheelbase;
    const double aero_front = nominal.aero_balance_front;
    const double aero_rear = 1.0 - aero_front;
    const double mu_f = nominal.front_tire.mu;
    const double mu_r = nominal.rear_tire.mu;

    front_ = {f * mu_f * kGravity, f * mu_f * aero_front * q / (m * share_front)};
    rear_ = {f * mu_r * kGravity, f * mu_r * aero_rear * q / (m * share_rear)};
    long_static_ = f * kGravity * (mu_f * share_front + mu_r * share_rear);
    long_aero_ = f * q * (mu_f * aero_front + mu_r * aero_rear) / m;
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
    return std::min(front_.grip_static + front_.grip_aero * v2,
                    rear_.grip_static + rear_.grip_aero * v2);
}

double GgvEnvelope::longitudinal_limit(double v) const {
    return long_static_ + long_aero_ * v * v;
}

double GgvEnvelope::resistance(double v) const {
    return rolling_static_ + (drag_coefficient_ + rolling_aero_) * v * v;
}

double GgvEnvelope::remaining_longitudinal(double v, double curvature) const {
    const double ratio = v * v * std::abs(curvature) / lateral_limit(v);
    if (ratio >= 1.0) return 0.0;
    return longitudinal_limit(v) * std::sqrt(1.0 - ratio * ratio);
}

double GgvEnvelope::drive_limit(double v) const {
    const double power_force = v > 0.0 ? max_power_ / v : std::numeric_limits<double>::infinity();
    return std::min(max_drive_force_, power_force) / mass_;
}

double GgvEnvelope::max_acceleration(double v, double curvature) const {
    return std::min(drive_limit(v), remaining_longitudinal(v, curvature)) - resistance(v);
}

double GgvEnvelope::max_deceleration(double v, double curvature) const {
    return std::min(max_brake_force_ / mass_, remaining_longitudinal(v, curvature)) + resistance(v);
}

bool GgvEnvelope::steady_state_feasible(double v, double curvature) const {
    const double needed = resistance(v);
    if (drive_limit(v) < needed) return false;
    const double lateral = v * v * std::abs(curvature) / lateral_limit(v);
    const double longitudinal = needed / longitudinal_limit(v);
    return lateral * lateral + longitudinal * longitudinal <= 1.0;
}

double GgvEnvelope::cornering_speed(double curvature) const {
    const double k = std::abs(curvature);
    double hi = speed_cap_;
    // Pure lateral limit of each axle: v^2 k = grip_static + grip_aero v^2. Below grip_aero
    // curvature, downforce grows faster than the demand and the axle never saturates.
    for (const Axle& axle : {front_, rear_}) {
        if (k > axle.grip_aero)
            hi = std::min(hi, std::sqrt(axle.grip_static / (k - axle.grip_aero)));
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
