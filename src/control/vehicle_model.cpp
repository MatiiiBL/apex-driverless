#include "apex/control/vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace apex::control {

NominalModel::NominalModel(const VehicleParams& params, double kinematic_below,
                           double dynamic_above)
    : params_(params), kinematic_below_(kinematic_below), dynamic_above_(dynamic_above) {
    if (!(params.mass > 0.0) || !(params.yaw_inertia > 0.0) || !(params.lf > 0.0) ||
        !(params.lr > 0.0)) {
        throw std::invalid_argument("NominalModel: mass, inertia and axle distances must be > 0");
    }
    if (!(kinematic_below > 0.0) || !(dynamic_above > kinematic_below)) {
        throw std::invalid_argument("NominalModel: need 0 < kinematic_below < dynamic_above");
    }
    static_front_ = params.mass * kGravity * params.lr / params.wheelbase();
    static_rear_ = params.mass * kGravity * params.lf / params.wheelbase();
}

double NominalModel::max_drive_accel(double speed) const {
    const double force =
        std::min(params_.max_drive_force, params_.max_power / std::max(speed, 1.0));
    return force / params_.mass;
}

double NominalModel::max_brake_decel() const {
    return params_.max_brake_force / params_.mass;
}

namespace {

// The root in [lo, hi] of an increasing function, by bisection to double precision.
template <class F>
double bisect(F&& increasing, double target, double lo, double hi) {
    for (int i = 0; i < 64; ++i) {
        const double mid = 0.5 * (lo + hi);
        if (increasing(mid) < target) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return 0.5 * (lo + hi);
}

}  // namespace

double NominalModel::inverse_lateral_force(double fy, double fz, const TireParams& tire) const {
    const double peak = tire.mu * std::max(fz, 0.0);
    if (!(peak > 0.0) || fy == 0.0) return 0.0;
    auto shape = [&](double alpha) {  // the magic formula's inner argument, increasing
        const double b_alpha = tire.B * alpha;
        return b_alpha - tire.E * (b_alpha - std::atan(b_alpha));
    };
    // The force peaks where C atan(shape) reaches pi/2 and falls beyond, so the search stays
    // below the peak slip (or below 0.5 rad for a shape factor that never peaks).
    constexpr double kMaxSlip = 0.5;
    const double peak_slip =
        tire.C > 1.0 ? bisect(shape, std::tan(0.5 * kPi / tire.C), 0.0, kMaxSlip) : kMaxSlip;
    auto magnitude = [&](double alpha) {
        return peak * std::sin(tire.C * std::atan(shape(alpha)));
    };
    const double wanted = std::abs(fy);
    const double alpha =
        wanted < magnitude(peak_slip) ? bisect(magnitude, wanted, 0.0, peak_slip) : peak_slip;
    return fy > 0.0 ? -alpha : alpha;
}

SlipAngles NominalModel::steady_state_slip(double curvature, double speed) const {
    const VehicleParams& p = params_;
    const double centripetal = p.mass * speed * speed * curvature;  // N, positive to the left
    const double downforce = p.downforce(speed);
    const double wheelbase = p.wheelbase();
    SlipAngles slip;
    slip.front =
        inverse_lateral_force(centripetal * p.lr / wheelbase,
                              static_front_ + p.aero_balance_front * downforce, p.front_tire);
    slip.rear =
        inverse_lateral_force(centripetal * p.lf / wheelbase,
                              static_rear_ + (1.0 - p.aero_balance_front) * downforce, p.rear_tire);
    return slip;
}

}  // namespace apex::control
