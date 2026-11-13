#include "apex/estimation/lateral_model.hpp"

#include <algorithm>
#include <cmath>

#include "apex/core/angles.hpp"

namespace apex::estimation {

namespace {

// The magic formula's inner argument phi(alpha) = B alpha - E (B alpha - atan(B alpha)), which
// is increasing in alpha for E < 1.
double inner(const TireParams& tire, double alpha) {
    const double b_alpha = tire.B * alpha;
    return b_alpha - tire.E * (b_alpha - std::atan(b_alpha));
}

// Smallest alpha >= 0 with inner(alpha) = target, by bisection: the curvature factor E makes the
// inverse non-algebraic, and 60 halvings of a bracket below 1 rad reach double precision.
double invert_inner(const TireParams& tire, double target) {
    double lo = 0.0;
    double hi = 1.0;
    while (inner(tire, hi) < target && hi < 1e3) hi *= 2.0;
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        (inner(tire, mid) < target ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

}  // namespace

AxleLoads axle_normal_loads(const VehicleParams& vehicle, double speed, double ax) {
    const double wheelbase = vehicle.wheelbase();
    const double weight = vehicle.mass * kGravity;
    const double downforce = vehicle.downforce(speed);
    const double transfer = vehicle.mass * ax * vehicle.cg_height / wheelbase;
    AxleLoads loads;
    loads.front =
        weight * vehicle.lr / wheelbase + downforce * vehicle.aero_balance_front - transfer;
    loads.rear =
        weight * vehicle.lf / wheelbase + downforce * (1.0 - vehicle.aero_balance_front) + transfer;
    // A wheel cannot pull the ground: keep a small positive load for the inverse.
    const double floor = 0.05 * weight;
    loads.front = std::max(loads.front, floor);
    loads.rear = std::max(loads.rear, floor);
    return loads;
}

SlipAngle slip_angle_for_force(const TireParams& tire, double fz, double fy) {
    SlipAngle result;
    const double capacity = tire.mu * fz;
    if (!(capacity > 0.0) || fy == 0.0) return result;
    result.utilization = std::abs(fy) / capacity;
    // sin(C atan(phi)) reaches 1 at C atan(phi) = pi / 2 when C > 1; with C <= 1 the curve only
    // tends to sin(C pi / 2), its supremum.
    const double peak_argument = std::min(0.5 * kPi, tire.C * 0.5 * kPi * (1.0 - 1e-9));
    const double peak_force_ratio = std::sin(peak_argument);
    double argument = peak_argument;
    if (result.utilization < peak_force_ratio) {
        argument = std::asin(result.utilization);
    } else {
        result.utilization = std::max(result.utilization, 1.0);
    }
    const double magnitude = invert_inner(tire, std::tan(argument / tire.C));
    // Fy = -mu Fz sin(...): a positive force needs a negative slip angle.
    result.alpha = fy > 0.0 ? -magnitude : magnitude;
    return result;
}

AxleSlip predict_axle_slip(const VehicleParams& vehicle, double speed, double ax, double ay,
                           double yaw_accel) {
    const double wheelbase = vehicle.wheelbase();
    const double force_front =
        (vehicle.mass * ay * vehicle.lr + vehicle.yaw_inertia * yaw_accel) / wheelbase;
    const double force_rear =
        (vehicle.mass * ay * vehicle.lf - vehicle.yaw_inertia * yaw_accel) / wheelbase;
    const AxleLoads loads = axle_normal_loads(vehicle, speed, ax);
    return {slip_angle_for_force(vehicle.front_tire, loads.front, force_front),
            slip_angle_for_force(vehicle.rear_tire, loads.rear, force_rear)};
}

}  // namespace apex::estimation
