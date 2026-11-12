#pragma once

#include "apex/core/vehicle.hpp"
#include "apex/planning/planning.hpp"

namespace apex::planning {

// The g-g-v envelope of the nominal car, as a point mass: the accelerations its tires, motors
// and brakes allow at each speed. apply_speed_profile integrates along it.
//
// Tires. Each axle carries its static share of the weight plus its share of the downforce
// q v^2 (q = rho ClA / 2, split by aero_balance_front). In steady cornering the axles carry
// lateral force in proportion to their static load (yaw moment balance), so the lateral limit is
// the weaker axle's:
//
//   a_y,max(v) = min over axles a of  f mu_a (g + beta_a q v^2 / (m w_a)),
//
// with f the grip factor, w_a the static weight share and beta_a the downforce share. All four
// wheels drive and brake, so the longitudinal tire limit uses the whole normal load:
//
//   a_x,max(v) = f (mu_f F_zf(v) + mu_r F_zr(v)) / m.
//
// Combined slip follows the friction ellipse (a_x / a_x,max)^2 + (a_y / a_y,max)^2 <= 1.
// Load transfer is ignored; the grip factor absorbs it.
//
// Powertrain. Drive force is limited by max_drive_force and by max_power / v, braking by
// max_brake_force. Aerodynamic drag and rolling resistance (c_rr times the normal load, with
// downforce) oppose motion: they cost acceleration and help braking.
class GgvEnvelope {
public:
    // Throws std::invalid_argument for a non-positive grip factor or speed cap, or a vehicle
    // without mass, wheelbase or grip.
    GgvEnvelope(const VehicleParams& nominal, const SpeedProfileConfig& config);

    // Lateral acceleration limit at speed v (m/s^2).
    [[nodiscard]] double lateral_limit(double v) const;
    // Longitudinal tire limit at speed v with no lateral load (m/s^2).
    [[nodiscard]] double longitudinal_limit(double v) const;
    // Drag plus rolling resistance over mass (m/s^2, >= 0).
    [[nodiscard]] double resistance(double v) const;

    // Net acceleration dv/dt available at speed v on a path of curvature `curvature`: the drive
    // force the tires (after cornering), motors and power allow, minus resistance. May be
    // negative when the tires are saturated laterally.
    [[nodiscard]] double max_acceleration(double v, double curvature) const;
    // Net deceleration -dv/dt available (positive): braking plus resistance.
    [[nodiscard]] double max_deceleration(double v, double curvature) const;

    // The fastest steady speed on curvature `curvature`: the largest v <= speed_cap() at which
    // the tires can hold v^2 |curvature| laterally while still providing the drive force that
    // balances resistance, and the motors can provide it. Without resistance it is the closed
    // form v^2 = f mu g / (|curvature| - f mu beta q / (m w)) of the limiting axle.
    [[nodiscard]] double cornering_speed(double curvature) const;

    [[nodiscard]] double speed_cap() const { return speed_cap_; }

private:
    struct Axle {
        double grip_static;  // f mu g (m/s^2)
        double grip_aero;    // f mu beta q / (m w) (1/m): lateral grip gained per v^2
    };
    // Tire longitudinal acceleration left after cornering at speed v on `curvature`.
    [[nodiscard]] double remaining_longitudinal(double v, double curvature) const;
    [[nodiscard]] double drive_limit(double v) const;  // motors and power, over mass
    [[nodiscard]] bool steady_state_feasible(double v, double curvature) const;

    Axle front_{};
    Axle rear_{};
    double long_static_ = 0.0;  // a_x,max(0)
    double long_aero_ = 0.0;    // d a_x,max / d(v^2)
    double mass_ = 0.0;
    double max_drive_force_ = 0.0;
    double max_power_ = 0.0;
    double max_brake_force_ = 0.0;
    double drag_coefficient_ = 0.0;  // drag over mass per v^2 (1/m)
    double rolling_static_ = 0.0;    // c_rr g
    double rolling_aero_ = 0.0;      // c_rr q / m per v^2 (1/m)
    double speed_cap_ = 0.0;
};

}  // namespace apex::planning
