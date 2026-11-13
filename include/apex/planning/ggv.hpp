#pragma once

#include "apex/core/vehicle.hpp"
#include "apex/planning/planning.hpp"

namespace apex::planning {

// The g-g-v envelope of the nominal car, axle by axle: the accelerations its tires, motors and
// brakes allow at each speed, with the forces split between the axles as the car splits them,
// so that no axle is asked for more than its own share of grip.
//
// Forces, in steady state at speed v on curvature kappa with a total longitudinal tire force
// Fx (positive driving):
//   - normal loads N_a = m g w_a + beta_a q v^2 -/+ h Fx / L: static weight share w_a (lr / L
//     front, lf / L rear), downforce q v^2 (q = rho ClA / 2) split by aero_balance_front, and
//     longitudinal load transfer (cg_height h over the wheelbase L; driving loads the rear,
//     braking the front), as in the simulator's vehicle model;
//   - longitudinal force s_a Fx, with s_a = drive_split_front or brake_split_front (and one
//     minus it for the rear);
//   - lateral force Y_a = w_a m v^2 kappa, the split that balances the yaw moment in steady
//     cornering;
//   - each axle's friction ellipse scaled by the grip factor f:
//     (s_a Fx)^2 + Y_a^2 <= (f mu_a N_a)^2.
// Under braking the front gains load and the rear, which still takes 40% of the brake force,
// loses it: at 25 m/s on a straight (grip factor 0.85) the rear limits the tires' braking to
// 17.2 m/s^2 where the whole car's ellipse would allow 20.8, and in a corner, where the rear
// also carries lf / L of the lateral force, to less still. Drive force is limited by
// max_drive_force and max_power / v, braking by max_brake_force; drag and rolling resistance
// (c_rr times the normal load with downforce) oppose motion. Lateral load transfer is ignored;
// the grip factor absorbs it.
class GgvEnvelope {
public:
    // Throws std::invalid_argument for a non-positive grip factor or speed cap, or a vehicle
    // without mass, wheelbase or grip.
    GgvEnvelope(const VehicleParams& nominal, const SpeedProfileConfig& config);

    // Lateral acceleration limit at speed v with no longitudinal force (m/s^2).
    [[nodiscard]] double lateral_limit(double v) const;
    // Drag plus rolling resistance over mass (m/s^2, >= 0).
    [[nodiscard]] double resistance(double v) const;

    // The largest total longitudinal tire force (N) both axles allow at speed v on
    // `curvature`, driving (>= 0) or braking (returned as a magnitude, >= 0), before the
    // powertrain's and brakes' own limits. Zero if the tires are already saturated laterally.
    [[nodiscard]] double tire_drive_force(double v, double curvature) const;
    [[nodiscard]] double tire_brake_force(double v, double curvature) const;

    // Net acceleration dv/dt available at speed v on a path of curvature `curvature`: the drive
    // force the tires (after cornering), motors and power allow, minus resistance. May be
    // negative when the tires are saturated laterally.
    [[nodiscard]] double max_acceleration(double v, double curvature) const;
    // Net deceleration -dv/dt available (positive): braking plus resistance.
    [[nodiscard]] double max_deceleration(double v, double curvature) const;

    // The fastest steady speed on curvature `curvature`: the largest v <= speed_cap() at which
    // both axles can hold their share of v^2 kappa while still providing the drive force that
    // balances resistance, and the motors can provide it. Without resistance it is the closed
    // form v^2 = f mu g / (|curvature| - f mu beta q / (m w)) of the limiting axle.
    [[nodiscard]] double cornering_speed(double curvature) const;

    [[nodiscard]] double speed_cap() const { return speed_cap_; }

private:
    struct Axle {
        double grip;         // f mu
        double weight;       // static normal load m g w (N)
        double aero;         // downforce share beta q (N per (m/s)^2)
        double yaw_share;    // w: share of the lateral force in steady cornering
        double drive_share;  // s under drive
        double brake_share;  // s under braking
        double transfer;     // dN / dFx (1): -h / L front, +h / L rear
    };
    // The largest |Fx| this axle allows, driving or braking, at speed v on `curvature`.
    [[nodiscard]] double axle_force(const Axle& axle, double v, double curvature,
                                    bool braking) const;
    [[nodiscard]] double drive_limit(double v) const;  // motors and power (N)
    [[nodiscard]] bool steady_state_feasible(double v, double curvature) const;

    Axle front_{};
    Axle rear_{};
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
