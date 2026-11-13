#pragma once

#include <cstddef>
#include <deque>

#include <Eigen/Core>

#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

// The simulated car's dynamics (docs/sim.md, "Vehicle model"; docs/design.md, section 5.1).
namespace apex::sim {

// Throws std::invalid_argument naming the first parameter that is non-finite or out of its
// physical range (masses, lengths and time constants positive, shares in [0, 1], ...).
void validate_vehicle_params(const VehicleParams& params);

// What the tires do at one instant: per-axle forces in each axle's wheel frame (x along the
// wheel's heading, y to its left), normal loads and slip angles.
struct TireForces {
    double fx_front = 0.0;  // longitudinal (N), positive driving
    double fx_rear = 0.0;
    double fy_front = 0.0;  // lateral (N), positive to the left
    double fy_rear = 0.0;
    double fz_front = 0.0;  // normal load (N)
    double fz_rear = 0.0;
    double slip_front = 0.0;  // slip angle (rad)
    double slip_rear = 0.0;
    double blend =
        1.0;  // weight of the dynamic model: 0 below kKinematicSpeed, 1 above kDynamicSpeed
};

// A dynamic single-track (bicycle) model with Pacejka tires, blended into the kinematic
// single-track model at walking pace, with actuator dead time, lag and limits.
//
// Dynamic model (Rajamani, "Vehicle Dynamics and Control", Springer, 2nd ed. 2012, ch. 2-3),
// body frame at the CoG:
//   m (vx' - vy r) = Fxf cos d - Fyf sin d + Fxr - F_drag - F_roll
//   m (vy' + vx r) = Fxf sin d + Fyf cos d + Fyr
//   Iz r'          = lf (Fxf sin d + Fyf cos d) - lr Fyr
// Lateral forces from the simplified magic formula of TireParams (Pacejka, "Tire and Vehicle
// Dynamics", Butterworth-Heinemann, 3rd ed. 2012) at slip angles
//   af = atan2(vy + lf r, vx) - d,  ar = atan2(vy - lr r, vx),
// scaled by the friction ellipse sqrt(1 - (Fx / (mu Fz))^2) so the longitudinal force each
// axle carries leaves less for cornering. Normal loads: static weight, downforce split by
// aero_balance_front, and longitudinal load transfer h (Fxf + Fxr) / L (drag acts at the CoG
// height, so it transfers no load). Longitudinal force per axle is limited by traction, mu Fz.
//
// Below kKinematicSpeed slip angles are ill-defined and the equations stiff, so the velocity
// derivatives blend linearly into those of the kinematic model, which ties vy = lr tan(d) vx / L
// and r = tan(d) vx / L, as in Kabzan et al., "AMZ Driverless: The full autonomous racing
// system", Journal of Field Robotics 37(7), 2020. A relaxation term (time constant
// kKinematicRelaxation) pulls vy and r onto those values, so a car that slows down while
// sliding comes to rest instead of spinning on the spot.
//
// Actuators. Steering: dead time, then a first-order lag towards the target clamped to
// max_steer, at most max_steer_rate. Drive and brake: dead time, then a first-order lag towards
// accel * mass, clamped to [-max_brake_force, max_drive_force]; the motors also deliver at most
// max_power / vx, and their force tapers to zero over the last meter per second below
// max_speed. Drive force is split by drive_split_front, brake force by brake_split_front.
//
// The car does not reverse: vx is clamped at zero, and at rest brakes and rolling resistance
// hold it there. Integration is classic fourth-order Runge-Kutta with the actuator targets held
// over the step.
class VehicleModel {
public:
    static constexpr double kStep = 1e-3;                 // s, the World's physics tick
    static constexpr double kKinematicSpeed = 1.5;        // m/s: purely kinematic below
    static constexpr double kDynamicSpeed = 3.0;          // m/s: purely dynamic above
    static constexpr double kKinematicRelaxation = 0.05;  // s

    // Throws std::invalid_argument for invalid parameters or a friction scale that is not
    // positive. friction_scale multiplies both axles' mu.
    explicit VehicleModel(const VehicleParams& params, double friction_scale = 1.0);

    // Places the car in `state` at `time` and forgets pending commands. The actuator targets
    // become the state's current steer and force, so the actuators hold still.
    void reset(const VehicleState& state, Time time = 0.0);

    // Received now: the steering responds after steer_dead_time, the drive after
    // drive_dead_time. Out-of-range values are clamped; a non-finite field is ignored, keeping
    // that actuator's previous target.
    void command(const ControlCommand& command);

    // Advances by dt (s). Commands whose dead time has elapsed take effect at the start of the
    // step. dt must be positive and at most a few milliseconds; other values are ignored.
    void step(double dt = kStep);

    [[nodiscard]] const VehicleState& state() const { return state_; }
    [[nodiscard]] Time time() const { return time_; }
    // At the current state.
    [[nodiscard]] const TireForces& tire_forces() const { return forces_; }
    [[nodiscard]] const VehicleParams& params() const { return params_; }
    [[nodiscard]] double friction_scale() const { return friction_scale_; }
    // Steps whose result was not finite: the car was stopped where it stood. Always zero unless
    // something is badly wrong; tests check it.
    [[nodiscard]] std::size_t recoveries() const { return recoveries_; }

private:
    using Vector = Eigen::Matrix<double, 8, 1>;  // x, y, yaw, vx, vy, r, steer, force

    struct Evaluation {
        Vector derivative = Vector::Zero();
        TireForces forces;
        double ax = 0.0;
        double ay = 0.0;
    };
    struct Pending {
        Time apply_at = 0.0;
        double value = 0.0;
    };

    [[nodiscard]] Evaluation evaluate(const Vector& x) const;
    [[nodiscard]] double drive_limit(double vx) const;
    void apply_due_commands();
    [[nodiscard]] Vector to_vector() const;
    void store(const Vector& x);

    VehicleParams params_;
    double friction_scale_ = 1.0;
    VehicleState state_;
    TireForces forces_;
    Time time_ = 0.0;
    double steer_target_ = 0.0;
    double force_target_ = 0.0;
    std::deque<Pending> steer_queue_;
    std::deque<Pending> force_queue_;
    std::size_t recoveries_ = 0;
};

}  // namespace apex::sim
