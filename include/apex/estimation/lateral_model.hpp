#pragma once

#include "apex/core/vehicle.hpp"

// The single-track (bicycle) model's lateral relations, used by the velocity filter as
// pseudo-measurements: given the measured steering angle, the IMU's lateral acceleration and the
// nominal tires, each axle's slip angle is predicted, which ties vy and the yaw rate to vx.
// Model: Rajamani, "Vehicle Dynamics and Control", 2nd ed., Springer, 2012, chapter 2; tire
// formula as in core/vehicle.hpp (Pacejka, "Tire and Vehicle Dynamics", 3rd ed., 2012).
namespace apex::estimation {

struct AxleLoads {
    double front = 0.0;  // N
    double rear = 0.0;
};

// Static weight split by the CoG position, downforce split by aero_balance_front, and the
// longitudinal load transfer m ax h / L (ax: body-frame acceleration, positive forward).
[[nodiscard]] AxleLoads axle_normal_loads(const VehicleParams& vehicle, double speed, double ax);

struct AxleForces {
    double front = 0.0;  // N, positive driving
    double rear = 0.0;
};

// The longitudinal tire force of each axle: the measured specific force times the mass, plus
// the drag and rolling resistance it overcomes while moving, split as the drivetrain (driving)
// or the brakes (braking) split it.
[[nodiscard]] AxleForces axle_longitudinal_forces(const VehicleParams& vehicle, double speed,
                                                  double ax);

struct SlipAngle {
    double alpha = 0.0;        // rad, with the formula's sign convention: Fy = -mu Fz sin(...)
    double utilization = 0.0;  // of the lateral grip the longitudinal force leaves: 1 at the peak
};

// The slip angle at which a tire produces lateral force `fy` under normal load `fz` while it
// also carries longitudinal force `fx`: the inverse of the magic formula on its rising branch,
// with the lateral capacity reduced by the friction ellipse, mu Fz sqrt(1 - (Fx / mu Fz)^2), as
// combined slip does (Pacejka, 2012, chapter 4). Forces at or beyond the peak return the peak
// slip angle with utilization >= 1, where the inverse is meaningless.
[[nodiscard]] SlipAngle slip_angle_for_force(const TireParams& tire, double fz, double fy,
                                             double fx = 0.0);

struct AxleSlip {
    SlipAngle front;
    SlipAngle rear;
};

// Axle slip angles from the lateral force each axle must produce. The forces come from the
// planar equations of motion with the IMU's ay and an estimate of the yaw acceleration:
//   Fyf + Fyr = m ay,   lf Fyf - lr Fyr = Iz r'
// (small steering angle). Steady state gives the familiar m ay lr / L at the front. Each axle's
// longitudinal force (axle_longitudinal_forces) narrows its lateral capacity.
[[nodiscard]] AxleSlip predict_axle_slip(const VehicleParams& vehicle, double speed, double ax,
                                         double ay, double yaw_accel);

}  // namespace apex::estimation
