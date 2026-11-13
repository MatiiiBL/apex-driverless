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

struct SlipAngle {
    double alpha = 0.0;        // rad, with the formula's sign convention: Fy = -mu Fz sin(...)
    double utilization = 0.0;  // |Fy| / (mu Fz): 1 at the peak of the curve
};

// The slip angle at which a tire produces lateral force `fy` under normal load `fz`: the inverse
// of the magic formula on its rising branch. Forces at or beyond the peak return the peak slip
// angle with utilization >= 1, where the inverse is meaningless.
[[nodiscard]] SlipAngle slip_angle_for_force(const TireParams& tire, double fz, double fy);

struct AxleSlip {
    SlipAngle front;
    SlipAngle rear;
};

// Axle slip angles from the lateral force each axle must produce. The forces come from the
// planar equations of motion with the IMU's ay and an estimate of the yaw acceleration:
//   Fyf + Fyr = m ay,   lf Fyf - lr Fyr = Iz r'
// (small steering angle). Steady state gives the familiar m ay lr / L at the front.
[[nodiscard]] AxleSlip predict_axle_slip(const VehicleParams& vehicle, double speed, double ax,
                                         double ay, double yaw_accel);

}  // namespace apex::estimation
