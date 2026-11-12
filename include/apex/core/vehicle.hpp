#pragma once

#include "apex/core/types.hpp"

namespace apex {

// Simplified Pacejka "magic formula" for the lateral force of one axle:
//   Fy = -mu Fz sin(C atan(B alpha - E (B alpha - atan(B alpha))))
// with slip angle alpha in radians. B = 12, C = 1.45 peak near 7.5 degrees of slip, typical of
// FS slicks.
struct TireParams {
    double B = 12.0;  // stiffness factor
    double C = 1.45;  // shape factor
    double E = 0.0;   // curvature factor
    double mu = 1.6;  // peak friction coefficient
};

// A representative Formula Student electric car with an autonomous system and no driver: the
// values are typical of the class, not of a specific team's car. The simulator uses these as the
// truth; the car's software gets its own copy, which experiments may perturb (model mismatch).
struct VehicleParams {
    double mass = 240.0;         // kg
    double yaw_inertia = 130.0;  // kg m^2
    double lf = 0.80;            // CoG to front axle (m)
    double lr = 0.75;            // CoG to rear axle (m)
    double cg_height = 0.27;     // m
    double track_width = 1.22;   // m

    // Body footprint for cone-hit detection, from the CoG.
    double body_front = 1.60;  // CoG to the front of the front wing (m)
    double body_rear = 1.30;   // CoG to the back of the rear wing (m)
    double body_width = 1.40;  // m

    TireParams front_tire;
    TireParams rear_tire;

    double drag_area = 1.40;           // Cd * A (m^2)
    double downforce_area = 3.50;      // Cl * A (m^2)
    double air_density = 1.20;         // kg/m^3
    double aero_balance_front = 0.45;  // share of downforce on the front axle
    double rolling_resistance = 0.015;

    double max_power = 80e3;          // W, the FS rules' limit at the accumulator
    double max_drive_force = 3000.0;  // N, all motors at low speed
    double max_brake_force = 5000.0;  // N, hydraulic limit (the tires saturate first)
    double drive_split_front = 0.40;  // share of drive force on the front axle (AWD)
    double brake_split_front = 0.60;  // share of brake force on the front axle
    double max_speed = 30.0;          // m/s, gearing limit

    double max_steer = 0.40;            // rad, front wheel angle
    double max_steer_rate = 1.5;        // rad/s, steering actuator
    double steer_time_constant = 0.03;  // s, first-order lag of the steering actuator
    double steer_dead_time = 0.03;      // s, command to actuator response
    double drive_time_constant = 0.05;  // s, first-order lag of the force response
    double drive_dead_time = 0.02;      // s

    [[nodiscard]] double wheelbase() const { return lf + lr; }
    // Aerodynamic downforce and drag at speed v (N).
    [[nodiscard]] double downforce(double v) const;
    [[nodiscard]] double drag(double v) const;
};

// Ground-truth state of the simulated car. The car's software sees a StateEstimate instead.
struct VehicleState {
    Pose2 pose;             // CoG in the world frame
    double vx = 0.0;        // body-frame velocity at the CoG (m/s), x forward
    double vy = 0.0;        // y left
    double yaw_rate = 0.0;  // rad/s
    double steer = 0.0;     // actual front wheel angle (rad), after the actuator
    double force = 0.0;     // actual total longitudinal tire force (N), after the actuator
    double ax = 0.0;        // body-frame acceleration at the CoG (m/s^2): what an IMU measures
    double ay = 0.0;
};

// What the car's software asks of the actuators.
struct ControlCommand {
    Time stamp = 0.0;
    double steer = 0.0;  // target front wheel angle (rad), positive to the left
    // Target total longitudinal tire force divided by the mass (m/s^2): positive drives,
    // negative brakes. The vehicle does not compensate drag or rolling resistance; the
    // controller must. Power, traction and actuator limits apply.
    double accel = 0.0;
};

}  // namespace apex
