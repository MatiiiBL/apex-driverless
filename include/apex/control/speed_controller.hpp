#pragma once

#include "apex/control/controller.hpp"
#include "apex/control/vehicle_model.hpp"

// The longitudinal loop of pure pursuit and Stanley (SpeedLoopConfig). The command is total
// tire force over mass, and the vehicle does not compensate its own drag and rolling
// resistance, so the feed-forward adds the nominal model's resistance to the reference
// acceleration; the PI term absorbs what the nominal model gets wrong.
//
// Anti-windup: the integral term is clamped to +/- integral_limit and frozen while the output
// saturates in the direction the error pushes it (conditional integration; K. J. Astrom and
// T. Hagglund, "Advanced PID Control", ISA, 2006, section 3.5).
namespace apex::control {

class SpeedController {
public:
    SpeedController(const SpeedLoopConfig& config, const NominalModel& model);

    // The acceleration command for a target speed and its feed-forward acceleration at the
    // current speed, dt seconds after the previous update. Within the drive and brake limits.
    [[nodiscard]] double update(double target_speed, double target_accel, double speed, double dt);
    void reset() { integral_ = 0.0; }
    [[nodiscard]] double integral() const { return integral_; }

private:
    SpeedLoopConfig config_;
    NominalModel model_;
    double integral_ = 0.0;  // m/s^2
};

}  // namespace apex::control
