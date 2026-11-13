#pragma once

#include <cmath>

#include "apex/core/messages.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/sim/vehicle_model.hpp"

// The car the control tests and evaluations drive: the simulator's vehicle model (dynamic
// single-track, Pacejka tires with combined slip, load transfer, aero, power and traction
// limits, actuator dead time, lag and rate limits) stepped at its 1 ms tick, with the true
// state as the estimate. A test gives the controller a different, nominal copy of the
// parameters, or a friction scale here, when it wants model mismatch.
namespace apex::control::testing {

class Plant {
public:
    Plant(const VehicleParams& truth, const Pose2& start, double speed = 0.0,
          double friction_scale = 1.0)
        : model_(truth, friction_scale) {
        VehicleState state;
        state.pose = start;
        state.vx = speed;
        model_.reset(state);
    }

    // Reaches the actuators after their dead times.
    void command(const ControlCommand& command) { model_.command(command); }

    // Advances by whole physics ticks.
    void advance(double seconds) {
        const auto ticks = std::lround(seconds / sim::VehicleModel::kStep);
        for (long i = 0; i < ticks; ++i) model_.step();
    }

    [[nodiscard]] Time time() const { return model_.time(); }
    [[nodiscard]] const VehicleState& truth() const { return model_.state(); }
    [[nodiscard]] const sim::VehicleModel& model() const { return model_; }

    [[nodiscard]] StateEstimate estimate() const {
        const VehicleState& s = model_.state();
        StateEstimate e;
        e.stamp = model_.time();
        e.pose = s.pose;
        e.vx = s.vx;
        e.vy = s.vy;
        e.yaw_rate = s.yaw_rate;
        e.valid = true;
        return e;
    }

private:
    sim::VehicleModel model_;
};

}  // namespace apex::control::testing
