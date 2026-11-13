#pragma once

#include <vector>

#include "apex/control/mpc_model.hpp"
#include "apex/control/vehicle_model.hpp"
#include "apex/core/trajectory.hpp"
#include "apex/core/types.hpp"

// The MPC's warm start. A solved plan is kept in the world frame (poses, velocities, inputs at
// absolute times), not in path coordinates or by stage index: during lap 1 the reference is
// replanned every 100 ms, with a new arc-length origin each time, and only the world-frame plan
// means the same thing on the new reference. At the next call the plan's inputs, shifted in
// time, are rolled out through the nonlinear model from the new initial state along the
// current reference, which gives the stations, error states and inputs that the next QP is
// linearized around and warm-started from.
//
// The rollout, rather than the plan's own states re-projected onto the new reference, is what
// keeps the linearization physical. A replanned exploration path can move its end (where its
// speed profile reaches zero) or start with a kink back to the track's middle, and the old
// plan's states then disagree with the new reference and with each other; linearized around
// such states the model can be unstable, and the QP's optimum about it was absurd (90 m/s, 10 m
// off the path), which as the next plan made the next linearization worse. A rollout of
// bounded inputs through the nonlinear model stays a trajectory the car can drive.
namespace apex::control::mpc {

struct PlanStage {
    Time time = 0.0;
    Pose2 pose;  // CoG, world frame
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
    double steer = 0.0;
    double steer_rate = 0.0;  // the inputs applied from this stage to the next
    double accel = 0.0;
    double slack = 0.0;
};

class Plan {
public:
    [[nodiscard]] bool empty() const { return stages_.empty(); }
    void clear() { stages_.clear(); }
    [[nodiscard]] const std::vector<PlanStage>& stages() const { return stages_; }
    [[nodiscard]] std::vector<PlanStage>& stages() { return stages_; }

    // The plan at time t: interpolated between stages (yaw along the shorter arc), held before
    // the first stage, and continued at the last stage's velocity beyond the last one.
    [[nodiscard]] PlanStage at(Time t) const;

private:
    std::vector<PlanStage> stages_;
};

// The linearization point of one stage: its predicted station on the current reference, the
// error state and input there, and the reference sampled at that station.
struct StagePoint {
    double s = 0.0;  // unwrapped: increases along the horizon even across a closed path's end
    State x = State::Zero();
    Input u = Input::Zero();
    TrajectoryPoint reference;
};

// Makes `guess` (horizon guess.size() - 1) a trajectory of the nonlinear model: its inputs are
// kept, clamped to the actuator limits (steering rate and angle, drive and brake), and its
// states and stations replaced by integrating the model (mpc_model.hpp, RK4) from x0 at
// station s0, with each stage's curvature from `reference` at its station. The car does not
// reverse: the speed stops at zero.
void rollout(const NominalModel& model, const Trajectory& reference, const State& x0, double s0,
             double dt, std::vector<StagePoint>& guess);

// Stage points for horizon guess.size() - 1 starting at time t0 from x0 at station s0: the
// plan's inputs at t0 + k dt, rolled out.
void guess_from_plan(const NominalModel& model, const Plan& plan, const Trajectory& reference,
                     const State& x0, double s0, Time t0, double dt,
                     std::vector<StagePoint>& guess);

// Stage points from the reference alone, for the first call: the speed follows the profile
// (within comfortable acceleration), the errors decay, and the steering is the kinematic one
// for the path's curvature.
void guess_from_reference(const NominalModel& model, const Trajectory& reference, const State& x0,
                          double s0, double dt, std::vector<StagePoint>& guess);

// The world-frame plan of a solution: each stage's error state placed relative to the reference
// at the station reached by integrating s_dot along the solution from the first stage's
// station.
void plan_from_solution(const Trajectory& reference, double s0, const std::vector<State>& states,
                        const std::vector<Input>& inputs, Time t0, double dt, Plan& plan);

}  // namespace apex::control::mpc
