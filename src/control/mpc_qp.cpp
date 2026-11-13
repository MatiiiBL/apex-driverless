#include "mpc_qp.hpp"

#include <algorithm>

#include "apex/control/mpc_model.hpp"
#include "apex/control/reference.hpp"

namespace apex::control::mpc {

namespace {

using Row = qp::LinearConstraint<kStates, kInputs>;

Row input_bound(int input, double lower, double upper) {
    Row row;
    row.d(input) = 1.0;
    row.lower = lower;
    row.upper = upper;
    return row;
}

// Below this speed the course error's sideslip term would be ill-scaled.
constexpr double kMinCourseSpeed = 1.0;  // m/s

}  // namespace

void build_qp(const MpcConfig& config, const NominalModel& model,
              const std::vector<StagePoint>& guess, double dt, Qp& problem) {
    const VehicleParams& p = model.params();
    const std::size_t n = guess.size() - 1;
    const double inset = 0.5 * p.body_width + config.lateral_margin;
    problem.stages.resize(n + 1);
    problem.x0 = guess[0].x;
    // The speed target is the reference's, capped by what the car can reach from its current
    // speed at the traction limit. A target far out of reach (the full profile, from rest)
    // would make the QP chase it through the linearization's errors.
    double reachable = std::max(guess[0].x(kVx), 0.0);
    for (std::size_t k = 0; k <= n; ++k) {
        const StagePoint& point = guess[k];
        const TrajectoryPoint& ref = point.reference;
        const double speed = std::max(point.x(kVx), 0.0);
        const AccelRange accel =
            model.traction_limits(speed, point.x(kVx) * point.x(kYawRate), config.grip_fraction);
        const double target_speed = std::min(ref.speed, reachable);
        reachable += accel.max * dt;
        qp::OcpStage<kStates, kInputs>& stage = problem.stages[k];

        if (k < n) {
            const DiscreteModel d = discretize(model, point.x, point.u, ref.curvature, dt);
            stage.A = d.A;
            stage.B = d.B;
            stage.b = d.c;
        } else {
            stage.A.setIdentity();
            stage.B.setZero();
            stage.b.setZero();
        }

        stage.Q.setZero();
        stage.S.setZero();
        stage.q.setZero();
        if (k > 0) {
            const double scale = k == n ? config.terminal_scale : 1.0;
            State course = State::Zero();
            course(kHeading) = 1.0;
            course(kVy) = 1.0 / std::max(speed, kMinCourseSpeed);
            stage.Q.noalias() = (scale * config.weight_course) * course * course.transpose();
            stage.Q(kLateral, kLateral) += scale * config.weight_lateral;
            stage.Q(kVx, kVx) += scale * config.weight_speed;
            stage.q(kVx) = -scale * config.weight_speed * target_speed;
        }
        const double feed_forward = ref.accel + model.resistance(speed);
        stage.R.setZero();
        stage.R.diagonal() << config.weight_steer_rate, config.weight_accel, config.slack_quadratic;
        stage.r << 0.0, -config.weight_accel * feed_forward, config.slack_linear;

        stage.constraints.clear();
        stage.constraints.push_back(input_bound(kSlack, 0.0, qp::kInfinity));
        if (k < n) {
            stage.constraints.push_back(
                input_bound(kSteerRate, -p.max_steer_rate, p.max_steer_rate));
            stage.constraints.push_back(input_bound(kAccel, accel.min, accel.max));
        }
        if (k == 0) continue;
        Row steer;
        steer.c(kSteer) = 1.0;
        steer.lower = -p.max_steer;
        steer.upper = p.max_steer;
        stage.constraints.push_back(steer);
        // A reference without widths (both zero) carries no lateral bounds.
        if (ref.width_left + ref.width_right > 0.0) {
            Row left;
            left.c(kLateral) = 1.0;
            left.d(kSlack) = -1.0;
            left.upper = ref.width_left - inset;
            stage.constraints.push_back(left);
            Row right;
            right.c(kLateral) = 1.0;
            right.d(kSlack) = 1.0;
            right.lower = -(ref.width_right - inset);
            stage.constraints.push_back(right);
        }
    }
}

void guess_from_solution(const Trajectory& reference, const Solution& solution, double dt,
                         std::vector<StagePoint>& guess) {
    double s = guess[0].s;
    for (std::size_t k = 0; k < guess.size(); ++k) {
        StagePoint& point = guess[k];
        if (k > 0) {
            const double curvature = guess[k - 1].reference.curvature;
            s += 0.5 * dt *
                 (path_speed(to_array(solution.x[k - 1]), curvature) +
                  path_speed(to_array(solution.x[k]), curvature));
        }
        point.s = s;
        point.x = solution.x[k];
        point.u = solution.u[k];
        point.reference = sample_reference(reference, s);
    }
}

}  // namespace apex::control::mpc
