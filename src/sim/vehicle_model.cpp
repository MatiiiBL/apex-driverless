#include "apex/sim/vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "apex/core/angles.hpp"
#include "apex/core/log.hpp"

namespace apex::sim {

namespace {

enum Index : Eigen::Index { kX, kY, kYaw, kVx, kVy, kR, kSteer, kForce };

// Commands take effect at the first step that starts within this of their due time, which
// absorbs the rounding of a time accumulated over millions of steps.
constexpr double kTimeTolerance = 1e-6;  // s
// The motors' force fades to zero over this speed range below max_speed (field weakening).
constexpr double kTopSpeedTaper = 1.0;  // m/s
// The power limit divides by speed; below this the drive force limit applies instead.
constexpr double kMinPowerSpeed = 0.1;  // m/s
constexpr double kMaxStep = 0.01;       // s

void require(bool ok, const char* what) {
    if (!ok) throw std::invalid_argument(std::string("VehicleParams: ") + what);
}

bool finite_positive(double v) {
    return std::isfinite(v) && v > 0.0;
}
bool finite_non_negative(double v) {
    return std::isfinite(v) && v >= 0.0;
}
bool share(double v) {
    return std::isfinite(v) && v >= 0.0 && v <= 1.0;
}

void validate_tire(const TireParams& tire, const char* axle) {
    const std::string prefix = std::string(axle) + " tire ";
    if (!finite_positive(tire.B) || !finite_positive(tire.C) || !finite_positive(tire.mu) ||
        !std::isfinite(tire.E) || tire.E > 1.0) {
        throw std::invalid_argument("VehicleParams: " + prefix +
                                    "needs B, C, mu > 0 and E <= 1, all finite");
    }
}

// Lateral force of one axle (N) from the simplified magic formula, before combined slip.
double magic_formula(const TireParams& tire, double mu, double fz, double alpha) {
    const double b_alpha = tire.B * alpha;
    const double shape = std::atan(b_alpha - tire.E * (b_alpha - std::atan(b_alpha)));
    return -mu * fz * std::sin(tire.C * shape);
}

}  // namespace

void validate_vehicle_params(const VehicleParams& p) {
    require(finite_positive(p.mass), "mass must be positive");
    require(finite_positive(p.yaw_inertia), "yaw_inertia must be positive");
    require(finite_positive(p.lf) && finite_positive(p.lr), "lf and lr must be positive");
    require(finite_non_negative(p.cg_height), "cg_height must be non-negative");
    require(finite_positive(p.track_width), "track_width must be positive");
    require(finite_positive(p.body_front) && finite_positive(p.body_rear) &&
                finite_positive(p.body_width),
            "body dimensions must be positive");
    validate_tire(p.front_tire, "front");
    validate_tire(p.rear_tire, "rear");
    require(finite_non_negative(p.drag_area) && finite_non_negative(p.downforce_area) &&
                finite_non_negative(p.air_density),
            "aero coefficients and air density must be non-negative");
    require(share(p.aero_balance_front), "aero_balance_front must be in [0, 1]");
    require(finite_non_negative(p.rolling_resistance), "rolling_resistance must be >= 0");
    require(finite_positive(p.max_power), "max_power must be positive");
    require(finite_positive(p.max_drive_force), "max_drive_force must be positive");
    require(finite_positive(p.max_brake_force), "max_brake_force must be positive");
    require(share(p.drive_split_front), "drive_split_front must be in [0, 1]");
    require(share(p.brake_split_front), "brake_split_front must be in [0, 1]");
    require(finite_positive(p.max_speed) && p.max_speed > kTopSpeedTaper,
            "max_speed must exceed 1 m/s");
    // tan(steer) appears in the kinematic model; 1.2 rad is far beyond any real steering rack.
    require(finite_positive(p.max_steer) && p.max_steer <= 1.2, "max_steer must be in (0, 1.2]");
    require(finite_positive(p.max_steer_rate), "max_steer_rate must be positive");
    // RK4 at the 1 ms tick is stable for lags down to ~0.4 ms; 1 ms keeps a safe margin.
    require(std::isfinite(p.steer_time_constant) && p.steer_time_constant >= 1e-3,
            "steer_time_constant must be at least 1 ms");
    require(std::isfinite(p.drive_time_constant) && p.drive_time_constant >= 1e-3,
            "drive_time_constant must be at least 1 ms");
    require(finite_non_negative(p.steer_dead_time) && finite_non_negative(p.drive_dead_time),
            "dead times must be non-negative");
}

VehicleModel::VehicleModel(const VehicleParams& params, double friction_scale)
    : params_(params), friction_scale_(friction_scale) {
    validate_vehicle_params(params);
    if (!finite_positive(friction_scale)) {
        throw std::invalid_argument("VehicleModel: friction_scale must be positive");
    }
    reset(VehicleState{});
}

void VehicleModel::reset(const VehicleState& state, Time time) {
    state_ = state;
    state_.vx = std::max(state_.vx, 0.0);
    state_.steer = std::clamp(state_.steer, -params_.max_steer, params_.max_steer);
    time_ = time;
    steer_target_ = state_.steer;
    force_target_ = state_.force;
    steer_queue_.clear();
    force_queue_.clear();
    const Evaluation now = evaluate(to_vector());
    forces_ = now.forces;
    state_.ax = now.ax;
    state_.ay = now.ay;
}

void VehicleModel::command(const ControlCommand& command) {
    if (std::isfinite(command.steer)) {
        steer_queue_.push_back({time_ + params_.steer_dead_time,
                                std::clamp(command.steer, -params_.max_steer, params_.max_steer)});
    }
    if (std::isfinite(command.accel)) {
        force_queue_.push_back({time_ + params_.drive_dead_time,
                                std::clamp(command.accel * params_.mass, -params_.max_brake_force,
                                           params_.max_drive_force)});
    }
}

void VehicleModel::apply_due_commands() {
    while (!steer_queue_.empty() && steer_queue_.front().apply_at <= time_ + kTimeTolerance) {
        steer_target_ = steer_queue_.front().value;
        steer_queue_.pop_front();
    }
    while (!force_queue_.empty() && force_queue_.front().apply_at <= time_ + kTimeTolerance) {
        force_target_ = force_queue_.front().value;
        force_queue_.pop_front();
    }
}

double VehicleModel::drive_limit(double vx) const {
    const double power_limit = params_.max_power / std::max(vx, kMinPowerSpeed);
    const double taper = std::clamp((params_.max_speed - vx) / kTopSpeedTaper, 0.0, 1.0);
    return std::min(params_.max_drive_force, power_limit) * taper;
}

VehicleModel::Evaluation VehicleModel::evaluate(const Vector& x) const {
    const VehicleParams& p = params_;
    const double vx = x[kVx];
    const double vy = x[kVy];
    const double r = x[kR];
    const double steer = x[kSteer];
    const double length = p.wheelbase();

    Evaluation out;
    Vector& dx = out.derivative;
    TireForces& f = out.forces;

    // Actuators.
    dx[kSteer] = std::clamp((steer_target_ - steer) / p.steer_time_constant, -p.max_steer_rate,
                            p.max_steer_rate);
    dx[kForce] = (force_target_ - x[kForce]) / p.drive_time_constant;
    double applied = x[kForce];
    if (applied > 0.0) applied = std::min(applied, drive_limit(vx));
    const double split = applied >= 0.0 ? p.drive_split_front : p.brake_split_front;

    // Normal loads, and the longitudinal force each axle can carry. The load transfer depends on
    // the traction-limited forces, which depend on the loads: one fixed-point pass settles it to
    // well under a newton.
    const double v_air = std::max(vx, 0.0);
    const double downforce = p.downforce(v_air);
    const double static_front =
        p.mass * kGravity * p.lr / length + p.aero_balance_front * downforce;
    const double static_rear =
        p.mass * kGravity * p.lf / length + (1.0 - p.aero_balance_front) * downforce;
    const double mu_front = p.front_tire.mu * friction_scale_;
    const double mu_rear = p.rear_tire.mu * friction_scale_;
    double fx_front = split * applied;
    double fx_rear = (1.0 - split) * applied;
    for (int pass = 0; pass < 2; ++pass) {
        const double transfer = p.cg_height * (fx_front + fx_rear) / length;
        f.fz_front = std::max(static_front - transfer, 0.0);
        f.fz_rear = std::max(static_rear + transfer, 0.0);
        fx_front = std::clamp(split * applied, -mu_front * f.fz_front, mu_front * f.fz_front);
        fx_rear = std::clamp((1.0 - split) * applied, -mu_rear * f.fz_rear, mu_rear * f.fz_rear);
    }
    f.fx_front = fx_front;
    f.fx_rear = fx_rear;
    // Rolling resistance also acts at rest, as static friction (see the hold below), so a drive
    // force smaller than it does not make the car creep.
    const double resistance =
        p.drag(v_air) + p.rolling_resistance * (p.mass * kGravity + downforce);

    // Dynamic model.
    f.slip_front = std::atan2(vy + p.lf * r, vx) - steer;
    f.slip_rear = std::atan2(vy - p.lr * r, vx);
    auto lateral = [](const TireParams& tire, double mu, double fz, double fx, double alpha) {
        const double capacity = mu * fz;
        if (!(capacity > 0.0)) return 0.0;
        const double used = std::min(std::abs(fx) / capacity, 1.0);
        return magic_formula(tire, mu, fz, alpha) * std::sqrt(1.0 - used * used);
    };
    f.fy_front = lateral(p.front_tire, mu_front, f.fz_front, fx_front, f.slip_front);
    f.fy_rear = lateral(p.rear_tire, mu_rear, f.fz_rear, fx_rear, f.slip_rear);
    const double cos_steer = std::cos(steer);
    const double sin_steer = std::sin(steer);
    const double front_lateral = fx_front * sin_steer + f.fy_front * cos_steer;
    const double force_x = fx_front * cos_steer - f.fy_front * sin_steer + fx_rear - resistance;
    const double force_y = front_lateral + f.fy_rear;
    const double dyn_vx = force_x / p.mass + vy * r;
    const double dyn_vy = force_y / p.mass - vx * r;
    const double dyn_r = (p.lf * front_lateral - p.lr * f.fy_rear) / p.yaw_inertia;

    // Kinematic model: the derivative of the constraints vy = lr tan(d) vx / L and
    // r = tan(d) vx / L, plus relaxation onto them.
    const double tan_steer = std::tan(steer);
    const double kin_vx = (fx_front + fx_rear - resistance) / p.mass;
    const double curvature_rate =
        (kin_vx * tan_steer + vx * dx[kSteer] * (1.0 + tan_steer * tan_steer)) / length;
    const double kin_vy =
        p.lr * curvature_rate + (p.lr * tan_steer * vx / length - vy) / kKinematicRelaxation;
    const double kin_r = curvature_rate + (tan_steer * vx / length - r) / kKinematicRelaxation;

    const double speed = std::hypot(vx, vy);
    f.blend = std::clamp((speed - kKinematicSpeed) / (kDynamicSpeed - kKinematicSpeed), 0.0, 1.0);
    const double w = f.blend;
    dx[kVx] = w * dyn_vx + (1.0 - w) * kin_vx;
    dx[kVy] = w * dyn_vy + (1.0 - w) * kin_vy;
    dx[kR] = w * dyn_r + (1.0 - w) * kin_r;
    // At rest, brakes and rolling resistance are static friction: they hold, never push back.
    if (vx <= 0.0 && dx[kVx] < 0.0) dx[kVx] = 0.0;

    const double cos_yaw = std::cos(x[kYaw]);
    const double sin_yaw = std::sin(x[kYaw]);
    dx[kX] = vx * cos_yaw - vy * sin_yaw;
    dx[kY] = vx * sin_yaw + vy * cos_yaw;
    dx[kYaw] = r;

    out.ax = dx[kVx] - vy * r;
    out.ay = dx[kVy] + vx * r;
    return out;
}

VehicleModel::Vector VehicleModel::to_vector() const {
    Vector x;
    x << state_.pose.x, state_.pose.y, state_.pose.yaw, state_.vx, state_.vy, state_.yaw_rate,
        state_.steer, state_.force;
    return x;
}

void VehicleModel::store(const Vector& x) {
    state_.pose = {x[kX], x[kY], wrap_angle(x[kYaw])};
    state_.vx = std::max(x[kVx], 0.0);
    state_.vy = x[kVy];
    state_.yaw_rate = x[kR];
    state_.steer = std::clamp(x[kSteer], -params_.max_steer, params_.max_steer);
    state_.force = x[kForce];
}

void VehicleModel::step(double dt) {
    if (!(dt > 0.0 && dt <= kMaxStep)) return;
    apply_due_commands();
    const Vector x = to_vector();
    const Vector k1 = evaluate(x).derivative;
    const Vector k2 = evaluate(x + 0.5 * dt * k1).derivative;
    const Vector k3 = evaluate(x + 0.5 * dt * k2).derivative;
    const Vector k4 = evaluate(x + dt * k3).derivative;
    const Vector next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    time_ += dt;
    if (next.allFinite()) {
        store(next);
    } else {
        ++recoveries_;
        log(LogLevel::Error, "VehicleModel: non-finite state at t = {:.3f} s; stopping the car",
            time_);
        state_.vx = 0.0;
        state_.vy = 0.0;
        state_.yaw_rate = 0.0;
        state_.force = 0.0;
    }
    apply_due_commands();
    const Evaluation now = evaluate(to_vector());
    forces_ = now.forces;
    state_.ax = now.ax;
    state_.ay = now.ay;
}

}  // namespace apex::sim
