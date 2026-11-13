#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/core/track.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/planning/ggv.hpp"
#include "apex/planning/planning.hpp"
#include "apex/sim/vehicle_model.hpp"

namespace apex::planning {
namespace {

// A circle of radius r, counter-clockwise, with the exact curvature at every point.
Trajectory circle(double r, std::size_t n = 400) {
    std::vector<TrajectoryPoint> points(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double angle = kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        points[i].position = {r * std::cos(angle), r * std::sin(angle)};
        points[i].yaw = wrap_angle(angle + 0.5 * kPi);
        points[i].curvature = 1.0 / r;
    }
    return Trajectory(std::move(points), true);
}

Trajectory straight(double length, double spacing = 0.5) {
    const auto n = static_cast<std::size_t>(std::round(length / spacing)) + 1;
    std::vector<TrajectoryPoint> points(n);
    for (std::size_t i = 0; i < n; ++i) points[i].position = {spacing * static_cast<double>(i), 0};
    return Trajectory(std::move(points), false);
}

// Car parameters with no drag or rolling resistance and no gearing limit, so the cornering
// speed is the pure friction limit.
VehicleParams frictionless_air() {
    VehicleParams car;
    car.drag_area = 0.0;
    car.rolling_resistance = 0.0;
    car.max_speed = 100.0;
    return car;
}

SpeedProfileConfig uncapped(double grip = 0.85) {
    return {.grip_factor = grip, .max_speed = 100.0};
}

// Downforce per v^2 (N s^2 / m^2).
double downforce_coefficient(const VehicleParams& car) {
    return 0.5 * car.air_density * car.downforce_area;
}

// How much of each axle's friction ellipse the car uses at speed v with a total lateral tire
// force `lateral` and longitudinal tire force `fx` (positive driving), worked out from the
// vehicle parameters independently of GgvEnvelope: normal loads with downforce and load
// transfer, the longitudinal split of the drive or the brakes, the lateral split of steady
// cornering. 1 is the ellipse's edge.
struct AxleUse {
    double front;
    double rear;
};
AxleUse axle_use(const VehicleParams& car, double grip_factor, double v, double lateral,
                 double fx) {
    const double length = car.wheelbase();
    const double q = downforce_coefficient(car);
    const double transfer = car.cg_height * fx / length;
    const double load_front =
        car.mass * kGravity * car.lr / length + car.aero_balance_front * q * v * v - transfer;
    const double load_rear = car.mass * kGravity * car.lf / length +
                             (1.0 - car.aero_balance_front) * q * v * v + transfer;
    const double split = fx >= 0.0 ? car.drive_split_front : car.brake_split_front;
    const auto use = [&](double longitudinal, double side, double mu, double load) {
        return std::hypot(longitudinal, side) / (grip_factor * mu * load);
    };
    return {use(split * fx, car.lr / length * lateral, car.front_tire.mu, load_front),
            use((1.0 - split) * fx, car.lf / length * lateral, car.rear_tire.mu, load_rear)};
}

TEST(SpeedProfile, CircleMatchesTheClosedFormWithDownforce) {
    const VehicleParams car = frictionless_air();
    const SpeedProfileConfig config = uncapped();
    for (const double r : {8.0, 15.0, 40.0}) {
        Trajectory path = circle(r);
        apply_speed_profile(path, car, config);
        // Each axle holds m v^2 / r * (static share) with f mu (static load + downforce share):
        // v^2 / r = f mu (g + beta q v^2 / (m w))  =>  v^2 = f mu g / (1/r - f mu beta q / (m w)).
        const double q = downforce_coefficient(car);
        const double f_mu = config.grip_factor * car.front_tire.mu;
        const double w_front = car.lr / car.wheelbase();
        const double w_rear = car.lf / car.wheelbase();
        const double v_front = std::sqrt(
            f_mu * kGravity / (1.0 / r - f_mu * car.aero_balance_front * q / (car.mass * w_front)));
        const double v_rear =
            std::sqrt(f_mu * kGravity /
                      (1.0 / r - f_mu * (1.0 - car.aero_balance_front) * q / (car.mass * w_rear)));
        const double expected = std::min(v_front, v_rear);
        for (const TrajectoryPoint& p : path.points()) {
            EXPECT_NEAR(p.speed, expected, 1e-9 * expected) << "r = " << r;
            EXPECT_NEAR(p.accel, 0.0, 1e-6);
        }
    }
}

TEST(SpeedProfile, CircleWithoutDownforceIsSqrtOfMuGR) {
    VehicleParams car = frictionless_air();
    car.downforce_area = 0.0;
    Trajectory path = circle(20.0);
    apply_speed_profile(path, car, uncapped(0.9));
    EXPECT_NEAR(path[0].speed, std::sqrt(0.9 * 1.6 * kGravity * 20.0), 1e-9);
}

TEST(SpeedProfile, CircleWithDragSitsOnTheFrictionEllipse) {
    const VehicleParams car;  // drag and rolling resistance on
    const SpeedProfileConfig config = uncapped();
    Trajectory path = circle(15.0);
    apply_speed_profile(path, car, config);
    const double v = path[0].speed;
    for (const TrajectoryPoint& p : path.points()) EXPECT_NEAR(p.speed, v, 1e-9);

    // Steady state: the tires carry m v^2 / r sideways and the drive force that balances drag
    // and rolling resistance; the more loaded of the two axles is exactly at its ellipse's edge.
    const double q = downforce_coefficient(car);
    const double resistance =
        car.drag(v) + car.rolling_resistance * (car.mass * kGravity + q * v * v);
    const AxleUse use = axle_use(car, config.grip_factor, v, car.mass * v * v / 15.0, resistance);
    EXPECT_NEAR(std::max(use.front, use.rear), 1.0, 1e-9);
    EXPECT_LT(std::min(use.front, use.rear), 1.0);

    // Slower than without drag.
    Trajectory frictionless = circle(15.0);
    apply_speed_profile(frictionless, frictionless_air(), config);
    EXPECT_LT(v, frictionless[0].speed);
}

// Straight-line braking, tire-limited. Braking B moves h B / L of load from the rear to the
// front, and the rear takes 40% of B, so the rear runs out first: (1 - s) B <= k (N_r - h B / L)
// gives B = k N_r / (1 - s + k h / L), with N_r = m g lf / L + (1 - beta) q v^2. With drag and
// rolling resistance helping, the deceleration is b(v) = a0 + a2 v^2, and u = v^2 obeys
// du/dsigma = 2 (a0 + a2 u) going back from the stop, so v(sigma) = sqrt(a0 / a2 (exp(2 a2
// sigma) - 1)).
TEST(SpeedProfile, TireLimitedBrakingMatchesClosedForm) {
    VehicleParams car;
    car.max_speed = 100.0;
    car.max_brake_force = 1e6;
    const SpeedProfileConfig config = uncapped();
    Trajectory path = straight(150.0);
    apply_speed_profile(path, car, config, 30.0, 0.0);

    const double q = downforce_coefficient(car);
    const double k = config.grip_factor * car.rear_tire.mu;
    const double denominator = 1.0 - car.brake_split_front + k * car.cg_height / car.wheelbase();
    const double a0 =
        k * kGravity * car.lf / car.wheelbase() / denominator + car.rolling_resistance * kGravity;
    const double a2 = (k * (1.0 - car.aero_balance_front) * q / denominator +
                       0.5 * car.air_density * car.drag_area + car.rolling_resistance * q) /
                      car.mass;
    // The front, which gains the load, could brake harder at every speed.
    for (const double v : {0.0, 30.0}) {
        const AxleUse use = axle_use(car, config.grip_factor, v, 0.0,
                                     -k *
                                         (car.mass * kGravity * car.lf / car.wheelbase() +
                                          (1.0 - car.aero_balance_front) * q * v * v) /
                                         denominator);
        EXPECT_NEAR(use.rear, 1.0, 1e-12);
        EXPECT_LT(use.front, 1.0);
    }
    for (const TrajectoryPoint& p : path.points()) {
        const double sigma = path.length() - p.s;
        if (sigma > 15.0) continue;
        const double expected = std::sqrt(a0 / a2 * (std::exp(2.0 * a2 * sigma) - 1.0));
        EXPECT_NEAR(p.speed, expected, 2e-3 * expected + 1e-9) << "sigma = " << sigma;
    }
    EXPECT_DOUBLE_EQ(path.points().back().speed, 0.0);
}

TEST(SpeedProfile, BrakeForceLimitedBrakingMatchesClosedForm) {
    VehicleParams car;
    car.max_speed = 100.0;
    car.max_brake_force = 2000.0;  // below the tires' limit at every speed
    Trajectory path = straight(150.0);
    apply_speed_profile(path, car, uncapped(), 30.0, 0.0);

    const double q = downforce_coefficient(car);
    const double a0 = car.max_brake_force / car.mass + car.rolling_resistance * kGravity;
    const double a2 =
        (0.5 * car.air_density * car.drag_area + car.rolling_resistance * q) / car.mass;
    // Distance to stop from 20 m/s: sigma = ln(1 + a2 v^2 / a0) / (2 a2).
    const double v0 = 20.0;
    const double expected_distance = std::log(1.0 + a2 * v0 * v0 / a0) / (2.0 * a2);
    double distance = 0.0;
    for (std::size_t i = path.size() - 1; i > 0; --i) {
        if (path[i - 1].speed >= v0) {
            const double u = (v0 - path[i].speed) / (path[i - 1].speed - path[i].speed);
            distance = path.length() - (path[i].s - u * (path[i].s - path[i - 1].s));
            break;
        }
    }
    EXPECT_NEAR(distance, expected_distance, 0.05);
}

// Drive-force-limited acceleration from rest: du/ds = 2 (F / m - c_rr g - c2 u), so
// u(s) = (c0 / c2) (1 - exp(-2 c2 s)). The drive force is set below the front axle's traction
// (2.4 kN at rest, next test).
TEST(SpeedProfile, DriveForceLimitedAccelerationMatchesClosedForm) {
    VehicleParams car;
    car.max_speed = 100.0;
    car.max_power = 1e9;
    car.max_drive_force = 2000.0;
    Trajectory path = straight(200.0);
    apply_speed_profile(path, car, uncapped(), 0.0, 0.0);

    const double q = downforce_coefficient(car);
    const double c0 = car.max_drive_force / car.mass - car.rolling_resistance * kGravity;
    const double c2 =
        (0.5 * car.air_density * car.drag_area + car.rolling_resistance * q) / car.mass;
    for (const TrajectoryPoint& p : path.points()) {
        if (p.s > 40.0) break;
        const double expected = std::sqrt(c0 / c2 * (1.0 - std::exp(-2.0 * c2 * p.s)));
        EXPECT_NEAR(p.speed, expected, 2e-3 * expected + 1e-9) << "s = " << p.s;
    }
}

// Traction-limited acceleration from rest. Driving moves load to the rear, and the front takes
// 40% of the force: s F <= k (N_f - h F / L) gives F = k N_f / (s + k h / L), at rest 2.4 kN
// of the motors' 3 kN. With resistance, du/ds = 2 (c0 + c2 u), so u(s) = (c0 / c2)
// (exp(2 c2 s) - 1).
TEST(SpeedProfile, TractionLimitedAccelerationIsTheFrontAxles) {
    VehicleParams car;
    car.max_speed = 100.0;
    car.max_power = 1e9;
    car.max_drive_force = 1e6;
    const SpeedProfileConfig config = uncapped();
    Trajectory path = straight(200.0);
    apply_speed_profile(path, car, config, 0.0, 0.0);

    const double q = downforce_coefficient(car);
    const double k = config.grip_factor * car.front_tire.mu;
    const double denominator = car.drive_split_front + k * car.cg_height / car.wheelbase();
    const double c0 =
        k * kGravity * car.lr / car.wheelbase() / denominator - car.rolling_resistance * kGravity;
    const double c2 = (k * car.aero_balance_front * q / denominator -
                       0.5 * car.air_density * car.drag_area - car.rolling_resistance * q) /
                      car.mass;
    for (const TrajectoryPoint& p : path.points()) {
        if (p.s > 40.0) break;
        const double expected = std::sqrt(c0 / c2 * (std::exp(2.0 * c2 * p.s) - 1.0));
        EXPECT_NEAR(p.speed, expected, 2e-3 * expected + 1e-9) << "s = " << p.s;
    }
}

// Braking into a corner: the envelope's braking force leaves exactly one axle at the edge of
// its friction ellipse, the rear (it loses load and still carries lf / L of the cornering
// force), and asks no axle for more.
TEST(SpeedProfile, BrakingInACornerSaturatesTheRearAxleExactly) {
    const VehicleParams car;
    const SpeedProfileConfig config;
    const GgvEnvelope ggv(car, config);
    for (const double v : {10.0, 18.0, 25.0}) {
        const double curvature = 0.5 * ggv.lateral_limit(v) / (v * v);  // half the grip sideways
        const double brake = ggv.tire_brake_force(v, curvature);
        const AxleUse use =
            axle_use(car, config.grip_factor, v, car.mass * v * v * curvature, -brake);
        EXPECT_NEAR(use.rear, 1.0, 1e-9) << "v = " << v;
        EXPECT_LT(use.front, 1.0) << "v = " << v;
        // Less than the whole car's ellipse would allow.
        const double whole_car = std::sqrt(1.0 - 0.25) * config.grip_factor * car.front_tire.mu *
                                 (car.mass * kGravity + downforce_coefficient(car) * v * v);
        EXPECT_LT(brake, 0.9 * whole_car) << "v = " << v;
    }
}

// Against the simulator's own tire forces: with the grip factor at 1, the envelope's largest
// braking force on a straight is exactly where the simulated rear axle reaches traction, under
// the normal loads the simulator computes.
TEST(SpeedProfile, BrakingLimitIsTheSimulatorsRearTraction) {
    VehicleParams car;
    car.max_brake_force = 1e6;
    const GgvEnvelope ggv(car, {.grip_factor = 1.0, .max_speed = 100.0});
    for (const double v : {5.0, 15.0, 25.0}) {
        const double brake = ggv.tire_brake_force(v, 0.0);
        sim::VehicleModel model(car);
        VehicleState state;
        state.vx = v;
        state.force = -brake;
        model.reset(state);
        const sim::TireForces& forces = model.tire_forces();
        EXPECT_NEAR(-forces.fx_rear, car.rear_tire.mu * forces.fz_rear, 1e-6 * brake)
            << "v = " << v;
        EXPECT_LT(-forces.fx_front, car.front_tire.mu * forces.fz_front) << "v = " << v;
        EXPECT_NEAR(forces.fx_front + forces.fx_rear, -brake, 1e-6 * brake) << "v = " << v;
    }
}

// Power-limited acceleration without resistance: m v^2 dv = P ds, so v^3 = v0^3 + 3 P s / m.
TEST(SpeedProfile, PowerLimitedAccelerationMatchesClosedForm) {
    VehicleParams car = frictionless_air();
    car.max_drive_force = 1e6;
    const double v0 = 26.0;  // fast enough that P / v is below the tires' limit
    Trajectory path = straight(200.0);
    apply_speed_profile(path, car, uncapped(), v0, 0.0);
    for (const TrajectoryPoint& p : path.points()) {
        if (p.s > 30.0) break;
        const double expected = std::cbrt(v0 * v0 * v0 + 3.0 * car.max_power * p.s / car.mass);
        EXPECT_NEAR(p.speed, expected, 1e-4 * expected) << "s = " << p.s;
    }
}

TEST(SpeedProfile, OpenPathStartsAndEndsAtTheGivenSpeeds) {
    const VehicleParams car;
    Trajectory path = straight(60.0);
    apply_speed_profile(path, car, {}, 5.0, 2.0);
    EXPECT_DOUBLE_EQ(path[0].speed, 5.0);
    EXPECT_DOUBLE_EQ(path.points().back().speed, 2.0);
    EXPECT_GT(path[0].accel, 0.0);
    EXPECT_LT(path.points().back().accel, 0.0);
}

TEST(SpeedProfile, SpeedCapsApply) {
    const VehicleParams car;
    Trajectory path = straight(400.0);
    const SpeedProfileConfig config{.grip_factor = 0.85, .max_speed = 12.0};
    apply_speed_profile(path, car, config, 0.0, 0.0);
    double top = 0.0;
    for (const TrajectoryPoint& p : path.points()) top = std::max(top, p.speed);
    EXPECT_DOUBLE_EQ(top, 12.0);

    VehicleParams geared = car;
    geared.max_speed = 9.0;
    apply_speed_profile(path, geared, config, 0.0, 0.0);
    top = 0.0;
    for (const TrajectoryPoint& p : path.points()) top = std::max(top, p.speed);
    EXPECT_DOUBLE_EQ(top, 9.0);
}

// On a real track shape the profile must stay inside the envelope everywhere, including across
// the closing segment of the periodic profile.
TEST(SpeedProfile, ClosedTrackProfileStaysInsideTheEnvelope) {
    const VehicleParams car;
    const SpeedProfileConfig config;
    const GgvEnvelope ggv(car, config);
    Trajectory path = make_oval_track(50.0, 10.0).centerline;
    apply_speed_profile(path, car, config);
    const std::size_t n = path.size();
    double slowest = 1e9;
    double fastest = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const TrajectoryPoint& p = path[i];
        const TrajectoryPoint& next = path[(i + 1) % n];
        slowest = std::min(slowest, p.speed);
        fastest = std::max(fastest, p.speed);
        EXPECT_LE(p.speed, ggv.cornering_speed(p.curvature) + 1e-9);
        // The step's mean acceleration is within the limits at its two ends.
        const double ds = i + 1 < n ? next.s - p.s : path.length() - p.s;
        const double accel = (next.speed * next.speed - p.speed * p.speed) / (2.0 * ds);
        const double max_accel = std::max(ggv.max_acceleration(p.speed, p.curvature),
                                          ggv.max_acceleration(next.speed, next.curvature));
        const double max_decel = std::max(ggv.max_deceleration(p.speed, p.curvature),
                                          ggv.max_deceleration(next.speed, next.curvature));
        EXPECT_LE(accel, max_accel + 1e-6) << "i = " << i;
        EXPECT_GE(accel, -max_decel - 1e-6) << "i = " << i;
        EXPECT_NEAR(p.accel, accel, 1e-9);
    }
    EXPECT_GT(fastest, slowest + 3.0);  // it does accelerate on the straights
}

TEST(SpeedProfile, ProfileTimeOfAConstantSpeedLap) {
    Trajectory path = circle(20.0);
    const std::vector<double> speed(path.size(), 8.0);
    path.set_speed_profile(speed);
    EXPECT_NEAR(profile_time(path), path.length() / 8.0, 1e-9);

    const std::vector<double> stopped(path.size(), 0.0);
    path.set_speed_profile(stopped);
    EXPECT_TRUE(std::isinf(profile_time(path)));
}

TEST(SpeedProfile, InvalidConfigThrows) {
    const VehicleParams car;
    Trajectory path = straight(10.0);
    EXPECT_THROW(apply_speed_profile(path, car, {.grip_factor = 0.0, .max_speed = 10.0}),
                 std::invalid_argument);
    EXPECT_THROW(apply_speed_profile(path, car, {.grip_factor = 0.8, .max_speed = -1.0}),
                 std::invalid_argument);
    VehicleParams massless;
    massless.mass = 0.0;
    EXPECT_THROW(apply_speed_profile(path, massless, {}), std::invalid_argument);
    Trajectory empty;
    EXPECT_THROW(apply_speed_profile(empty, car, {}), std::invalid_argument);
}

}  // namespace
}  // namespace apex::planning
