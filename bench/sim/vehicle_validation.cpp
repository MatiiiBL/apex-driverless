// The vehicle model against its reference solutions, with the errors as numbers for
// docs/sim.md ("Validation"). The tests in tests/sim/vehicle_model_test.cpp assert the same
// comparisons against thresholds; this reports how far inside them the model is.
//
//   ./build/release/bench/sim/apex_sim_validation --out results/sim/vehicle_validation.json
#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

#include "apex/core/angles.hpp"
#include "apex/sim/vehicle_model.hpp"

namespace {

using apex::VehicleParams;
using apex::VehicleState;
using apex::sim::VehicleModel;
using nlohmann::json;

constexpr double kControlPeriod = 0.02;

double resistance(const VehicleParams& p, double v) {
    return p.drag(v) + p.rolling_resistance * (p.mass * apex::kGravity + p.downforce(v));
}

VehicleState moving(double vx, double steer = 0.0) {
    VehicleState state;
    state.vx = vx;
    state.steer = steer;
    return state;
}

// Constant speed (PI over a resistance feed-forward) with a steering schedule.
void hold(VehicleModel& model, double speed, double seconds,
          const std::function<double(double)>& steer_at,
          const std::function<void(const VehicleModel&)>& on_step = {}) {
    double integral = 0.0;
    const auto periods = static_cast<int>(std::lround(seconds / kControlPeriod));
    for (int k = 0; k < periods; ++k) {
        const VehicleParams& p = model.params();
        const double error = speed - model.state().vx;
        integral += error * kControlPeriod;
        model.command({model.time(), steer_at(model.time()),
                       resistance(p, model.state().vx) / p.mass + 3.0 * error + 2.0 * integral});
        for (int i = 0; i < 20; ++i) {
            model.step();
            if (on_step) on_step(model);
        }
    }
}

json kinematic_limit() {
    const VehicleParams p;
    double worst = 0.0;
    for (const double speed : {1.0, 2.0}) {
        for (const double steer : {0.1, 0.2, 0.3, 0.4}) {
            VehicleModel model{p};
            model.reset(moving(speed, steer));
            hold(model, speed, 5.0, [steer](double) { return steer; });
            const VehicleState& s = model.state();
            const double beta = std::atan(p.lr * std::tan(steer) / p.wheelbase());
            const double expected = std::tan(steer) * std::cos(beta) / p.wheelbase();
            worst = std::max(worst, std::abs(s.yaw_rate / std::hypot(s.vx, s.vy) / expected - 1.0));
        }
    }
    return {{"speeds_mps", {1.0, 2.0}},
            {"steer_rad", {0.1, 0.2, 0.3, 0.4}},
            {"max_curvature_error_pct", 100.0 * worst}};
}

json linear_cornering() {
    VehicleParams neutral;
    neutral.drag_area = 0.0;
    neutral.rolling_resistance = 0.0;
    VehicleParams understeer = neutral;
    understeer.front_tire.B = 6.0;
    VehicleParams oversteer = neutral;
    oversteer.rear_tire.B = 8.0;
    json rows = json::array();
    double worst = 0.0;
    for (const auto& [name, p] :
         {std::pair{"default", neutral}, std::pair{"understeer", understeer},
          std::pair{"oversteer", oversteer}}) {
        for (const double speed : {8.0, 12.0, 16.0, 20.0}) {
            const double L = p.wheelbase();
            const double downforce = p.downforce(speed);
            const double fzf =
                p.mass * apex::kGravity * p.lr / L + p.aero_balance_front * downforce;
            const double fzr =
                p.mass * apex::kGravity * p.lf / L + (1.0 - p.aero_balance_front) * downforce;
            const double cf = p.front_tire.B * p.front_tire.C * p.front_tire.mu * fzf;
            const double cr = p.rear_tire.B * p.rear_tire.C * p.rear_tire.mu * fzr;
            const double gradient = p.mass / L * (p.lr / cf - p.lf / cr);
            const double steer = (L + gradient * speed * speed) * 2.0 / (speed * speed);
            const double expected = speed * steer / (L + gradient * speed * speed);
            VehicleModel model{p};
            model.reset(moving(speed, steer));
            hold(model, speed, 4.0, [steer](double) { return steer; });
            const double error = model.state().yaw_rate / expected - 1.0;
            worst = std::max(worst, std::abs(error));
            rows.push_back({{"car", name},
                            {"speed_mps", speed},
                            {"understeer_gradient_rad_per_mps2", gradient},
                            {"yaw_rate_linear", expected},
                            {"yaw_rate_model", model.state().yaw_rate},
                            {"error_pct", 100.0 * error}});
        }
    }
    return {{"lateral_accel_mps2", 2.0}, {"cases", rows}, {"max_error_pct", 100.0 * worst}};
}

json coast_down() {
    const VehicleParams p;
    const double a = p.rolling_resistance * p.mass * apex::kGravity;
    const double b = 0.5 * p.air_density * (p.drag_area + p.rolling_resistance * p.downforce_area);
    const double v0 = 25.0;
    const double theta0 = std::atan(v0 * std::sqrt(b / a));
    const double c = std::sqrt(a * b) / p.mass;
    VehicleModel model{p};
    model.reset(moving(v0));
    double worst_v = 0.0;
    double worst_x = 0.0;
    for (int second = 1; second <= 40; ++second) {
        for (int i = 0; i < 1000; ++i) model.step();
        const double theta = theta0 - c * second;
        const double v = std::sqrt(a / b) * std::tan(theta);
        const double x = p.mass / b * std::log(std::cos(theta) / std::cos(theta0));
        worst_v = std::max(worst_v, std::abs(model.state().vx - v));
        worst_x = std::max(worst_x, std::abs(model.state().pose.x - x));
    }
    return {{"v0_mps", v0},
            {"seconds", 40},
            {"max_speed_error_mps", worst_v},
            {"max_distance_error_m", worst_x}};
}

json power_limit() {
    VehicleParams p;
    p.max_power = 40e3;
    VehicleModel model{p};
    model.reset(moving(14.0));
    double worst = 0.0;
    while (model.state().vx < 26.0) {
        model.command({model.time(), 0.0, 20.0});
        for (int i = 0; i < 20; ++i) {
            model.step();
            if (model.time() < 0.3) continue;
            const double v = model.state().vx;
            const double expected = (p.max_power / v - resistance(p, v)) / p.mass;
            worst = std::max(worst, std::abs(model.state().ax - expected));
        }
    }
    return {{"max_power_w", p.max_power},
            {"speed_range_mps", {14.0, 26.0}},
            {"max_accel_error_mps2", worst}};
}

json lateral_limit() {
    const VehicleParams p;
    json rows = json::array();
    for (const double speed : {10.0, 15.0, 20.0}) {
        for (const double friction : {1.0, 0.5}) {
            VehicleModel model{p, friction};
            model.reset(moving(speed));
            double best = 0.0;
            double best_ratio = 0.0;
            hold(
                model, speed, 8.0, [](double t) { return 0.05 * t; },
                [&](const VehicleModel& m) {
                    const double limit = p.front_tire.mu * friction *
                                         (apex::kGravity + p.downforce(m.state().vx) / p.mass);
                    if (m.state().ay > best) {
                        best = m.state().ay;
                        best_ratio = m.state().ay / limit;
                    }
                });
            rows.push_back({{"speed_mps", speed},
                            {"friction_scale", friction},
                            {"max_lateral_accel_mps2", best},
                            {"fraction_of_mu_g_plus_downforce", best_ratio}});
        }
    }
    return rows;
}

json step_convergence() {
    auto run = [](double dt) {
        VehicleModel model{VehicleParams{}};
        model.reset(moving(10.0));
        const auto steps = static_cast<int>(std::lround(kControlPeriod / dt));
        for (int k = 0; k < 250; ++k) {
            const double t = k * kControlPeriod;
            model.command({t, 0.05 * std::sin(apex::kPi * t), 1.0 + 0.5 * std::cos(2.0 * t)});
            for (int i = 0; i < steps; ++i) model.step(dt);
        }
        const VehicleState& s = model.state();
        return Eigen::Matrix<double, 6, 1>(s.pose.x, s.pose.y, s.pose.yaw, s.vx, s.vy, s.yaw_rate);
    };
    const auto h = run(1e-3);
    const auto h2 = run(5e-4);
    const auto h4 = run(2.5e-4);
    const double change_1ms = (h - h2).norm();
    const double change_05ms = (h2 - h4).norm();
    return {{"manoeuvre_s", 5.0},
            {"change_1ms_to_0.5ms", change_1ms},
            {"change_0.5ms_to_0.25ms", change_05ms},
            {"observed_order", std::log2(change_1ms / change_05ms)}};
}

}  // namespace

int main(int argc, char** argv) {
    std::string out;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--out" && i + 1 < argc) {
            out = argv[++i];
        } else {
            std::cerr << "usage: apex_sim_validation [--out results.json]\n";
            return 2;
        }
    }
    json results;
    results["kinematic_limit"] = kinematic_limit();
    results["linear_cornering"] = linear_cornering();
    results["coast_down"] = coast_down();
    results["power_limit"] = power_limit();
    results["lateral_limit"] = lateral_limit();
    results["step_convergence"] = step_convergence();
    std::cout << results.dump(2) << "\n";
    if (!out.empty()) {
        std::ofstream file(out);
        if (!file) {
            std::cerr << "cannot write " << out << "\n";
            return 1;
        }
        file << results.dump(2) << "\n";
    }
    return 0;
}
