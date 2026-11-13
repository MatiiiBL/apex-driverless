#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/geometry.hpp"
#include "apex/core/rng.hpp"
#include "apex/sim/sensors.hpp"
#include "apex/sim/vehicle_model.hpp"

namespace apex::sim {
namespace {

struct Moments {
    double mean = 0.0;
    double stddev = 0.0;
};
Moments moments(const std::vector<double>& values) {
    double sum = 0.0;
    for (const double v : values) sum += v;
    const double mean = sum / static_cast<double>(values.size());
    double squares = 0.0;
    for (const double v : values) squares += (v - mean) * (v - mean);
    return {mean, std::sqrt(squares / static_cast<double>(values.size() - 1))};
}

TEST(SensorNoise, ScaleMultipliesEveryStandardDeviationAndBias) {
    const SensorNoise base;
    const SensorNoise same = scale_noise(base, 1.0);
    const SensorNoise doubled = scale_noise(base, 2.0);
    const SensorNoise none = scale_noise(base, 0.0);
    auto fields = [](const SensorNoise& n) {
        return std::vector<double>{n.lidar.range_stddev,
                                   n.camera.bearing_stddev,
                                   n.camera.range_stddev,
                                   n.camera.range_stddev_quadratic,
                                   n.camera.confidence_stddev,
                                   n.imu.accel_stddev,
                                   n.imu.gyro_stddev,
                                   n.imu.accel_bias,
                                   n.imu.gyro_bias,
                                   n.imu.accel_bias_walk,
                                   n.imu.gyro_bias_walk,
                                   n.wheels.stddev,
                                   n.steering.stddev};
    };
    // Probabilities, rates and physical properties: unchanged at any factor.
    auto unscaled = [](const SensorNoise& n) {
        return std::vector<double>{n.lidar.dropout,
                                   n.camera.min_range,
                                   n.camera.detection_probability,
                                   n.camera.detection_half_range,
                                   n.camera.detection_falloff,
                                   n.camera.confusion_near,
                                   n.camera.confusion_far,
                                   n.camera.false_positives,
                                   n.wheels.slip_stiffness};
    };
    EXPECT_EQ(unscaled(doubled), unscaled(base));
    EXPECT_EQ(unscaled(none), unscaled(base));
    const std::vector<double> b = fields(base);
    const std::vector<double> s = fields(same);
    const std::vector<double> d = fields(doubled);
    const std::vector<double> z = fields(none);
    for (std::size_t i = 0; i < b.size(); ++i) {
        SCOPED_TRACE(i);
        EXPECT_GT(b[i], 0.0);
        EXPECT_EQ(s[i], b[i]);
        EXPECT_EQ(d[i], 2.0 * b[i]);
        EXPECT_EQ(z[i], 0.0);
    }
    EXPECT_THROW((void)scale_noise(base, -0.5), std::invalid_argument);
    EXPECT_THROW((void)scale_noise(base, std::numeric_limits<double>::quiet_NaN()),
                 std::invalid_argument);
    EXPECT_THROW((void)scale_noise(base, std::numeric_limits<double>::infinity()),
                 std::invalid_argument);
}

TEST(SensorNoise, ValidationNamesTheBadField) {
    EXPECT_NO_THROW(validate_noise(SensorNoise{}));
    SensorNoise noise;
    noise.imu.gyro_bias = -1.0;
    try {
        validate_noise(noise);
        FAIL() << "accepted a negative bias";
    } catch (const std::invalid_argument& e) {
        EXPECT_NE(std::string(e.what()).find("imu.gyro_bias"), std::string::npos);
    }
    noise = {};
    noise.wheels.slip_stiffness = 0.0;
    EXPECT_THROW(validate_noise(noise), std::invalid_argument);
}

// Without noise, the IMU reads the CoG's acceleration expressed in the body frame, which a
// central difference of the world-frame velocity confirms: launch from rest (the kinematic
// regime), then weaving at speed.
TEST(Imu, ReadsTheBodyFrameAccelerationOfTheTruth) {
    VehicleModel model{VehicleParams{}};
    const ImuNoise silent = scale_noise(SensorNoise{}, 0.0).imu;
    ImuBias bias;
    Rng rng(1);
    std::vector<VehicleState> states{model.state()};
    for (int k = 0; k < 300; ++k) {
        const double t = model.time();
        model.command({t, t < 2.0 ? 0.0 : 0.15 * std::sin(3.0 * t), t < 4.0 ? 5.0 : 0.0});
        for (int i = 0; i < 20; ++i) {
            model.step();
            states.push_back(model.state());
        }
    }
    auto world_velocity = [](const VehicleState& s) {
        const Vec2 body(s.vx, s.vy);
        return Vec2(std::cos(s.pose.yaw) * body.x() - std::sin(s.pose.yaw) * body.y(),
                    std::sin(s.pose.yaw) * body.x() + std::cos(s.pose.yaw) * body.y());
    };
    double worst = 0.0;
    std::size_t worst_step = 0;
    for (std::size_t i = 1; i + 1 < states.size(); ++i) {
        const VehicleState& s = states[i];
        const ImuSample sample = simulate_imu(silent, s, 0.0, 0.005, bias, rng);
        ASSERT_EQ(sample.ax, s.ax);
        ASSERT_EQ(sample.ay, s.ay);
        ASSERT_EQ(sample.yaw_rate, s.yaw_rate);
        // Actuator targets change at the start of steps on the 10 ms grid, and the car leaves
        // rest at one instant: the jerk jumps there, and a central difference straddling a
        // jump is off by h * jump / 4. Everywhere else the acceleration is smooth.
        if (i % 10 == 0 || states[i - 1].vx == 0.0) continue;
        const Vec2 world = (world_velocity(states[i + 1]) - world_velocity(states[i - 1])) /
                           (2.0 * VehicleModel::kStep);
        const Vec2 body(std::cos(s.pose.yaw) * world.x() + std::sin(s.pose.yaw) * world.y(),
                        -std::sin(s.pose.yaw) * world.x() + std::cos(s.pose.yaw) * world.y());
        const double error = (body - Vec2(s.ax, s.ay)).norm();
        if (error > worst) {
            worst = error;
            worst_step = i;
        }
    }
    EXPECT_LT(worst, 0.005) << "at t = " << static_cast<double>(worst_step) * 1e-3 << " s, vx "
                            << states[worst_step].vx << ", ax " << states[worst_step].ax;
    EXPECT_GT(states.back().vx, 10.0);
}

// White noise with the configured spread around the truth plus bias; the bias starts with the
// configured spread and wanders as a random walk.
TEST(Imu, NoiseAndBiasHaveTheirConfiguredSpread) {
    const ImuNoise noise;
    Rng rng(7);
    std::vector<double> turn_on;
    for (int i = 0; i < 4000; ++i) turn_on.push_back(draw_imu_bias(noise, rng).yaw_rate);
    EXPECT_NEAR(moments(turn_on).mean, 0.0, 4.0 * noise.gyro_bias / std::sqrt(4000.0));
    EXPECT_NEAR(moments(turn_on).stddev, noise.gyro_bias, 0.05 * noise.gyro_bias);

    VehicleState still;
    still.yaw_rate = 0.3;
    ImuNoise frozen = noise;
    frozen.gyro_bias_walk = 0.0;
    ImuBias bias{0.0, 0.0, 0.01};
    std::vector<double> rates;
    for (int i = 0; i < 4000; ++i) {
        rates.push_back(simulate_imu(frozen, still, 0.0, 0.005, bias, rng).yaw_rate);
    }
    EXPECT_NEAR(moments(rates).mean, 0.31, 4.0 * noise.gyro_stddev / std::sqrt(4000.0));
    EXPECT_NEAR(moments(rates).stddev, noise.gyro_stddev, 0.05 * noise.gyro_stddev);

    // After 100 s, the walk has spread the bias by walk * sqrt(100 s), whatever the sample period.
    std::vector<double> drift;
    for (int run = 0; run < 400; ++run) {
        ImuBias walking;
        for (int i = 0; i < 200; ++i) (void)simulate_imu(noise, still, 0.0, 0.5, walking, rng);
        drift.push_back(walking.yaw_rate);
    }
    EXPECT_NEAR(moments(drift).stddev, noise.gyro_bias_walk * 10.0,
                0.12 * noise.gyro_bias_walk * 10.0);
}

// The wheels turn at the axle's ground speed along them, faster when driving, slower when
// braking, by Fx / (slip_stiffness Fz).
TEST(WheelSpeeds, FollowGroundSpeedPlusSlip) {
    const VehicleParams params;
    const WheelNoise silent = scale_noise(SensorNoise{}, 0.0).wheels;
    Rng rng(3);
    VehicleModel model{params};
    VehicleState cruising;
    cruising.vx = 10.0;
    model.reset(cruising);
    WheelSpeeds w =
        simulate_wheel_speeds(params, silent, model.state(), model.tire_forces(), 0.0, rng);
    EXPECT_EQ(w.front, 10.0);
    EXPECT_EQ(w.rear, 10.0);

    auto settle = [&](double accel) {
        for (int k = 0; k < 25; ++k) {
            model.command({model.time(), 0.0, accel});
            for (int i = 0; i < 20; ++i) model.step();
        }
    };
    settle(8.0);
    const TireForces drive = model.tire_forces();
    w = simulate_wheel_speeds(params, silent, model.state(), drive, 0.0, rng);
    const double vx = model.state().vx;
    EXPECT_NEAR(w.rear, vx * (1.0 + drive.fx_rear / (silent.slip_stiffness * drive.fz_rear)),
                1e-12);
    EXPECT_GT(w.front, vx * 1.01);
    EXPECT_GT(w.rear, vx * 1.01);
    settle(-12.0);
    w = simulate_wheel_speeds(params, silent, model.state(), model.tire_forces(), 0.0, rng);
    EXPECT_LT(w.front, model.state().vx * 0.97);
    EXPECT_LT(w.rear, model.state().vx * 0.99);

    // Cornering at constant speed: the front wheels roll along their heading.
    VehicleState turning;
    turning.vx = 8.0;
    turning.vy = 0.3;
    turning.yaw_rate = 0.5;
    turning.steer = 0.2;
    const TireForces rolling{};
    w = simulate_wheel_speeds(params, silent, turning, rolling, 0.0, rng);
    EXPECT_NEAR(w.front, 8.0 * std::cos(0.2) + (0.3 + params.lf * 0.5) * std::sin(0.2), 1e-12);
    EXPECT_EQ(w.rear, 8.0);
}

TEST(WheelSpeeds, NoisyButNeverNegative) {
    const VehicleParams params;
    const WheelNoise noise;
    Rng rng(5);
    VehicleState cruising;
    cruising.vx = 12.0;
    const VehicleState at_rest;
    std::vector<double> readings;
    for (int i = 0; i < 4000; ++i) {
        readings.push_back(simulate_wheel_speeds(params, noise, cruising, {}, 0.0, rng).rear);
        const WheelSpeeds still = simulate_wheel_speeds(params, noise, at_rest, {}, 0.0, rng);
        ASSERT_GE(still.front, 0.0);
        ASSERT_GE(still.rear, 0.0);
    }
    EXPECT_NEAR(moments(readings).mean, 12.0, 4.0 * noise.stddev / std::sqrt(4000.0));
    EXPECT_NEAR(moments(readings).stddev, noise.stddev, 0.05 * noise.stddev);
}

TEST(Steering, MeasuresTheWheelAngleWithNoise) {
    const SteeringNoise noise;
    Rng rng(11);
    VehicleState state;
    state.steer = -0.25;
    std::vector<double> readings;
    for (int i = 0; i < 4000; ++i) {
        const SteeringSample sample = simulate_steering(noise, state, 1.5, rng);
        ASSERT_EQ(sample.stamp, 1.5);
        readings.push_back(sample.angle);
    }
    EXPECT_NEAR(moments(readings).mean, -0.25, 4.0 * noise.stddev / std::sqrt(4000.0));
    EXPECT_NEAR(moments(readings).stddev, noise.stddev, 0.05 * noise.stddev);
}

}  // namespace
}  // namespace apex::sim
