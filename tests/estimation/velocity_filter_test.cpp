#include "apex/estimation/velocity_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Cholesky>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "apex/estimation/eval/consistency.hpp"
#include "apex/estimation/eval/synthetic.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::estimation {
namespace {

using eval::DriveProfile;
using eval::SyntheticRun;
using eval::SyntheticSensors;

// Feeds the velocity filter the IMU, wheel and steering messages of a run.
struct FilterSink {
    VelocityFilter& filter;
    void on_imu(const ImuSample& s) { filter.predict(s); }
    void on_wheels(const WheelSpeeds& s) { filter.update_wheels(s); }
    void on_steering(const SteeringSample& s) { filter.update_steering(s); }
    void on_cones(const ConeObservations& /*unused*/) {}
};

struct FilterRun {
    double vx_rms = 0.0;
    double vy_rms = 0.0;
    double yaw_rate_rms = 0.0;
    double vx_max = 0.0;
    double bias_error = 0.0;  // at the end
    double bias_std = 0.0;
    eval::NeesSummary nees;  // of (vx, vy, yaw rate)
    std::size_t wheel_outliers = 0;
};

// Errors from `settle` seconds into the run until `until`. The velocity NEES is sampled every
// 2 s: the error decorrelates within about a second, so the samples are close to independent
// and the chi-square bounds of their mean apply.
FilterRun run_filter(const SyntheticRun& run, const VelocityFilterConfig& config,
                     double settle = 2.0, double until = std::numeric_limits<double>::infinity()) {
    VelocityFilter filter(config, VehicleParams{});
    FilterSink sink{filter};
    FilterRun out;
    eval::NeesAccumulator nees(3);
    double sum[3] = {};
    std::size_t n = 0;
    Time next_nees = settle;
    for (const eval::TimedMessage& m : run.messages) {
        if (m.delivery > until) break;
        eval::deliver(sink, m);
        const auto* imu = std::get_if<ImuSample>(&m.message);
        if (imu == nullptr) continue;
        const eval::TruthSample& t = run.truth_at(imu->stamp);
        out.bias_error = filter.gyro_bias() - t.gyro_bias;
        if (imu->stamp < settle) continue;
        const Vec3 error(filter.vx() - t.vx, filter.vy() - t.vy, filter.yaw_rate() - t.yaw_rate);
        for (int i = 0; i < 3; ++i) sum[i] += error[i] * error[i];
        out.vx_max = std::max(out.vx_max, std::abs(error[0]));
        ++n;
        if (imu->stamp >= next_nees) {
            nees.add(eval::nees(error, filter.velocity_covariance()));
            next_nees += 2.0;
        }
    }
    const auto count = static_cast<double>(std::max<std::size_t>(n, 1));
    out.vx_rms = std::sqrt(sum[0] / count);
    out.vy_rms = std::sqrt(sum[1] / count);
    out.yaw_rate_rms = std::sqrt(sum[2] / count);
    out.bias_std = std::sqrt(filter.covariance()(3, 3));
    out.nees = nees.summary();
    out.wheel_outliers = filter.wheel_outliers();
    return out;
}

SyntheticRun drive(std::uint64_t seed, const DriveProfile& profile) {
    return eval::make_synthetic_run(sim::generate_track(seed), VehicleParams{}, profile,
                                    SyntheticSensors{}, seed);
}

DriveProfile three_laps() {
    DriveProfile profile;
    profile.laps = 3;
    return profile;
}

// Hard acceleration and braking near the tire limit, where the wheels spin and lock: the
// synthetic slip grows without bound as the force approaches the friction limit, far beyond the
// filter's linear slip model.
DriveProfile at_the_limit() {
    DriveProfile profile = three_laps();
    profile.max_accel = 12.0;
    profile.max_brake = 16.0;
    profile.racing_lateral_accel = 18.0;
    return profile;
}

// The filter's noise model matched to the synthetic sensors. The gyro and wheel figures are the
// sensors' own. The accelerometer density is ten times the sensor's (0.05 m/s^2 per 200 Hz
// sample), because it also absorbs the mismatch between the drive's slip and the filter's.
VelocityFilterConfig matched() {
    const SyntheticSensors sensors;
    VelocityFilterConfig config;
    config.gyro_noise = sensors.gyro_noise;
    config.gyro_bias_walk = sensors.gyro_bias_walk;
    config.wheel_noise = sensors.wheel_noise;
    config.accel_noise_density = 0.035;
    return config;
}

TEST(VelocityFilter, CalibratesTheGyroBiasAtRest) {
    // At rest the lateral model pins the yaw rate to zero, so the gyro reads its bias.
    DriveProfile profile;
    profile.hold_at_start = 4.0;
    const SyntheticRun run = drive(1, profile);
    const VelocityFilterConfig config;
    const FilterRun result = run_filter(run, config, 0.5, 3.9);
    EXPECT_LT(result.bias_std, 0.2 * config.initial_gyro_bias_std);
    EXPECT_LT(std::abs(result.bias_error), 3.0 * result.bias_std);
    EXPECT_LT(std::abs(result.bias_error), 0.25 * SyntheticSensors{}.gyro_bias);
    EXPECT_LT(result.vx_max, 0.05);  // the wheel-speed noise is 0.03 m/s
}

TEST(VelocityFilter, TracksTheVelocityOnGeneratedTracks) {
    for (const std::uint64_t seed : {1U, 2U, 3U}) {
        const FilterRun result = run_filter(drive(seed, three_laps()), {});
        EXPECT_LT(result.vx_rms, 0.06) << seed;
        EXPECT_LT(result.vx_max, 0.4) << seed;
        EXPECT_LT(result.vy_rms, 0.07) << seed;
        EXPECT_LT(result.yaw_rate_rms, 0.003) << seed;
        // The bias drifts during the run and is tracked within its uncertainty.
        EXPECT_LT(std::abs(result.bias_error), 3.0 * result.bias_std) << seed;
    }
}

TEST(VelocityFilter, VelocityEstimateIsConsistent) {
    for (const std::uint64_t seed : {1U, 2U, 3U}) {
        const FilterRun result = run_filter(drive(seed, three_laps()), matched());
        ASSERT_GT(result.nees.samples, 25U);
        const eval::NeesBounds bounds = eval::nees_bounds(3, result.nees.samples);
        EXPECT_GT(result.nees.mean, bounds.lower) << seed;
        EXPECT_LT(result.nees.mean, bounds.upper) << seed;
    }
}

TEST(VelocityFilter, DownWeightsSlippingWheels) {
    const SyntheticRun run = drive(1, at_the_limit());
    const FilterRun robust = run_filter(run, {});
    VelocityFilterConfig trusting;
    trusting.wheel_robust_threshold = 1e9;  // every wheel sample at full weight
    const FilterRun naive = run_filter(run, trusting);
    EXPECT_GT(robust.wheel_outliers, 100U);
    EXPECT_EQ(naive.wheel_outliers, 0U);
    EXPECT_LT(robust.vx_rms, 0.12);
    EXPECT_LT(robust.vx_rms, 0.95 * naive.vx_rms);
    EXPECT_LT(robust.vx_max, 0.95 * naive.vx_max);
}

TEST(VelocityFilter, RejectsInvalidSamples) {
    VelocityFilter filter({}, VehicleParams{});
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(filter.predict({0.010, 1.0, 0.0, 0.01}));
    const VelocityFilter::Vec4 state = filter.state();
    const VelocityFilter::Mat4 covariance = filter.covariance();
    EXPECT_FALSE(filter.predict({0.010, 1.0, 0.0, 0.01}));  // not newer
    EXPECT_FALSE(filter.predict({0.005, 1.0, 0.0, 0.01}));  // older
    EXPECT_FALSE(filter.predict({0.015, nan, 0.0, 0.01}));
    EXPECT_FALSE(filter.update_wheels({0.015, nan, 0.0}));
    EXPECT_FALSE(filter.update_wheels({nan, 0.0, 0.0}));
    EXPECT_FALSE(filter.update_steering({0.015, std::numeric_limits<double>::infinity()}));
    EXPECT_EQ(filter.state(), state);
    EXPECT_EQ(filter.covariance(), covariance);
    EXPECT_EQ(filter.stamp(), 0.010);
}

TEST(VelocityFilter, CovarianceStaysSymmetricPositiveDefinite) {
    VelocityFilter filter({}, VehicleParams{});
    FilterSink sink{filter};
    const SyntheticRun run = drive(2, at_the_limit());
    std::size_t checked = 0;
    for (const eval::TimedMessage& m : run.messages) {
        eval::deliver(sink, m);
        if (std::holds_alternative<WheelSpeeds>(m.message) && ++checked % 50 == 0) {
            const VelocityFilter::Mat4& p = filter.covariance();
            ASSERT_LT((p - p.transpose()).cwiseAbs().maxCoeff(), 1e-12);
            ASSERT_EQ(Eigen::LLT<VelocityFilter::Mat4>(p).info(), Eigen::Success);
        }
    }
    EXPECT_GT(checked, 1000U);
}

TEST(VelocityFilter, ConfigRoundTripsThroughJson) {
    VelocityFilterConfig config;
    config.wheel_noise = 0.07;
    config.slip_correlation_time = 0.4;
    const nlohmann::json j = config;
    const auto back = j.get<VelocityFilterConfig>();
    EXPECT_EQ(back.wheel_noise, 0.07);
    EXPECT_EQ(back.slip_correlation_time, 0.4);
    EXPECT_EQ(back.gyro_noise, VelocityFilterConfig{}.gyro_noise);
    // Missing keys keep their defaults; unknown ones are rejected.
    const auto partial = nlohmann::json{{"gyro_noise", 0.01}}.get<VelocityFilterConfig>();
    EXPECT_EQ(partial.gyro_noise, 0.01);
    EXPECT_EQ(partial.wheel_noise, VelocityFilterConfig{}.wheel_noise);
    EXPECT_THROW((void)nlohmann::json({{"gyro_nosie", 0.01}}).get<VelocityFilterConfig>(),
                 std::invalid_argument);
}

TEST(VelocityFilter, RejectsAnInvalidConfig) {
    VelocityFilterConfig config;
    config.wheel_noise = 0.0;
    EXPECT_THROW(VelocityFilter(config, VehicleParams{}), std::invalid_argument);
    config = {};
    config.slip_correlation_time = -1.0;
    EXPECT_THROW(VelocityFilter(config, VehicleParams{}), std::invalid_argument);
    VehicleParams massless;
    massless.mass = 0.0;
    EXPECT_THROW(VelocityFilter({}, massless), std::invalid_argument);
}

}  // namespace
}  // namespace apex::estimation
