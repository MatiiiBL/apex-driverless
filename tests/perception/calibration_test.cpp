#include "apex/perception/calibration.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/rng.hpp"

namespace apex::perception {
namespace {

std::vector<VarianceSample> draw(const VarianceModel& truth, std::size_t count, Rng& rng) {
    std::vector<VarianceSample> samples;
    for (std::size_t i = 0; i < count; ++i) {
        VarianceSample s;
        s.range = rng.uniform(2.0, 35.0);
        s.points = 3 + rng.index(300);
        s.error = rng.normal(0.0, std::sqrt(truth.at(s.range, s.points)));
        samples.push_back(s);
    }
    return samples;
}

double mean_ratio(const VarianceModel& model, const std::vector<VarianceSample>& samples) {
    double sum = 0.0;
    for (const VarianceSample& s : samples) sum += s.error * s.error / model.at(s.range, s.points);
    return sum / static_cast<double>(samples.size());
}

TEST(FitVarianceModel, RecoversTheModelBehindGaussianErrors) {
    const VarianceModel truth{1.0e-4, 2.0e-7, 5.0e-3, 0.0};
    Rng rng(17);
    const std::vector<VarianceSample> samples = draw(truth, 20000, rng);
    const VarianceModel fit = fit_variance_model(samples);
    EXPECT_NEAR(fit.constant, truth.constant, 0.15 * truth.constant);
    EXPECT_NEAR(fit.range_sq, truth.range_sq, 0.15 * truth.range_sq);
    EXPECT_NEAR(fit.inv_points, truth.inv_points, 0.15 * truth.inv_points);
    EXPECT_GE(fit.range_sq_inv_points, 0.0);
    EXPECT_LT(fit.range_sq_inv_points, 2e-6);
    // Calibrated: the normalized squared error averages exactly 1 per coordinate.
    EXPECT_NEAR(mean_ratio(fit, samples), 1.0, 1e-9);
}

TEST(FitVarianceModel, NeverReturnsNegativeTerms) {
    // Errors that shrink with range: the r^2 terms would come out negative, so they are dropped.
    Rng rng(18);
    std::vector<VarianceSample> samples;
    for (int i = 0; i < 4000; ++i) {
        VarianceSample s;
        s.range = rng.uniform(2.0, 35.0);
        s.points = 10;
        s.error = rng.normal(0.0, 0.1 / s.range);
        samples.push_back(s);
    }
    const VarianceModel fit = fit_variance_model(samples);
    EXPECT_GT(fit.constant, 0.0);
    EXPECT_GE(fit.range_sq, 0.0);
    EXPECT_GE(fit.inv_points, 0.0);
    EXPECT_GE(fit.range_sq_inv_points, 0.0);
    EXPECT_NEAR(mean_ratio(fit, samples), 1.0, 1e-9);
}

TEST(FitVarianceModel, RejectsUnusableSamples) {
    EXPECT_THROW((void)fit_variance_model(std::vector<VarianceSample>(3, {5.0, 10, 0.01})),
                 std::invalid_argument);
    EXPECT_THROW((void)fit_variance_model(std::vector<VarianceSample>(10, {5.0, 10, 0.0})),
                 std::invalid_argument);
    std::vector<VarianceSample> samples(10, {5.0, 10, 0.01});
    samples[3].error = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW((void)fit_variance_model(samples), std::invalid_argument);
}

}  // namespace
}  // namespace apex::perception
