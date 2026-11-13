#pragma once

#include <cstddef>
#include <span>

#include "apex/perception/cone_model.hpp"

// Offline calibration of the covariance model on simulated scans (bench/perception/,
// docs/perception.md, "Covariance"). Not used at run time.
namespace apex::perception {

// One coordinate (radial or tangential) of the error of a cone center estimated from `points`
// returns at horizontal range `range` from the LiDAR.
struct VarianceSample {
    double range = 0.0;  // m
    std::size_t points = 0;
    double error = 0.0;  // m
};

// The VarianceModel that best explains the errors as zero-mean Gaussians: the maximum-likelihood
// fit of e ~ N(0, v(r, n)) with v linear in the four non-negative coefficients.
//
// Iteratively reweighted least squares on the squared errors with weights 1 / v^2 (each e^2 has
// variance 2 v^2), whose fixed point is the likelihood's stationary point, for each subset of
// the three range and count terms (the constant is always kept); the best subset with no
// negative coefficient wins. At the optimum the mean of e^2 / v is 1, the share of one
// coordinate in an average NEES of 2; a final rescale makes it exact after finite iterations.
//   P. McCullagh and J. A. Nelder, "Generalized Linear Models", 2nd ed., Chapman & Hall, 1989
//   (Gamma family with identity link).
// Throws std::invalid_argument for fewer than 4 samples, a non-finite sample, or errors that are
// all zero.
[[nodiscard]] VarianceModel fit_variance_model(std::span<const VarianceSample> samples);

}  // namespace apex::perception
