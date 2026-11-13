#include "apex/perception/ground.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>

#include "apex/core/angles.hpp"

namespace apex::perception {
namespace {

constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
// Tuning constants for 95% asymptotic efficiency under Gaussian noise (P. W. Holland and
// R. E. Welsch, "Robust regression using iteratively reweighted least-squares", Communications
// in Statistics 6(9), 1977). 1.4826 turns a median absolute deviation into a Gaussian sigma.
constexpr double kHuberK = 1.345;
constexpr double kTukeyC = 4.685;
constexpr double kMadToSigma = 1.4826;

// The median of `values`, which it reorders. `values` must not be empty.
double median_inplace(std::vector<double>& values) {
    const auto mid = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
}

bool finite(const LidarPoint& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

}  // namespace

double GroundPlane::height_of(double x, double y, double z) const {
    return (z - slope_x * x - slope_y * y - offset) /
           std::sqrt(1.0 + slope_x * slope_x + slope_y * slope_y);
}

double GroundPlane::sensor_height() const {
    return height_of(0.0, 0.0, 0.0);
}

double GroundPlane::tilt() const {
    return std::atan(std::hypot(slope_x, slope_y));
}

GroundEstimator::GroundEstimator(const GroundConfig& config, const LidarGeometry& geometry)
    : config_(config) {
    if (!(config.seed_cell_size > 0.0) || !(config.threshold > 0.0) || config.iterations < 1 ||
        !(config.min_scale > 0.0) || !(config.max_tilt_deg > 0.0) || config.min_seeds < 3) {
        throw std::invalid_argument(
            "GroundConfig: cell size, threshold, scale and tilt must be "
            "positive, iterations >= 1, min_seeds >= 3");
    }
    if (!(geometry.max_range > 0.0)) throw std::invalid_argument("LidarGeometry: max_range <= 0");
    cells_per_side_ =
        static_cast<std::size_t>(std::ceil(2.0 * geometry.max_range / config.seed_cell_size));
    extent_ = 0.5 * static_cast<double>(cells_per_side_) * config.seed_cell_size;
    lowest_.assign(cells_per_side_ * cells_per_side_, kNone);
    last_good_.offset = -geometry.mount.z();
}

GroundFit GroundEstimator::fit(std::span<const LidarPoint> points) {
    touched_.clear();
    const double inv_cell = 1.0 / config_.seed_cell_size;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const LidarPoint& p = points[i];
        if (!finite(p)) continue;
        const double gx = std::floor((p.x + extent_) * inv_cell);
        const double gy = std::floor((p.y + extent_) * inv_cell);
        const auto n = static_cast<double>(cells_per_side_);
        if (gx < 0.0 || gy < 0.0 || gx >= n || gy >= n) continue;
        const std::size_t cell =
            static_cast<std::size_t>(gy) * cells_per_side_ + static_cast<std::size_t>(gx);
        std::size_t& lowest = lowest_[cell];
        if (lowest == kNone) {
            touched_.push_back(cell);
            lowest = i;
        } else if (p.z < points[lowest].z) {
            lowest = i;
        }
    }
    seeds_.clear();
    for (const std::size_t cell : touched_) {
        const LidarPoint& p = points[lowest_[cell]];
        seeds_.push_back({p.x, p.y, p.z});
        lowest_[cell] = kNone;
    }

    GroundFit result;
    result.seeds = seeds_.size();
    if (fit_seeds(result)) {
        result.ok = true;
        last_good_ = result.plane;
    } else {
        result.ok = false;
        result.plane = last_good_;
    }
    return result;
}

bool GroundEstimator::fit_seeds(GroundFit& fit) {
    const std::size_t n = seeds_.size();
    if (n < config_.min_seeds) return false;

    scratch_.resize(n);
    for (std::size_t i = 0; i < n; ++i) scratch_[i] = seeds_[i].z;
    GroundPlane plane{0.0, 0.0, median_inplace(scratch_)};

    residuals_.resize(n);
    double scale = config_.min_scale;
    const int huber_iterations = config_.iterations / 2;
    for (int iteration = 0; iteration < config_.iterations; ++iteration) {
        for (std::size_t i = 0; i < n; ++i) {
            const Seed& s = seeds_[i];
            residuals_[i] = s.z - plane.slope_x * s.x - plane.slope_y * s.y - plane.offset;
            scratch_[i] = std::abs(residuals_[i]);
        }
        scale = std::max(config_.min_scale, kMadToSigma * median_inplace(scratch_));
        const bool tukey = iteration >= huber_iterations;

        Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
        Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
        for (std::size_t i = 0; i < n; ++i) {
            const double u = std::abs(residuals_[i]) / scale;
            double w = 0.0;
            if (tukey) {
                const double t = u / kTukeyC;
                w = t < 1.0 ? (1.0 - t * t) * (1.0 - t * t) : 0.0;
            } else {
                w = u <= kHuberK ? 1.0 : kHuberK / u;
            }
            if (w == 0.0) continue;
            const Eigen::Vector3d a(seeds_[i].x, seeds_[i].y, 1.0);
            normal.noalias() += w * a * a.transpose();
            rhs.noalias() += w * seeds_[i].z * a;
        }
        const Eigen::Vector3d solution = normal.ldlt().solve(rhs);
        if (!solution.allFinite()) return false;
        plane = {solution.x(), solution.y(), solution.z()};
    }

    std::size_t inliers = 0;
    for (const Seed& s : seeds_) {
        const double r = s.z - plane.slope_x * s.x - plane.slope_y * s.y - plane.offset;
        if (std::abs(r) < kTukeyC * scale) ++inliers;
    }
    fit.plane = plane;
    fit.inliers = inliers;
    fit.scale = scale;
    return inliers >= config_.min_seeds && plane.tilt() <= deg_to_rad(config_.max_tilt_deg);
}

void extract_obstacles(std::span<const LidarPoint> points, const GroundPlane& plane,
                       double threshold, double max_range, std::vector<ObstaclePoint>& out) {
    out.clear();
    const double norm =
        1.0 / std::sqrt(1.0 + plane.slope_x * plane.slope_x + plane.slope_y * plane.slope_y);
    const double max_range_sq = max_range * max_range;
    for (const LidarPoint& p : points) {
        if (!finite(p)) continue;
        const double x = p.x;
        const double y = p.y;
        const double height = (p.z - plane.slope_x * x - plane.slope_y * y - plane.offset) * norm;
        if (height <= threshold || x * x + y * y > max_range_sq) continue;
        out.push_back({p.x, p.y, static_cast<float>(height), p.truth_id});
    }
}

}  // namespace apex::perception
