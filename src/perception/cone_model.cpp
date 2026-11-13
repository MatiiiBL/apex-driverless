#include "apex/perception/cone_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <Eigen/Dense>

#include "apex/core/angles.hpp"

namespace apex::perception {
namespace {

// Distance from the cone's axis to its surface at height z above the ground.
double section_radius(const ConeShape& shape, double z) {
    return shape.base_radius * std::max(0.0, 1.0 - z / shape.height);
}

// Sum over the members of the squared distance from the return to the surface of a cone of
// `shape` centered at `center` (m^2).
double surface_residual_sq(std::span<const ObstaclePoint> points,
                           std::span<const std::uint32_t> members, const ConeShape& shape,
                           const Vec2& center) {
    double sum = 0.0;
    for (const std::uint32_t i : members) {
        const double distance = (center - Vec2(points[i].x, points[i].y)).norm();
        const double residual = distance - section_radius(shape, points[i].height);
        sum += residual * residual;
    }
    return sum;
}

}  // namespace

double VarianceModel::at(double range, std::size_t points) const {
    const double r2 = range * range;
    const double inv_n = 1.0 / static_cast<double>(std::max<std::size_t>(points, 1));
    return constant + range_sq * r2 + inv_points * inv_n + range_sq_inv_points * r2 * inv_n;
}

Mat2 observation_covariance(const CovarianceModel& model, const Vec2& center, std::size_t points) {
    const double range = center.norm();
    const Vec2 radial = range > 1e-9 ? Vec2(center / range) : Vec2(1.0, 0.0);
    const Vec2 tangential(-radial.y(), radial.x());
    return model.radial.at(range, points) * radial * radial.transpose() +
           model.tangential.at(range, points) * tangential * tangential.transpose();
}

double expected_cone_returns(const LidarGeometry& geometry, const ConeShape& shape, double range,
                             double sensor_height, double min_height) {
    if (!(range > shape.base_radius) || !(geometry.azimuth_step > 0.0)) return 0.0;
    double total = 0.0;
    for (const double elevation : geometry.elevations) {
        const double slope = std::tan(elevation);
        // Height where the channel meets the near face: one fixed-point step from the axis.
        const double z_axis = sensor_height + range * slope;
        const double distance = range - section_radius(shape, z_axis);
        const double z = sensor_height + distance * slope;
        if (z < min_height || z >= shape.height) continue;
        const double slant = std::hypot(distance, z - sensor_height);
        if (slant < geometry.min_range || slant > geometry.max_range) continue;
        const double rho = section_radius(shape, z);
        total += 2.0 * std::asin(std::min(1.0, rho / range)) / geometry.azimuth_step;
    }
    return total;
}

ClusterSummary summarize_cluster(std::span<const ObstaclePoint> points,
                                 std::span<const std::uint32_t> members) {
    ClusterSummary s;
    s.points = members.size();
    if (members.empty()) return s;
    s.min_height = std::numeric_limits<double>::infinity();
    s.max_height = -std::numeric_limits<double>::infinity();
    for (const std::uint32_t i : members) {
        s.centroid += Vec2(points[i].x, points[i].y);
        s.min_height = std::min(s.min_height, static_cast<double>(points[i].height));
        s.max_height = std::max(s.max_height, static_cast<double>(points[i].height));
    }
    s.centroid /= static_cast<double>(members.size());
    s.range = s.centroid.norm();
    const Vec2 radial = s.range > 1e-9 ? Vec2(s.centroid / s.range) : Vec2(1.0, 0.0);
    const Vec2 tangential(-radial.y(), radial.x());
    double t_min = std::numeric_limits<double>::infinity();
    double t_max = -t_min;
    double r_min = t_min;
    double r_max = -t_min;
    for (const std::uint32_t i : members) {
        const Vec2 d = Vec2(points[i].x, points[i].y) - s.centroid;
        t_min = std::min(t_min, d.dot(tangential));
        t_max = std::max(t_max, d.dot(tangential));
        r_min = std::min(r_min, d.dot(radial));
        r_max = std::max(r_max, d.dot(radial));
    }
    s.tangential_extent = t_max - t_min;
    s.radial_extent = r_max - r_min;
    return s;
}

std::string_view to_string(ConeCheck check) {
    switch (check) {
        case ConeCheck::Accepted:
            return "accepted";
        case ConeCheck::TooTall:
            return "too_tall";
        case ConeCheck::TooWide:
            return "too_wide";
        case ConeCheck::TooDeep:
            return "too_deep";
        case ConeCheck::Floating:
            return "floating";
        case ConeCheck::TooFewPoints:
            return "too_few_points";
        case ConeCheck::TooManyPoints:
            return "too_many_points";
    }
    return "accepted";
}

ConeValidator::ConeValidator(const ConeCheckConfig& config, const LidarGeometry& geometry,
                             double ground_threshold)
    : config_(config),
      geometry_(geometry),
      ground_threshold_(ground_threshold),
      small_(cone_shape(ConeColor::Blue)),
      big_(cone_shape(ConeColor::BigOrange)) {
    if (config.min_points < 1 || !(config.min_visible_fraction >= 0.0) ||
        !(config.max_excess_factor >= 1.0) || !(config.height_margin >= 0.0) ||
        !(config.width_margin >= 0.0) || !(config.depth_margin >= 0.0) ||
        !(config.float_margin >= 0.0)) {
        throw std::invalid_argument(
            "ConeCheckConfig: min_points >= 1, max_excess_factor >= 1, margins >= 0");
    }
    if (geometry.elevations.empty()) throw std::invalid_argument("LidarGeometry: no channels");
    std::vector<double> sorted = geometry.elevations;
    std::sort(sorted.begin(), sorted.end());
    for (std::size_t i = 1; i < sorted.size(); ++i) {
        max_channel_gap_ = std::max(max_channel_gap_, sorted[i] - sorted[i - 1]);
    }
}

ConeCheck ConeValidator::check(const ClusterSummary& cluster, double sensor_height) const {
    const double r = cluster.range;
    if (cluster.max_height > big_.height + config_.height_margin) return ConeCheck::TooTall;
    if (cluster.tangential_extent >
        2.0 * big_.base_radius + r * geometry_.azimuth_step + config_.width_margin) {
        return ConeCheck::TooWide;
    }
    if (cluster.radial_extent > big_.base_radius + config_.depth_margin) {
        return ConeCheck::TooDeep;
    }
    if (cluster.min_height >
        ground_threshold_ + r * std::tan(max_channel_gap_) + config_.float_margin) {
        return ConeCheck::Floating;
    }
    const auto n = static_cast<double>(cluster.points);
    const double expected_small =
        expected_cone_returns(geometry_, small_, r, sensor_height, ground_threshold_);
    if (cluster.points < config_.min_points || n < config_.min_visible_fraction * expected_small) {
        return ConeCheck::TooFewPoints;
    }
    const double expected_big =
        expected_cone_returns(geometry_, big_, r, sensor_height, ground_threshold_);
    if (n > config_.max_excess_factor * expected_big + static_cast<double>(config_.min_points)) {
        return ConeCheck::TooManyPoints;
    }
    return ConeCheck::Accepted;
}

ConeFit fit_cone_center(std::span<const ObstaclePoint> points,
                        std::span<const std::uint32_t> members, const ConeShape& shape,
                        const CenterFitConfig& config) {
    if (members.empty()) return {};
    Vec2 centroid = Vec2::Zero();
    double mean_rho = 0.0;
    for (const std::uint32_t i : members) {
        centroid += Vec2(points[i].x, points[i].y);
        mean_rho += section_radius(shape, points[i].height);
    }
    const auto n = static_cast<double>(members.size());
    centroid /= n;
    mean_rho /= n;
    const double range = centroid.norm();
    if (!(range > 1e-6)) return {centroid, surface_residual_sq(points, members, shape, centroid)};
    const Vec2 prior = centroid + (0.25 * kPi * mean_rho / range) * centroid;

    const double w_surface = 1.0 / (config.range_sigma * config.range_sigma);
    const double w_prior = 1.0 / (shape.base_radius * shape.base_radius);
    Vec2 c = prior;
    for (int iteration = 0; iteration < config.iterations; ++iteration) {
        Mat2 normal = w_prior * Mat2::Identity();
        Vec2 gradient = w_prior * (c - prior);
        for (const std::uint32_t i : members) {
            const Vec2 d = c - Vec2(points[i].x, points[i].y);
            const double distance = d.norm();
            if (distance < 1e-9) continue;
            const Vec2 jacobian = d / distance;
            const double residual = distance - section_radius(shape, points[i].height);
            normal.noalias() += w_surface * jacobian * jacobian.transpose();
            gradient += w_surface * residual * jacobian;
        }
        const Vec2 step = -normal.ldlt().solve(gradient);
        if (!step.allFinite()) break;
        c += step;
        if (step.squaredNorm() < 1e-12) break;
    }
    // The fit cannot put the center in front of the visible face; if it did, it diverged.
    if (!c.allFinite() || c.dot(centroid) < range * range) c = prior;
    return {c, surface_residual_sq(points, members, shape, c)};
}

ShapedConeFit fit_cone(std::span<const ObstaclePoint> points,
                       std::span<const std::uint32_t> members, const ClusterSummary& cluster,
                       const CenterFitConfig& config) {
    const ConeShape big = cone_shape(ConeColor::BigOrange);
    if (cluster.max_height > config.big_cone_height) {
        return {fit_cone_center(points, members, big, config), true};
    }
    const ConeFit small_fit = fit_cone_center(points, members, cone_shape(ConeColor::Blue), config);
    const ConeFit big_fit = fit_cone_center(points, members, big, config);
    const double log_likelihood_ratio = (small_fit.residual_sq - big_fit.residual_sq) /
                                        (2.0 * config.range_sigma * config.range_sigma);
    if (log_likelihood_ratio + config.big_cone_log_odds > 0.0) return {big_fit, true};
    return {small_fit, false};
}

std::int32_t majority_truth_id(std::span<const ObstaclePoint> points,
                               std::span<const std::uint32_t> members,
                               std::vector<std::int32_t>& scratch) {
    scratch.clear();
    for (const std::uint32_t i : members) scratch.push_back(points[i].truth_id);
    std::sort(scratch.begin(), scratch.end());
    std::int32_t best = -1;
    std::size_t best_count = 0;
    for (std::size_t i = 0; i < scratch.size();) {
        std::size_t j = i;
        while (j < scratch.size() && scratch[j] == scratch[i]) ++j;
        if (j - i > best_count) {
            best = scratch[i];
            best_count = j - i;
        }
        i = j;
    }
    return best;
}

}  // namespace apex::perception
