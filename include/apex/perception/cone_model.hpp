#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/sensor_rig.hpp"
#include "apex/core/types.hpp"
#include "apex/perception/ground.hpp"

// What a cone looks like to the LiDAR: the returns it should give at a range, the checks that
// tell a cone's cluster from anything else, the center estimate from its visible face, and the
// covariance of that estimate. See docs/perception.md, "Cone validation" and "Center".
namespace apex::perception {

struct ConeCheckConfig {
    // No cone from fewer returns, whatever the range. Two returns are as likely to be a chance
    // pair of clutter points as a cone; three limit small cones to about 25 m (docs/perception.md).
    std::size_t min_points = 3;
    // A cluster needs at least this fraction of the returns an unoccluded small cone would give
    // at its range: lower admits partly occluded cones, higher rejects sparse noise.
    double min_visible_fraction = 0.3;
    // At most this multiple of the returns of an unoccluded big cone at that range (plus
    // min_points): more means a merge or a larger object.
    double max_excess_factor = 2.0;
    double height_margin = 0.08;  // m above the big cone's height before a cluster is too tall
    // m beyond the big cone's diameter (plus one azimuth step at the cluster's range) before it
    // is too wide across the line of sight
    double width_margin = 0.12;
    double depth_margin = 0.12;  // m beyond the big cone's radius along the line of sight
    // A cone stands on the ground: its lowest return must be within one channel spacing (at its
    // range) of the ground threshold, plus this margin (m).
    double float_margin = 0.10;
};

void to_json(nlohmann::json& j, const ConeCheckConfig& config);
void from_json(const nlohmann::json& j, ConeCheckConfig& config);

struct CenterFitConfig {
    double range_sigma = 0.02;  // m, LiDAR range noise: weights the surface residuals
    int iterations = 6;         // Gauss-Newton iterations
    // A return higher than this (m) can only come from a big cone (a small one is 0.325 m tall).
    double big_cone_height = 0.38;
    // Log prior odds of a big cone against a small one: a trackdrive has 4 big cones among about
    // 150, ln(4 / 146) = -3.6.
    double big_cone_log_odds = -3.6;
};

void to_json(nlohmann::json& j, const CenterFitConfig& config);
void from_json(const nlohmann::json& j, CenterFitConfig& config);

// Variance of one coordinate of a cone center estimate (m^2), as a function of the horizontal
// range r (m) from the LiDAR and the number of returns n:
//   constant + range_sq r^2 + inv_points / n + range_sq_inv_points r^2 / n.
// The terms: a floor for model error, the angular resolution's growth with range, range noise
// averaged over n returns, and angular quantization averaged over n returns.
struct VarianceModel {
    double constant = 0.0;
    double range_sq = 0.0;
    double inv_points = 0.0;
    double range_sq_inv_points = 0.0;

    [[nodiscard]] double at(double range, std::size_t points) const;
};

// Covariance of a cone center, diagonal along (radial) and across (tangential) the line of
// sight from the LiDAR. Defaults calibrated on simulated scans (docs/perception.md,
// "Covariance"), so the normalized estimation error squared averages 2.
struct CovarianceModel {
    VarianceModel radial{1.4e-4, 0.0, 6.0e-3, 0.0};
    VarianceModel tangential{2.5e-5, 0.0, 1.0e-3, 3.0e-5};
};

void to_json(nlohmann::json& j, const VarianceModel& model);
void from_json(const nlohmann::json& j, VarianceModel& model);
void to_json(nlohmann::json& j, const CovarianceModel& model);
void from_json(const nlohmann::json& j, CovarianceModel& model);

// The covariance of a center at `center` (LiDAR frame, m) estimated from `points` returns. The
// LiDAR's axes are parallel to the body's, so it is also the covariance in the body frame.
[[nodiscard]] Mat2 observation_covariance(const CovarianceModel& model, const Vec2& center,
                                          std::size_t points);

// Expected number of returns from an unoccluded cone of `shape` standing on the ground at
// horizontal range `range` from the LiDAR, when the LiDAR is `sensor_height` above the ground
// and returns lower than `min_height` above it are removed as ground.
//
// Each channel at elevation e crosses the cone's near face at height z = h + d tan(e), with d
// the horizontal distance to the face; where 0 < z < H the cone's horizontal section has radius
// rho = R (1 - z / H) and subtends 2 asin(rho / range), so the channel gives that angle divided
// by the azimuth step returns on average (the azimuth phase is uniform). Summed over channels.
[[nodiscard]] double expected_cone_returns(const LidarGeometry& geometry, const ConeShape& shape,
                                           double range, double sensor_height, double min_height);

// The shape statistics of one cluster that the cone checks need.
struct ClusterSummary {
    std::size_t points = 0;
    Vec2 centroid = Vec2::Zero();    // LiDAR frame (m)
    double range = 0.0;              // horizontal distance of the centroid (m)
    double min_height = 0.0;         // above the ground plane (m)
    double max_height = 0.0;         //
    double tangential_extent = 0.0;  // across the line of sight to the centroid (m)
    double radial_extent = 0.0;      // along it (m)
};

[[nodiscard]] ClusterSummary summarize_cluster(std::span<const ObstaclePoint> points,
                                               std::span<const std::uint32_t> members);

enum class ConeCheck : std::uint8_t {
    Accepted,
    TooTall,
    TooWide,
    TooDeep,
    Floating,
    TooFewPoints,
    TooManyPoints,
};
inline constexpr std::size_t kConeCheckCount = 7;
[[nodiscard]] std::string_view to_string(ConeCheck check);

// Accepts a cluster only if it could be a cone at its range: no taller, wider or deeper than a
// big cone (with margins for noise and angular resolution), standing on the ground, and with a
// number of returns between a fraction of a small cone's and a multiple of a big cone's,
// as predicted by expected_cone_returns from the beam geometry. The count bounds are what reject
// far-away noise (too few returns for a cone that close) and partial merges or large objects
// (too many).
class ConeValidator {
public:
    // Throws std::invalid_argument for inconsistent margins or fractions.
    ConeValidator(const ConeCheckConfig& config, const LidarGeometry& geometry,
                  double ground_threshold);

    [[nodiscard]] ConeCheck check(const ClusterSummary& cluster, double sensor_height) const;
    [[nodiscard]] const ConeCheckConfig& config() const { return config_; }

private:
    ConeCheckConfig config_;
    LidarGeometry geometry_;
    double ground_threshold_ = 0.0;
    double max_channel_gap_ = 0.0;  // largest elevation step between adjacent channels (rad)
    ConeShape small_;
    ConeShape big_;
};

// The center of a cone from the returns on its visible face, LiDAR frame (m).
//
// A return at height z above the ground lies on the cone's surface, at horizontal distance
// rho(z) = R (1 - z / H) from the axis. The center c minimizes
//   sum_i (|p_i - c| - rho(z_i))^2 / sigma^2 + |c - c0|^2 / R^2
// (Gauss-Newton). The prior c0 is the centroid pushed away from the LiDAR by pi/4 times the mean
// rho, which is where the center of a circle lies behind the mean of points spread evenly across
// its visible half; it only decides the directions the returns leave unconstrained (a single
// column of returns at long range). Fitting the face rather than offsetting the centroid keeps a
// partly occluded cone's center right: the visible arc still fixes the circle.
struct ConeFit {
    Vec2 center = Vec2::Zero();
    double residual_sq = 0.0;  // sum of the squared surface residuals (m^2)
};
[[nodiscard]] ConeFit fit_cone_center(std::span<const ObstaclePoint> points,
                                      std::span<const std::uint32_t> members,
                                      const ConeShape& shape, const CenterFitConfig& config);

// The center under the cone size the returns favor. A return above big_cone_height settles it;
// otherwise both sizes are fitted and the big cone wins only if its likelihood ratio,
// (S_small - S_big) / (2 sigma^2) for the residual sums S, beats the prior odds against it.
// The top of a big cone is often unseen (above the highest channel close by, between channels
// far away), but the two sizes taper differently: the radius falls 2.8 mm per cm of height on a
// big cone and 3.5 mm on a small one, from different base radii.
struct ShapedConeFit {
    ConeFit fit;
    bool big = false;
};
[[nodiscard]] ShapedConeFit fit_cone(std::span<const ObstaclePoint> points,
                                     std::span<const std::uint32_t> members,
                                     const ClusterSummary& cluster, const CenterFitConfig& config);

// The most frequent truth_id among the members (ties: the smaller id, so -1 wins a tie with a
// real cone). For the evaluation only; nothing decides on it.
[[nodiscard]] std::int32_t majority_truth_id(std::span<const ObstaclePoint> points,
                                             std::span<const std::uint32_t> members,
                                             std::vector<std::int32_t>& scratch);

}  // namespace apex::perception
