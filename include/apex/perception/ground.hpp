#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"

// Ground removal: a robust fit of the ground plane in the LiDAR frame, then a height threshold
// above it. See docs/perception.md, "Ground removal".
namespace apex::perception {

struct GroundConfig {
    // Side of the square cells whose lowest return seeds the plane fit (m). Large enough that a
    // cone's shadow never covers a whole cell, small enough to give hundreds of seeds.
    double seed_cell_size = 2.0;
    // Returns higher than this above the fitted plane are obstacles (m). About ten times the
    // height noise of ground returns, so the ground never leaks into the clusters.
    double threshold = 0.04;
    // Iterations of the reweighted least-squares fit: half Huber, then half Tukey.
    int iterations = 12;
    // The fit needs at least this many seeds with nonzero weight to be trusted.
    std::size_t min_seeds = 20;
    // Planes steeper than this are rejected as failed fits (degrees).
    double max_tilt_deg = 10.0;
    // Lower bound on the robust residual scale (m), so a near-perfect ground does not make the
    // Tukey weights reject every seed but the best few.
    double min_scale = 0.005;
};

void to_json(nlohmann::json& j, const GroundConfig& config);
void from_json(const nlohmann::json& j, GroundConfig& config);

// The plane z = slope_x x + slope_y y + offset, in the LiDAR frame.
struct GroundPlane {
    double slope_x = 0.0;
    double slope_y = 0.0;
    double offset = 0.0;

    // Signed distance of a point above the plane, along the plane's upward normal (m).
    [[nodiscard]] double height_of(double x, double y, double z) const;
    // Distance from the LiDAR origin down to the plane (m).
    [[nodiscard]] double sensor_height() const;
    // Angle between the plane's normal and the LiDAR's z axis (rad).
    [[nodiscard]] double tilt() const;
};

struct GroundFit {
    GroundPlane plane;
    std::size_t seeds = 0;    // cells that contributed a seed
    std::size_t inliers = 0;  // seeds with nonzero final weight
    double scale = 0.0;       // robust residual scale of the inliers (m)
    bool ok = false;          // false: the plane is a fallback (previous or nominal)
};

// Estimates the ground plane of each scan without knowing the LiDAR's mount height.
//
// Seeds: the lowest return in each cell of a coarse Cartesian grid. A cone covers a small part
// of a cell, so nearly every seed lies on the ground, and one seed per cell weights the far
// ground as much as the dense near field (the far field is where a tilt error grows into a
// height error). The plane is then fitted to the seeds by iteratively reweighted least squares:
// first with Huber weights (a convex problem, so it converges from the flat initial guess at the
// median seed height), then with Tukey's biweight, which gives outliers (seeds on cones beyond
// the ground's range, in shadows) zero weight. Scale: the median absolute residual.
//
//   P. J. Huber, "Robust estimation of a location parameter", Annals of Mathematical
//   Statistics 35(1), 1964.
//   A. E. Beaton and J. W. Tukey, "The fitting of power series, meaning polynomials,
//   illustrated on band-spectroscopic data", Technometrics 16(2), 1974.
//   D. Zermas, I. Izzat and N. Papanikolopoulos, "Fast segmentation of 3D point clouds: a
//   paradigm on LiDAR data for autonomous vehicle applications", ICRA 2017 (lowest-point seeds).
//
// A fit with too few inliers, a non-finite result or a tilt beyond max_tilt_deg is rejected:
// the previous accepted plane is returned with ok = false (before any success, the nominal
// plane z = -mount height). Cost: linear in the number of points.
class GroundEstimator {
public:
    // Throws std::invalid_argument for a non-positive cell size, threshold or iteration count.
    GroundEstimator(const GroundConfig& config, const LidarGeometry& geometry);

    [[nodiscard]] GroundFit fit(std::span<const LidarPoint> points);
    [[nodiscard]] const GroundConfig& config() const { return config_; }

private:
    struct Seed {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    [[nodiscard]] bool fit_seeds(GroundFit& fit);

    GroundConfig config_;
    double extent_ = 0.0;  // the seed grid spans [-extent_, extent_] in x and y
    std::size_t cells_per_side_ = 0;
    GroundPlane last_good_;
    std::vector<std::size_t> lowest_;  // per cell: index of its lowest point, or kNone
    std::vector<std::size_t> touched_;
    std::vector<Seed> seeds_;
    std::vector<double> residuals_;
    std::vector<double> scratch_;
};

// A non-ground return, ready for clustering: position in the LiDAR frame and height above the
// fitted ground plane.
struct ObstaclePoint {
    float x = 0.0F;
    float y = 0.0F;
    float height = 0.0F;
    std::int32_t truth_id = -1;
};

// Appends to `out` (after clearing it) every finite point higher than `threshold` above `plane`
// and within `max_range` (horizontal) of the LiDAR.
void extract_obstacles(std::span<const LidarPoint> points, const GroundPlane& plane,
                       double threshold, double max_range, std::vector<ObstaclePoint>& out);

}  // namespace apex::perception
