#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/sensor_rig.hpp"
#include "apex/perception/ground.hpp"

// Euclidean clustering of the obstacle points on the ground plane. See docs/perception.md,
// "Clustering".
namespace apex::perception {

struct ClusterConfig {
    // Grid cell side (m). Points closer than this always share a cluster; points farther apart
    // than 2 sqrt(2) times this are never joined directly. 0.15 m keeps the returns of one cone
    // together out to ~40 m (0.2 degree azimuth steps are 0.14 m apart there) and keeps cones
    // 0.43 m or more apart (surface to surface) in separate clusters.
    double cell_size = 0.15;
    // Points farther than this from the LiDAR (horizontally) are not clustered (m). Beyond it a
    // cone gives one or two returns, too few to tell it from noise.
    double max_range = 40.0;
};

void to_json(nlohmann::json& j, const ClusterConfig& config);
void from_json(const nlohmann::json& j, ClusterConfig& config);

// The clusters of one scan, stored flat: cluster k is the points
// members[offsets[k]] .. members[offsets[k + 1] - 1], as indices into the clustered span.
struct Clusters {
    std::vector<std::uint32_t> members;
    std::vector<std::uint32_t> offsets{0};

    [[nodiscard]] std::size_t size() const { return offsets.size() - 1; }
    [[nodiscard]] std::span<const std::uint32_t> operator[](std::size_t k) const {
        return std::span<const std::uint32_t>(members).subspan(offsets[k],
                                                               offsets[k + 1] - offsets[k]);
    }
    void clear() {
        members.clear();
        offsets.assign(1, 0);
    }
};

// Connected components of the occupied cells of a 2D grid, under 8-connectivity.
//
// Each point is dropped into its cell (a per-cell linked list), then a flood fill labels the
// occupied cells it can reach through occupied neighbors. Every step touches each point or each
// occupied cell a constant number of times, so the cost is linear in the number of points; only
// the cells touched by this scan are reset afterwards, so the grid's size does not count. The
// grid covers the LiDAR's azimuth sector out to max_range; it is allocated once.
//
// The result is deterministic: clusters come in the order of their first point's cell, which
// follows the input order.
class GridClusterer {
public:
    // Throws std::invalid_argument for a non-positive cell size or range.
    GridClusterer(const ClusterConfig& config, const LidarGeometry& geometry);

    void cluster(std::span<const ObstaclePoint> points, Clusters& out);
    [[nodiscard]] const ClusterConfig& config() const { return config_; }

private:
    ClusterConfig config_;
    double x_min_ = 0.0;
    double y_min_ = 0.0;
    std::size_t width_ = 0;  // cells along x
    std::size_t height_ = 0;
    std::vector<std::uint32_t> head_;   // per cell: first point, or kEmpty
    std::vector<std::uint32_t> label_;  // per cell: cluster, or kEmpty
    std::vector<std::uint32_t> next_;   // per point: next point in the same cell, or kEmpty
    std::vector<std::size_t> touched_;
    std::vector<std::size_t> stack_;
};

}  // namespace apex::perception
