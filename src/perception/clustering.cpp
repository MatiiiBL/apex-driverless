#include "apex/perception/clustering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "apex/core/angles.hpp"

namespace apex::perception {
namespace {

constexpr std::uint32_t kEmpty = std::numeric_limits<std::uint32_t>::max();

struct Bounds {
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;
};

// Bounding box of the circular sector of radius `range` between two azimuths, origin included.
Bounds sector_bounds(double azimuth_min, double azimuth_max, double range) {
    if (azimuth_max - azimuth_min >= kTwoPi) return {-range, range, -range, range};
    Bounds b;
    const auto include = [&](double angle) {
        b.x_min = std::min(b.x_min, range * std::cos(angle));
        b.x_max = std::max(b.x_max, range * std::cos(angle));
        b.y_min = std::min(b.y_min, range * std::sin(angle));
        b.y_max = std::max(b.y_max, range * std::sin(angle));
    };
    include(azimuth_min);
    include(azimuth_max);
    // The extremes of cos and sin inside the interval: multiples of pi/2.
    const double quarter = 0.5 * kPi;
    for (double a = std::ceil(azimuth_min / quarter) * quarter; a <= azimuth_max; a += quarter) {
        include(a);
    }
    return b;
}

}  // namespace

GridClusterer::GridClusterer(const ClusterConfig& config, const LidarGeometry& geometry)
    : config_(config) {
    if (!(config.cell_size > 0.0) || !(config.max_range > 0.0)) {
        throw std::invalid_argument("ClusterConfig: cell_size and max_range must be positive");
    }
    if (!(geometry.azimuth_max > geometry.azimuth_min)) {
        throw std::invalid_argument("LidarGeometry: azimuth_max must exceed azimuth_min");
    }
    const double range = std::min(config.max_range, geometry.max_range);
    const Bounds b = sector_bounds(geometry.azimuth_min, geometry.azimuth_max, range);
    // One cell of margin on every side, so rounding at the sector's edge never drops a point.
    x_min_ = b.x_min - config.cell_size;
    y_min_ = b.y_min - config.cell_size;
    width_ = static_cast<std::size_t>(std::ceil((b.x_max - b.x_min) / config.cell_size)) + 3;
    height_ = static_cast<std::size_t>(std::ceil((b.y_max - b.y_min) / config.cell_size)) + 3;
    head_.assign(width_ * height_, kEmpty);
    label_.assign(width_ * height_, kEmpty);
}

void GridClusterer::cluster(std::span<const ObstaclePoint> points, Clusters& out) {
    out.clear();
    touched_.clear();
    const std::size_t count = std::min<std::size_t>(points.size(), kEmpty - 1);
    next_.resize(count);
    const double inv_cell = 1.0 / config_.cell_size;
    const auto width = static_cast<double>(width_);
    const auto height = static_cast<double>(height_);
    for (std::size_t i = 0; i < count; ++i) {
        next_[i] = kEmpty;
        const double gx = std::floor((points[i].x - x_min_) * inv_cell);
        const double gy = std::floor((points[i].y - y_min_) * inv_cell);
        if (!(gx >= 0.0 && gy >= 0.0 && gx < width && gy < height)) continue;
        const std::size_t cell =
            static_cast<std::size_t>(gy) * width_ + static_cast<std::size_t>(gx);
        if (head_[cell] == kEmpty) touched_.push_back(cell);
        next_[i] = head_[cell];
        head_[cell] = static_cast<std::uint32_t>(i);
    }

    std::uint32_t clusters = 0;
    for (const std::size_t seed : touched_) {
        if (label_[seed] != kEmpty) continue;
        label_[seed] = clusters;
        stack_.assign(1, seed);
        while (!stack_.empty()) {
            const std::size_t cell = stack_.back();
            stack_.pop_back();
            for (std::uint32_t p = head_[cell]; p != kEmpty; p = next_[p]) out.members.push_back(p);
            const std::size_t cx = cell % width_;
            const std::size_t cy = cell / width_;
            const std::size_t x0 = cx > 0 ? cx - 1 : 0;
            const std::size_t y0 = cy > 0 ? cy - 1 : 0;
            const std::size_t x1 = std::min(cx + 1, width_ - 1);
            const std::size_t y1 = std::min(cy + 1, height_ - 1);
            for (std::size_t y = y0; y <= y1; ++y) {
                for (std::size_t x = x0; x <= x1; ++x) {
                    const std::size_t neighbor = y * width_ + x;
                    if (head_[neighbor] == kEmpty || label_[neighbor] != kEmpty) continue;
                    label_[neighbor] = clusters;
                    stack_.push_back(neighbor);
                }
            }
        }
        out.offsets.push_back(static_cast<std::uint32_t>(out.members.size()));
        ++clusters;
    }

    for (const std::size_t cell : touched_) {
        head_[cell] = kEmpty;
        label_[cell] = kEmpty;
    }
}

}  // namespace apex::perception
