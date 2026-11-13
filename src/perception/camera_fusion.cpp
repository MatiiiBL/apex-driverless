#include "apex/perception/camera_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "apex/core/angles.hpp"
#include "apex/perception/assignment.hpp"

namespace apex::perception {
namespace {

bool finite(const CameraDetection& d) {
    return std::isfinite(d.bearing) && std::isfinite(d.range) && std::isfinite(d.confidence);
}

}  // namespace

CameraBuffer::CameraBuffer(std::size_t capacity) : capacity_(capacity) {
    if (capacity == 0) throw std::invalid_argument("CameraBuffer: capacity must be positive");
    frames_.reserve(capacity + 1);
}

CameraBuffer::Insert CameraBuffer::push(const CameraFrame& frame) {
    if (!std::isfinite(frame.stamp)) return Insert::Dropped;
    const auto at = std::lower_bound(
        frames_.begin(), frames_.end(), frame.stamp,
        [](const CameraFrame& buffered, Time stamp) { return buffered.stamp < stamp; });
    if (at != frames_.end() && at->stamp == frame.stamp) {
        *at = frame;
        return Insert::Replaced;
    }
    if (frames_.size() == capacity_ && at == frames_.begin()) return Insert::Dropped;
    frames_.insert(at, frame);
    if (frames_.size() > capacity_) frames_.erase(frames_.begin());
    return Insert::Added;
}

const CameraFrame* CameraBuffer::closest(Time stamp) const {
    if (frames_.empty()) return nullptr;
    const auto after =
        std::lower_bound(frames_.begin(), frames_.end(), stamp,
                         [](const CameraFrame& buffered, Time t) { return buffered.stamp < t; });
    if (after == frames_.begin()) return &*after;
    const auto before = std::prev(after);
    if (after == frames_.end()) return &*before;
    return stamp - before->stamp <= after->stamp - stamp ? &*before : &*after;
}

CameraFusion::CameraFusion(const CameraFusionConfig& config, const CameraGeometry& geometry)
    : config_(config), geometry_(geometry), buffer_(config.buffer_frames) {
    if (!(config.max_time_gap >= 0.0) || !(config.bearing_sigma > 0.0) ||
        !(config.bearing_sigma_per_second >= 0.0) || !(config.range_sigma > 0.0) ||
        !(config.range_sigma_per_meter >= 0.0) || !(config.gate > 0.0) ||
        !(config.fov_margin >= 0.0) || !(config.range_margin >= 0.0)) {
        throw std::invalid_argument(
            "CameraFusionConfig: sigmas and gate must be positive, margins non-negative");
    }
    if (!(geometry.horizontal_fov > 0.0) || !(geometry.max_range > 0.0)) {
        throw std::invalid_argument("CameraGeometry: field of view and range must be positive");
    }
}

bool CameraFusion::colorize(std::span<const Vec2> positions, Time scan_stamp, double yaw_rate,
                            std::vector<ColorMatch>& out) {
    out.assign(positions.size(), ColorMatch{});
    const CameraFrame* frame = buffer_.closest(scan_stamp);
    if (frame == nullptr || !std::isfinite(yaw_rate)) return false;
    const double gap = frame->stamp - scan_stamp;
    if (!(std::abs(gap) <= config_.max_time_gap)) return false;
    if (frame->detections.empty()) return true;

    // Where each cone appears from the camera at the frame's stamp.
    const double rotation = -yaw_rate * gap;
    const double c = std::cos(rotation);
    const double s = std::sin(rotation);
    const Vec2 camera = geometry_.mount.head<2>();
    const double half_fov = 0.5 * geometry_.horizontal_fov + config_.fov_margin;
    const double max_range = geometry_.max_range + config_.range_margin;
    candidates_.clear();
    predicted_bearing_.clear();
    predicted_range_.clear();
    for (std::size_t j = 0; j < positions.size(); ++j) {
        const Vec2& p = positions[j];
        const Vec2 rel = Vec2(c * p.x() - s * p.y(), s * p.x() + c * p.y()) - camera;
        const double bearing = std::atan2(rel.y(), rel.x());
        const double range = rel.norm();
        if (std::abs(bearing) > half_fov || range > max_range || !std::isfinite(range)) continue;
        candidates_.push_back(j);
        predicted_bearing_.push_back(bearing);
        predicted_range_.push_back(range);
    }
    if (candidates_.empty()) return true;

    const std::vector<CameraDetection>& detections = frame->detections;
    const double bearing_var = config_.bearing_sigma * config_.bearing_sigma +
                               std::pow(config_.bearing_sigma_per_second * gap, 2);
    cost_.assign(candidates_.size() * detections.size(), config_.gate);
    for (std::size_t row = 0; row < candidates_.size(); ++row) {
        const double range_sigma =
            config_.range_sigma + config_.range_sigma_per_meter * predicted_range_[row];
        for (std::size_t col = 0; col < detections.size(); ++col) {
            const CameraDetection& d = detections[col];
            if (!finite(d)) continue;
            const double db = angle_diff(d.bearing, predicted_bearing_[row]);
            const double dr = (d.range - predicted_range_[row]) / range_sigma;
            cost_[row * detections.size() + col] = db * db / bearing_var + dr * dr;
        }
    }
    const std::vector<int> match =
        solve_assignment(cost_, candidates_.size(), detections.size(), config_.gate);
    for (std::size_t row = 0; row < candidates_.size(); ++row) {
        if (match[row] < 0) continue;
        const auto col = static_cast<std::size_t>(match[row]);
        const CameraDetection& d = detections[col];
        ColorMatch& result = out[candidates_[row]];
        result.detection = match[row];
        if (d.color == ConeColor::Unknown) continue;
        result.color = d.color;
        result.confidence = std::clamp(d.confidence, 0.0, 1.0) *
                            std::exp(-0.5 * cost_[row * detections.size() + col]);
    }
    return true;
}

}  // namespace apex::perception
