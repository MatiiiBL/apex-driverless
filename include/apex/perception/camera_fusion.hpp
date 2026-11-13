#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"

// Camera color fusion: the LiDAR's cones take their color from the detections of the camera
// frame closest in time. See docs/perception.md, "Camera fusion".
namespace apex::perception {

struct CameraFusionConfig {
    std::size_t buffer_frames = 8;  // newest frames kept, by stamp (0.4 s at 20 Hz)
    double max_time_gap = 0.06;     // s: a frame farther from the scan than this gives no colors
    double bearing_sigma = 0.01;    // rad, detector bearing noise plus LiDAR center error
    // rad per second of stamp gap: bearing tolerance for the car's translation between the two
    // stamps, which the yaw-rate compensation leaves out
    double bearing_sigma_per_second = 0.6;
    double range_sigma = 0.3;             // m, detector range noise at short range
    double range_sigma_per_meter = 0.08;  // m per m: monocular range error grows with distance
    double gate = 9.21;                   // chi-square gate on the match cost (2 dof, 99%)
    double fov_margin = 0.05;             // rad beyond the half field of view still matched
    double range_margin = 2.0;            // m beyond the camera's max range still matched
};

void to_json(nlohmann::json& j, const CameraFusionConfig& config);
void from_json(const nlohmann::json& j, CameraFusionConfig& config);

// The newest camera frames, sorted by stamp, whatever order they arrive in.
class CameraBuffer {
public:
    enum class Insert { Added, Replaced, Dropped };

    explicit CameraBuffer(std::size_t capacity);

    // A frame with a stamp already buffered replaces it; once full, the oldest frame goes, and a
    // frame older than every buffered one is dropped. Frames with a non-finite stamp are dropped.
    Insert push(const CameraFrame& frame);
    // The frame whose stamp is closest to `stamp` (ties: the earlier), or nullptr when empty.
    [[nodiscard]] const CameraFrame* closest(Time stamp) const;
    [[nodiscard]] std::size_t size() const { return frames_.size(); }
    [[nodiscard]] std::span<const CameraFrame> frames() const { return frames_; }

private:
    std::size_t capacity_;
    std::vector<CameraFrame> frames_;
};

struct ColorMatch {
    ConeColor color = ConeColor::Unknown;
    double confidence = 0.0;  // in [0, 1]; 0 when Unknown
    int detection = -1;       // index into the frame's detections, -1 if unmatched
};

// Matches LiDAR cones to camera detections and hands each matched cone the detection's color.
//
// Each cone, given in the body frame at the scan's stamp, is rotated by -yaw_rate (t_camera -
// t_scan) about the body origin: where the car's turning between the stamps moves it in the
// camera frame. Its predicted bearing and range from the camera mount are compared with each
// detection by the cost
//   d^2 = (bearing error / sigma_b)^2 + (range error / sigma_r(range))^2,
// sigma_b^2 = bearing_sigma^2 + (bearing_sigma_per_second |t_camera - t_scan|)^2 and
// sigma_r = range_sigma + range_sigma_per_meter range. Pairs with d^2 >= gate are not matched;
// the rest are assigned jointly by minimum total cost (solve_assignment), so a near and a far
// cone at almost the same bearing each take the detection whose range fits, rather than both
// competing for the first one found. Color confidence is the detector's confidence times the
// match quality exp(-d^2 / 2).
class CameraFusion {
public:
    // Throws std::invalid_argument for non-positive sigmas, gate or buffer size.
    CameraFusion(const CameraFusionConfig& config, const CameraGeometry& geometry);

    CameraBuffer::Insert add_frame(const CameraFrame& frame) { return buffer_.push(frame); }

    // Fills `out` with one ColorMatch per position. Returns false, with every cone Unknown, when
    // no buffered frame is within max_time_gap of the scan (or yaw_rate is not finite).
    bool colorize(std::span<const Vec2> positions, Time scan_stamp, double yaw_rate,
                  std::vector<ColorMatch>& out);

    [[nodiscard]] const CameraBuffer& buffer() const { return buffer_; }

private:
    CameraFusionConfig config_;
    CameraGeometry geometry_;
    CameraBuffer buffer_;
    std::vector<std::size_t> candidates_;  // cones inside the camera's view
    std::vector<double> predicted_bearing_;
    std::vector<double> predicted_range_;
    std::vector<double> cost_;
};

}  // namespace apex::perception
