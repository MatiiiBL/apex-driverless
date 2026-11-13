#pragma once

#include <array>
#include <cstddef>
#include <span>
#include <string>
#include <vector>

#include "apex/core/messages.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/types.hpp"
#include "apex/perception/calibration.hpp"
#include "apex/perception/detector.hpp"

// Scoring of the perception output against the ground truth, for the evaluation. Ranges are
// horizontal distances from the LiDAR; truth ids are read here, and only here.
namespace apex::perception::eval {

// 0-10, 10-20, 20-30 and 30+ m.
inline constexpr std::size_t kBins = 4;
[[nodiscard]] std::size_t bin_of(double range);
[[nodiscard]] std::string bin_label(std::size_t bin);

struct DetectionBin {
    std::size_t in_view = 0;   // cones in the LiDAR's field of view and range
    std::size_t hit = 0;       // ... with at least one return
    std::size_t detected = 0;  // ... reported: the true positives
    std::size_t reported = 0;  // observations in this bin (by their own range)
    std::size_t false_positives = 0;
    // Sums over the true positives, by the cone's range.
    double error_sq = 0.0;  // m^2
    double radial_sq = 0.0;
    double tangential_sq = 0.0;
    double nees = 0.0;
};
using DetectionBins = std::array<DetectionBin, kBins>;

struct ColorScore {
    std::size_t visible = 0;          // true positives in the camera's field of view and range
    std::size_t colored = 0;          // ... given a color
    std::size_t correct = 0;          // ... the right one
    double confidence_correct = 0.0;  // sums of the color confidence
    double confidence_wrong = 0.0;
};

// Calibration samples and the big/small classification of the detector's cones.
struct DetectorScore {
    std::vector<VarianceSample> radial;
    std::vector<VarianceSample> tangential;
    std::size_t big_cones = 0;     // true positives that are big orange cones
    std::size_t big_as_big = 0;    // ... fitted as big
    std::size_t small_cones = 0;   // true positives that are small cones
    std::size_t small_as_big = 0;  // ... fitted as big
};

// The scene of one scan: the true cones and the car's true pose when the scan was taken.
struct Truth {
    std::span<const Cone> cones;
    Pose2 body;
};

// The truth cone index of each observation: the cone its truth_id names, if within `gate` (m)
// of it and not claimed by an earlier observation; -1, a false positive, otherwise.
[[nodiscard]] std::vector<int> match_observations(const ConeObservations& observations,
                                                  const Truth& truth, double gate);

void score_detections(const ConeObservations& observations, std::span<const int> matches,
                      const LidarScan& scan, const Truth& truth, const LidarGeometry& lidar,
                      DetectionBins& bins);

// `camera_body` is the car's pose at the camera frame's stamp: what the camera could see.
void score_colors(const ConeObservations& observations, std::span<const int> matches,
                  const Truth& truth, const Pose2& camera_body, const CameraGeometry& camera,
                  ColorScore& score);

void score_detector(std::span<const DetectedCone> detected, const Truth& truth,
                    const LidarGeometry& lidar, double gate, DetectorScore& score);

}  // namespace apex::perception::eval
