#pragma once

#include <span>

#include "apex/core/messages.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/sensor_rig.hpp"
#include "apex/core/types.hpp"

// Synthetic sensor data with known answers for the perception tests: an exact ray caster of the
// LiDAR against cones (right circular cones of cone_shape()) on a flat ground, and a cone
// detector for the camera.
namespace apex::perception::testing {

struct LidarSimOptions {
    double sensor_height = 0.20;  // LiDAR origin above the ground (m)
    double pitch = 0.0;           // rad, rotation about the LiDAR's y axis, positive nose down
    double range_sigma = 0.0;     // m, Gaussian range noise
    double dropout = 0.0;         // probability that a ray returns nothing
};

// One revolution from a LiDAR whose ground-plane pose in the world is `sensor_pose`. Points in
// the LiDAR frame, tagged with the id of the cone they hit (-1 for the ground).
[[nodiscard]] LidarScan cast_lidar(const LidarGeometry& geometry, const Pose2& sensor_pose,
                                   std::span<const Cone> cones, const LidarSimOptions& options,
                                   Rng& rng, Time stamp = 0.0);

struct CameraSimOptions {
    double bearing_sigma = 0.0;         // rad
    double range_sigma_fraction = 0.0;  // relative range noise
    double confidence = 0.9;
    // Detection probability, falling linearly from 1 at the camera to this at its max range.
    double detect_at_max_range = 1.0;
    // Probability of reporting another color, rising linearly from 0 at the camera to this at
    // its max range.
    double confusion_at_max_range = 0.0;
    double false_positives = 0.0;  // mean per frame (Poisson), anywhere in the field of view
};

// The cones in the camera's field of view and range, seen from a car at `body_pose` (no
// occlusion), with misses, color confusion and false positives as `options` asks.
[[nodiscard]] CameraFrame detect_cones(const CameraGeometry& geometry, const Pose2& body_pose,
                                       std::span<const Cone> cones, const CameraSimOptions& options,
                                       Rng& rng, Time stamp);

// The LiDAR's ground-plane pose for a car at `body_pose`.
[[nodiscard]] Pose2 lidar_pose(const Pose2& body_pose, const LidarGeometry& geometry);

}  // namespace apex::perception::testing
