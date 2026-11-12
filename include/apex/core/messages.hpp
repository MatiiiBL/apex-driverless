#pragma once

#include <array>
#include <cstdint>
#include <variant>
#include <vector>

#include "apex/core/types.hpp"

// The records that cross module boundaries, from sensor data to the map. See docs/design.md,
// section 4.
//
// Fields named truth_id carry the id of the ground-truth Cone a measurement came from (-1 for
// none: ground, clutter, a false positive). They exist so the evaluation can score data
// association. The car's software only passes them along, never reads them to make a decision:
// a test runs the whole stack with shuffled ids and checks that nothing changes.
namespace apex {

// ---- Sensor messages, produced by the simulator ----

struct ImuSample {
    Time stamp = 0.0;
    double ax = 0.0;  // body-frame specific force at the CoG (m/s^2)
    double ay = 0.0;
    double yaw_rate = 0.0;  // rad/s
};

struct WheelSpeeds {
    Time stamp = 0.0;
    double front = 0.0;  // circumferential speed, averaged over the axle (m/s)
    double rear = 0.0;
};

struct SteeringSample {
    Time stamp = 0.0;
    double angle = 0.0;  // measured front wheel angle (rad)
};

struct LidarPoint {
    float x = 0.0F;  // LiDAR frame (m)
    float y = 0.0F;
    float z = 0.0F;
    std::int32_t truth_id = -1;
};

// One revolution, treated as instantaneous at `stamp` (no motion distortion within a scan).
// Only returns are listed: rays that hit nothing within range are omitted.
struct LidarScan {
    Time stamp = 0.0;
    std::vector<LidarPoint> points;
};

struct CameraDetection {
    double bearing = 0.0;  // rad from the camera's +x axis (= body +x), counter-clockwise
    double range = 0.0;    // ground-plane distance from the camera mount (m)
    ConeColor color = ConeColor::Unknown;
    double confidence = 0.0;  // detector score in [0, 1]
    std::int32_t truth_id = -1;
};

struct CameraFrame {
    Time stamp = 0.0;
    std::vector<CameraDetection> detections;
};

using SensorMessage = std::variant<ImuSample, WheelSpeeds, SteeringSample, LidarScan, CameraFrame>;

[[nodiscard]] Time stamp_of(const SensorMessage& message);

// ---- Perception output ----

struct ConeObservation {
    Vec2 position = Vec2::Zero();        // cone center in the body frame at the stamp (m)
    Mat2 covariance = Mat2::Identity();  // of position (m^2)
    ConeColor color = ConeColor::Unknown;
    double color_confidence = 0.0;  // in [0, 1]; 0 when color is Unknown
    std::int32_t truth_id = -1;
};

struct ConeObservations {
    Time stamp = 0.0;  // the stamp of the LiDAR scan they came from
    std::vector<ConeObservation> cones;
};

// ---- Estimation output ----

struct StateEstimate {
    Time stamp = 0.0;
    Pose2 pose;  // CoG in the map frame, which is the world frame (the car is told its start)
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
    Mat3 pose_covariance = Mat3::Zero();  // over (x, y, yaw)
    bool valid = false;
};

struct MapCone {
    std::int32_t id = -1;  // stable for the lifetime of the map
    Vec2 position = Vec2::Zero();
    Mat2 covariance = Mat2::Identity();
    ConeColor color = ConeColor::Unknown;  // the color with the most votes
    std::array<std::uint32_t, kConeColorCount> color_votes{};
    std::uint32_t observations = 0;
    std::int32_t truth_id = -1;  // majority truth id of the observations associated with it
};

struct ConeMap {
    std::vector<MapCone> cones;
    std::uint64_t revision = 0;  // incremented whenever the map changes
};

}  // namespace apex
