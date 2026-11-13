#pragma once

#include <nlohmann/json.hpp>

#include "apex/sim/sensors.hpp"
#include "apex/sim/track_gen.hpp"
#include "apex/sim/track_rules.hpp"

// JSON forms of the sim module's configs (docs/design.md, section 5): objects keyed by field
// name; on reading, missing keys keep their defaults and unknown keys throw
// std::invalid_argument naming them. TrackGenOptions omits `build` (core's TrackBuildOptions has
// no JSON form; tracks are configured by seed and rules).
namespace apex::sim {

void to_json(nlohmann::json& j, const LidarNoise& noise);
void from_json(const nlohmann::json& j, LidarNoise& noise);
void to_json(nlohmann::json& j, const CameraNoise& noise);
void from_json(const nlohmann::json& j, CameraNoise& noise);
void to_json(nlohmann::json& j, const ImuNoise& noise);
void from_json(const nlohmann::json& j, ImuNoise& noise);
void to_json(nlohmann::json& j, const WheelNoise& noise);
void from_json(const nlohmann::json& j, WheelNoise& noise);
void to_json(nlohmann::json& j, const SteeringNoise& noise);
void from_json(const nlohmann::json& j, SteeringNoise& noise);
void to_json(nlohmann::json& j, const SensorNoise& noise);
void from_json(const nlohmann::json& j, SensorNoise& noise);
void to_json(nlohmann::json& j, const TrackRules& rules);
void from_json(const nlohmann::json& j, TrackRules& rules);
void to_json(nlohmann::json& j, const TrackGenOptions& options);
void from_json(const nlohmann::json& j, TrackGenOptions& options);

}  // namespace apex::sim
