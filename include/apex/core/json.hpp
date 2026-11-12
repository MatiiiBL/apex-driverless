#pragma once

#include <nlohmann/json.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/track.hpp"
#include "apex/core/trajectory.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

// JSON forms of the core records, for configs, saved tracks and run logs. Vectors are [x, y]
// arrays, colors are their to_string() names. Every module's config struct provides
// to_json/from_json in its own header, with missing keys keeping their defaults.
namespace apex {

void to_json(nlohmann::json& j, const Pose2& pose);
void from_json(const nlohmann::json& j, Pose2& pose);
void to_json(nlohmann::json& j, ConeColor color);
void from_json(const nlohmann::json& j, ConeColor& color);
void to_json(nlohmann::json& j, const Cone& cone);
void from_json(const nlohmann::json& j, Cone& cone);
void to_json(nlohmann::json& j, const TrajectoryPoint& point);
void from_json(const nlohmann::json& j, TrajectoryPoint& point);
void to_json(nlohmann::json& j, const Trajectory& trajectory);
void from_json(const nlohmann::json& j, Trajectory& trajectory);
void to_json(nlohmann::json& j, const Track& track);
void from_json(const nlohmann::json& j, Track& track);
void to_json(nlohmann::json& j, const TireParams& params);
void from_json(const nlohmann::json& j, TireParams& params);
void to_json(nlohmann::json& j, const VehicleParams& params);
void from_json(const nlohmann::json& j, VehicleParams& params);
void to_json(nlohmann::json& j, const ConeMap& map);

[[nodiscard]] nlohmann::json vec2_to_json(const Vec2& v);
[[nodiscard]] Vec2 vec2_from_json(const nlohmann::json& j);

// Throws std::invalid_argument naming the first key of object `j` that is not in `known`, so a
// typo in a config file fails loudly instead of silently keeping a default.
void reject_unknown_keys(const nlohmann::json& j, std::initializer_list<std::string_view> known,
                         std::string_view context);

// Reads j[key] into `value` when present; leaves the default otherwise.
template <class T>
void read_optional(const nlohmann::json& j, std::string_view key, T& value) {
    const auto it = j.find(key);
    if (it != j.end()) it->get_to(value);
}

}  // namespace apex
