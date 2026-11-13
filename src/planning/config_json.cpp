#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"
#include "apex/planning/planning.hpp"

namespace apex::planning {

using nlohmann::json;

// Each config's fields are listed once; the macros expand the list into the writer, the
// known-key check and the reader.
#define APEX_WRITE(name) j[#name] = c.name;
#define APEX_NAME(name) #name,
#define APEX_READ(name) read_optional(j, #name, c.name);

#define APEX_SPEED_FIELDS(X) \
    X(grip_factor)           \
    X(max_speed)

void to_json(json& j, const SpeedProfileConfig& c) {
    j = json::object();
    APEX_SPEED_FIELDS(APEX_WRITE)
}

void from_json(const json& j, SpeedProfileConfig& c) {
    reject_unknown_keys(j, {APEX_SPEED_FIELDS(APEX_NAME)}, "speed profile config");
    APEX_SPEED_FIELDS(APEX_READ)
}

#define APEX_TRACK_MODEL_FIELDS(X) \
    X(reference_spacing)           \
    X(min_lateral)                 \
    X(max_lateral)                 \
    X(outlier_distance)            \
    X(outlier_neighbors)           \
    X(fill_gap)                    \
    X(fill_spacing)                \
    X(max_gap)                     \
    X(min_cones_per_side)          \
    X(max_closure_gap)             \
    X(smoothing_wavelength)        \
    X(width_wavelength)            \
    X(passes)                      \
    X(centerline_spacing)

void to_json(json& j, const TrackModelConfig& c) {
    j = json::object();
    APEX_TRACK_MODEL_FIELDS(APEX_WRITE)
}

void from_json(const json& j, TrackModelConfig& c) {
    reject_unknown_keys(j, {APEX_TRACK_MODEL_FIELDS(APEX_NAME)}, "track model config");
    APEX_TRACK_MODEL_FIELDS(APEX_READ)
}

#undef APEX_TRACK_MODEL_FIELDS
#undef APEX_SPEED_FIELDS
#undef APEX_READ
#undef APEX_NAME
#undef APEX_WRITE

}  // namespace apex::planning
