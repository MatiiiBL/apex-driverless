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

#define APEX_EXPLORATION_FIELDS(X) \
    X(cone_range)                  \
    X(behind_distance)             \
    X(max_cones)                   \
    X(beam_width)                  \
    X(max_edges)                   \
    X(max_path_length)             \
    X(start_radius)                \
    X(min_width)                   \
    X(max_width)                   \
    X(max_edge_length)             \
    X(max_cone_spacing)            \
    X(max_turn_deg)                \
    X(min_cone_clearance)          \
    X(track_width)                 \
    X(width_weight)                \
    X(curvature_scale)             \
    X(curvature_weight)            \
    X(color_weight)                \
    X(cone_spacing)                \
    X(spacing_weight)              \
    X(heading_scale_deg)           \
    X(heading_weight)              \
    X(length_reward)               \
    X(smoothing_wavelength)        \
    X(path_spacing)                \
    X(stop_margin)                 \
    X(stop_reserve)                \
    X(min_path_length)             \
    X(speed)

void to_json(json& j, const ExplorationConfig& c) {
    j = json::object();
    APEX_EXPLORATION_FIELDS(APEX_WRITE)
}

void from_json(const json& j, ExplorationConfig& c) {
    reject_unknown_keys(j, {APEX_EXPLORATION_FIELDS(APEX_NAME)}, "exploration config");
    APEX_EXPLORATION_FIELDS(APEX_READ)
}

#undef APEX_EXPLORATION_FIELDS

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

#define APEX_RACELINE_FIELDS(X) \
    X(step)                     \
    X(safety_margin)            \
    X(iterations)               \
    X(max_curvature)            \
    X(regularization)           \
    X(qp_max_iterations)        \
    X(qp_tolerance)             \
    X(path_spacing)             \
    X(speed)

void to_json(json& j, const RacelineConfig& c) {
    j = json::object();
    APEX_RACELINE_FIELDS(APEX_WRITE)
}

void from_json(const json& j, RacelineConfig& c) {
    reject_unknown_keys(j, {APEX_RACELINE_FIELDS(APEX_NAME)}, "raceline config");
    APEX_RACELINE_FIELDS(APEX_READ)
}

#undef APEX_RACELINE_FIELDS
#undef APEX_TRACK_MODEL_FIELDS
#undef APEX_SPEED_FIELDS
#undef APEX_READ
#undef APEX_NAME
#undef APEX_WRITE

}  // namespace apex::planning
