#include "apex/estimation/slam_config.hpp"

#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"

namespace apex::estimation {

// clang-format off
#define APEX_SLAM_FIELDS(X) \
    X(velocity_correlation_time) \
    X(position_noise_density) \
    X(yaw_noise_density) \
    X(initial_position_std) \
    X(initial_yaw_std) \
    X(observation_noise_scale) \
    X(min_observation_std) \
    X(max_observation_range) \
    X(max_new_landmark_range) \
    X(min_landmark_separation) \
    X(confirm_detections) \
    X(tentative_timeout) \
    X(max_landmarks) \
    X(history_duration)
// clang-format on

void to_json(nlohmann::json& j, const SlamConfig& config) {
    j = nlohmann::json::object();
#define APEX_WRITE(name) j[#name] = config.name;
    APEX_SLAM_FIELDS(APEX_WRITE)
#undef APEX_WRITE
}

void from_json(const nlohmann::json& j, SlamConfig& config) {
#define APEX_NAME(name) #name,
    reject_unknown_keys(j, {APEX_SLAM_FIELDS(APEX_NAME)}, "slam");
#undef APEX_NAME
#define APEX_READ(name) read_optional(j, #name, config.name);
    APEX_SLAM_FIELDS(APEX_READ)
#undef APEX_READ
}

#undef APEX_SLAM_FIELDS

}  // namespace apex::estimation
