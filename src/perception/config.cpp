// JSON forms of the perception configs: keys are the field names, missing keys keep their
// defaults, unknown keys throw (reject_unknown_keys).
#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"
#include "apex/perception/perception.hpp"

namespace apex::perception {

using nlohmann::json;

// Each APEX_*_FIELDS list drives both directions, so a field cannot be written but not read.
#define APEX_WRITE(name) j[#name] = c.name;
#define APEX_NAME(name) #name,
#define APEX_READ(name) read_optional(j, #name, c.name);
#define APEX_CONFIG_JSON(Type, FIELDS, context)               \
    void to_json(json& j, const Type& c) {                    \
        j = json::object();                                   \
        FIELDS(APEX_WRITE)                                    \
    }                                                         \
    void from_json(const json& j, Type& c) {                  \
        reject_unknown_keys(j, {FIELDS(APEX_NAME)}, context); \
        FIELDS(APEX_READ)                                     \
    }

// clang-format off
#define APEX_GROUND_FIELDS(X) \
    X(seed_cell_size) X(threshold) X(iterations) X(min_seeds) X(max_tilt_deg) X(min_scale)
#define APEX_CLUSTER_FIELDS(X) X(cell_size) X(max_range)
#define APEX_CONE_CHECK_FIELDS(X) \
    X(min_points) X(min_visible_fraction) X(max_excess_factor) X(height_margin) X(width_margin) \
    X(depth_margin) X(float_margin)
#define APEX_CENTER_FIT_FIELDS(X) \
    X(range_sigma) X(iterations) X(big_cone_height) X(big_cone_log_odds)
#define APEX_VARIANCE_FIELDS(X) X(constant) X(range_sq) X(inv_points) X(range_sq_inv_points)
#define APEX_COVARIANCE_FIELDS(X) X(radial) X(tangential)
#define APEX_CAMERA_FIELDS(X) \
    X(buffer_frames) X(max_time_gap) X(bearing_sigma) X(bearing_sigma_per_second) \
    X(range_sigma) X(range_sigma_per_meter) X(gate) X(fov_margin) X(range_margin)
#define APEX_PERCEPTION_FIELDS(X) \
    X(ground) X(clustering) X(cone_check) X(center_fit) X(covariance) X(camera)
// clang-format on

APEX_CONFIG_JSON(GroundConfig, APEX_GROUND_FIELDS, "perception.ground")
APEX_CONFIG_JSON(ClusterConfig, APEX_CLUSTER_FIELDS, "perception.clustering")
APEX_CONFIG_JSON(ConeCheckConfig, APEX_CONE_CHECK_FIELDS, "perception.cone_check")
APEX_CONFIG_JSON(CenterFitConfig, APEX_CENTER_FIT_FIELDS, "perception.center_fit")
APEX_CONFIG_JSON(VarianceModel, APEX_VARIANCE_FIELDS, "perception.covariance variance")
APEX_CONFIG_JSON(CovarianceModel, APEX_COVARIANCE_FIELDS, "perception.covariance")
APEX_CONFIG_JSON(CameraFusionConfig, APEX_CAMERA_FIELDS, "perception.camera")
APEX_CONFIG_JSON(PerceptionConfig, APEX_PERCEPTION_FIELDS, "perception")

#undef APEX_PERCEPTION_FIELDS
#undef APEX_CAMERA_FIELDS
#undef APEX_COVARIANCE_FIELDS
#undef APEX_VARIANCE_FIELDS
#undef APEX_CENTER_FIT_FIELDS
#undef APEX_CONE_CHECK_FIELDS
#undef APEX_CLUSTER_FIELDS
#undef APEX_GROUND_FIELDS
#undef APEX_CONFIG_JSON
#undef APEX_READ
#undef APEX_NAME
#undef APEX_WRITE

}  // namespace apex::perception
