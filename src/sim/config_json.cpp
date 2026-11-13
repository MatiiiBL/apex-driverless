#include "apex/sim/config_json.hpp"

#include <array>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>

#include "apex/core/json.hpp"

namespace apex::sim {

namespace {

using nlohmann::json;

template <class Value>
struct Field {
    std::string_view key;
    Value* value;
};

// Each struct's fields, by name, as pointers into `n`; const or not with `n`, so one list
// serves both directions.
template <class N>
using Value = std::conditional_t<std::is_const_v<N>, const double, double>;

template <class N>
auto fields_of_lidar(N& n) {
    return std::array<Field<Value<N>>, 2>{
        {{"range_stddev", &n.range_stddev}, {"dropout", &n.dropout}}};
}
template <class N>
auto fields_of_camera(N& n) {
    return std::array<Field<Value<N>>, 11>{{{"min_range", &n.min_range},
                                            {"detection_probability", &n.detection_probability},
                                            {"detection_half_range", &n.detection_half_range},
                                            {"detection_falloff", &n.detection_falloff},
                                            {"bearing_stddev", &n.bearing_stddev},
                                            {"range_stddev", &n.range_stddev},
                                            {"range_stddev_quadratic", &n.range_stddev_quadratic},
                                            {"confusion_near", &n.confusion_near},
                                            {"confusion_far", &n.confusion_far},
                                            {"confidence_stddev", &n.confidence_stddev},
                                            {"false_positives", &n.false_positives}}};
}
template <class N>
auto fields_of_imu(N& n) {
    return std::array<Field<Value<N>>, 6>{{{"accel_stddev", &n.accel_stddev},
                                           {"gyro_stddev", &n.gyro_stddev},
                                           {"accel_bias", &n.accel_bias},
                                           {"gyro_bias", &n.gyro_bias},
                                           {"accel_bias_walk", &n.accel_bias_walk},
                                           {"gyro_bias_walk", &n.gyro_bias_walk}}};
}
template <class N>
auto fields_of_wheels(N& n) {
    return std::array<Field<Value<N>>, 2>{
        {{"stddev", &n.stddev}, {"slip_stiffness", &n.slip_stiffness}}};
}
template <class N>
auto fields_of_steering(N& n) {
    return std::array<Field<Value<N>>, 1>{{{"stddev", &n.stddev}}};
}
template <class N>
auto fields_of_rules(N& n) {
    return std::array<Field<Value<N>>, 7>{{{"min_length", &n.min_length},
                                           {"max_length", &n.max_length},
                                           {"min_radius", &n.min_radius},
                                           {"max_straight", &n.max_straight},
                                           {"min_clearance", &n.min_clearance},
                                           {"adjacency_length", &n.adjacency_length},
                                           {"straight_curvature", &n.straight_curvature}}};
}
template <class N>
auto fields_of_weights(N& n) {
    return std::array<Field<Value<N>>, 4>{{{"hairpin_weight", &n.hairpin_weight},
                                           {"corner_weight", &n.corner_weight},
                                           {"sweeper_weight", &n.sweeper_weight},
                                           {"chicane_weight", &n.chicane_weight}}};
}

void require_object(const json& j, std::string_view context) {
    if (!j.is_object()) {
        throw std::invalid_argument(std::string(context) + ": expected a JSON object");
    }
}

// Throws for a key of `j` that is neither a field nor one of `extra`.
template <class Fields>
void reject_unknown(const json& j, const Fields& fields, std::string_view context,
                    std::initializer_list<std::string_view> extra = {}) {
    for (const auto& item : j.items()) {
        bool known = false;
        for (const auto& field : fields) known = known || item.key() == field.key;
        for (const std::string_view key : extra) known = known || item.key() == key;
        if (!known) {
            throw std::invalid_argument(std::string(context) + ": unknown key '" + item.key() +
                                        "'");
        }
    }
}

template <class Fields>
void write(json& j, const Fields& fields) {
    for (const auto& field : fields) j[std::string(field.key)] = *field.value;
}

template <class Fields>
void read(const json& j, const Fields& fields) {
    for (const auto& field : fields) read_optional(j, field.key, *field.value);
}

template <class Fields>
void read_strict(const json& j, const Fields& fields, std::string_view context) {
    require_object(j, context);
    reject_unknown(j, fields, context);
    read(j, fields);
}

}  // namespace

void to_json(json& j, const LidarNoise& noise) {
    j = json::object();
    write(j, fields_of_lidar(noise));
}
void from_json(const json& j, LidarNoise& noise) {
    read_strict(j, fields_of_lidar(noise), "LidarNoise");
}
void to_json(json& j, const CameraNoise& noise) {
    j = json::object();
    write(j, fields_of_camera(noise));
}
void from_json(const json& j, CameraNoise& noise) {
    read_strict(j, fields_of_camera(noise), "CameraNoise");
}
void to_json(json& j, const ImuNoise& noise) {
    j = json::object();
    write(j, fields_of_imu(noise));
}
void from_json(const json& j, ImuNoise& noise) {
    read_strict(j, fields_of_imu(noise), "ImuNoise");
}
void to_json(json& j, const WheelNoise& noise) {
    j = json::object();
    write(j, fields_of_wheels(noise));
}
void from_json(const json& j, WheelNoise& noise) {
    read_strict(j, fields_of_wheels(noise), "WheelNoise");
}
void to_json(json& j, const SteeringNoise& noise) {
    j = json::object();
    write(j, fields_of_steering(noise));
}
void from_json(const json& j, SteeringNoise& noise) {
    read_strict(j, fields_of_steering(noise), "SteeringNoise");
}

void to_json(json& j, const SensorNoise& noise) {
    j = {{"lidar", noise.lidar},
         {"camera", noise.camera},
         {"imu", noise.imu},
         {"wheels", noise.wheels},
         {"steering", noise.steering}};
}
void from_json(const json& j, SensorNoise& noise) {
    reject_unknown_keys(j, {"lidar", "camera", "imu", "wheels", "steering"}, "SensorNoise");
    read_optional(j, "lidar", noise.lidar);
    read_optional(j, "camera", noise.camera);
    read_optional(j, "imu", noise.imu);
    read_optional(j, "wheels", noise.wheels);
    read_optional(j, "steering", noise.steering);
}

void to_json(json& j, const TrackRules& rules) {
    j = json::object();
    write(j, fields_of_rules(rules));
}
void from_json(const json& j, TrackRules& rules) {
    read_strict(j, fields_of_rules(rules), "TrackRules");
}

void to_json(json& j, const TrackGenOptions& options) {
    j = {{"rules", options.rules}, {"max_attempts", options.max_attempts}};
    write(j, fields_of_weights(options));
}
void from_json(const json& j, TrackGenOptions& options) {
    require_object(j, "TrackGenOptions");
    reject_unknown(j, fields_of_weights(options), "TrackGenOptions", {"rules", "max_attempts"});
    read(j, fields_of_weights(options));
    read_optional(j, "rules", options.rules);
    read_optional(j, "max_attempts", options.max_attempts);
}

}  // namespace apex::sim
