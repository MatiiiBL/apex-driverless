#include "apex/core/json.hpp"

#include <stdexcept>
#include <string>

namespace apex {

using nlohmann::json;

json vec2_to_json(const Vec2& v) {
    return json::array({v.x(), v.y()});
}

Vec2 vec2_from_json(const json& j) {
    if (!j.is_array() || j.size() != 2) throw std::invalid_argument("expected [x, y]");
    return {j[0].get<double>(), j[1].get<double>()};
}

void reject_unknown_keys(const json& j, std::initializer_list<std::string_view> known,
                         std::string_view context) {
    if (!j.is_object()) {
        throw std::invalid_argument(std::string(context) + ": expected a JSON object");
    }
    for (const auto& item : j.items()) {
        bool found = false;
        for (const std::string_view key : known) found = found || key == item.key();
        if (!found) {
            throw std::invalid_argument(std::string(context) + ": unknown key '" + item.key() +
                                        "'");
        }
    }
}

void to_json(json& j, const Pose2& pose) {
    j = {{"x", pose.x}, {"y", pose.y}, {"yaw", pose.yaw}};
}

void from_json(const json& j, Pose2& pose) {
    reject_unknown_keys(j, {"x", "y", "yaw"}, "pose");
    read_optional(j, "x", pose.x);
    read_optional(j, "y", pose.y);
    read_optional(j, "yaw", pose.yaw);
}

void to_json(json& j, ConeColor color) {
    j = std::string(to_string(color));
}

void from_json(const json& j, ConeColor& color) {
    const auto parsed = cone_color_from_string(j.get<std::string>());
    if (!parsed) throw std::invalid_argument("unknown cone color '" + j.get<std::string>() + "'");
    color = *parsed;
}

void to_json(json& j, const Cone& cone) {
    j = {{"id", cone.id}, {"position", vec2_to_json(cone.position)}, {"color", cone.color}};
}

void from_json(const json& j, Cone& cone) {
    cone.id = j.at("id").get<std::int32_t>();
    cone.position = vec2_from_json(j.at("position"));
    cone.color = j.at("color").get<ConeColor>();
}

void to_json(json& j, const TrajectoryPoint& p) {
    j = {{"s", p.s},
         {"position", vec2_to_json(p.position)},
         {"yaw", p.yaw},
         {"curvature", p.curvature},
         {"speed", p.speed},
         {"accel", p.accel},
         {"width_left", p.width_left},
         {"width_right", p.width_right}};
}

void from_json(const json& j, TrajectoryPoint& p) {
    read_optional(j, "s", p.s);
    p.position = vec2_from_json(j.at("position"));
    read_optional(j, "yaw", p.yaw);
    read_optional(j, "curvature", p.curvature);
    read_optional(j, "speed", p.speed);
    read_optional(j, "accel", p.accel);
    read_optional(j, "width_left", p.width_left);
    read_optional(j, "width_right", p.width_right);
}

// Trajectories are stored compactly, one row per point (s is recomputed on load).
void to_json(json& j, const Trajectory& trajectory) {
    json rows = json::array();
    for (const TrajectoryPoint& p : trajectory.points()) {
        rows.push_back({p.position.x(), p.position.y(), p.yaw, p.curvature, p.speed, p.accel,
                        p.width_left, p.width_right});
    }
    j = {{"closed", trajectory.closed()},
         {"columns", {"x", "y", "yaw", "curvature", "speed", "accel", "width_left", "width_right"}},
         {"points", std::move(rows)}};
}

void from_json(const json& j, Trajectory& trajectory) {
    std::vector<TrajectoryPoint> points;
    for (const json& row : j.at("points")) {
        if (!row.is_array() || row.size() != 8) {
            throw std::invalid_argument("trajectory: each point needs 8 columns");
        }
        TrajectoryPoint p;
        p.position = {row[0].get<double>(), row[1].get<double>()};
        p.yaw = row[2].get<double>();
        p.curvature = row[3].get<double>();
        p.speed = row[4].get<double>();
        p.accel = row[5].get<double>();
        p.width_left = row[6].get<double>();
        p.width_right = row[7].get<double>();
        points.push_back(p);
    }
    trajectory = Trajectory(std::move(points), j.at("closed").get<bool>());
}

void to_json(json& j, const Track& track) {
    j = {{"name", track.name},
         {"seed", track.seed},
         {"start_pose", track.start_pose},
         {"timing_line",
          {vec2_to_json(track.timing_line_left), vec2_to_json(track.timing_line_right)}},
         {"cones", track.cones},
         {"centerline", track.centerline}};
}

void from_json(const json& j, Track& track) {
    track.name = j.at("name").get<std::string>();
    track.seed = j.at("seed").get<std::uint64_t>();
    track.start_pose = j.at("start_pose").get<Pose2>();
    track.timing_line_left = vec2_from_json(j.at("timing_line").at(0));
    track.timing_line_right = vec2_from_json(j.at("timing_line").at(1));
    track.cones = j.at("cones").get<std::vector<Cone>>();
    track.centerline = j.at("centerline").get<Trajectory>();
}

void to_json(json& j, const TireParams& p) {
    j = {{"B", p.B}, {"C", p.C}, {"E", p.E}, {"mu", p.mu}};
}

void from_json(const json& j, TireParams& p) {
    reject_unknown_keys(j, {"B", "C", "E", "mu"}, "tire");
    read_optional(j, "B", p.B);
    read_optional(j, "C", p.C);
    read_optional(j, "E", p.E);
    read_optional(j, "mu", p.mu);
}

// One list of (name, member) pairs drives both directions, so they cannot drift apart.
// clang-format off
#define APEX_VEHICLE_FIELDS(X) \
    X(mass) \
    X(yaw_inertia) \
    X(lf) \
    X(lr) \
    X(cg_height) \
    X(track_width) \
    X(body_front) \
    X(body_rear) \
    X(body_width) \
    X(front_tire) \
    X(rear_tire) \
    X(drag_area) \
    X(downforce_area) \
    X(air_density) \
    X(aero_balance_front) \
    X(rolling_resistance) \
    X(max_power) \
    X(max_drive_force) \
    X(max_brake_force) \
    X(drive_split_front) \
    X(brake_split_front) \
    X(max_speed) \
    X(max_steer) \
    X(max_steer_rate) \
    X(steer_time_constant) \
    X(steer_dead_time) \
    X(drive_time_constant) \
    X(drive_dead_time)
// clang-format on

void to_json(json& j, const VehicleParams& p) {
    j = json::object();
#define APEX_WRITE(name) j[#name] = p.name;
    APEX_VEHICLE_FIELDS(APEX_WRITE)
#undef APEX_WRITE
}

void from_json(const json& j, VehicleParams& p) {
#define APEX_NAME(name) #name,
    reject_unknown_keys(j, {APEX_VEHICLE_FIELDS(APEX_NAME)}, "vehicle");
#undef APEX_NAME
#define APEX_READ(name) read_optional(j, #name, p.name);
    APEX_VEHICLE_FIELDS(APEX_READ)
#undef APEX_READ
}

#undef APEX_VEHICLE_FIELDS

void to_json(json& j, const ConeMap& map) {
    json cones = json::array();
    for (const MapCone& cone : map.cones) {
        cones.push_back(
            {{"id", cone.id},
             {"position", vec2_to_json(cone.position)},
             {"covariance", {cone.covariance(0, 0), cone.covariance(0, 1), cone.covariance(1, 1)}},
             {"color", cone.color},
             {"observations", cone.observations},
             {"truth_id", cone.truth_id}});
    }
    j = {{"revision", map.revision}, {"cones", std::move(cones)}};
}

}  // namespace apex
