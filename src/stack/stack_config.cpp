#include "apex/stack/stack_config.hpp"

#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"

namespace apex::stack {

void to_json(nlohmann::json& j, const StackConfig& c) {
    j = nlohmann::json::object();
    j["perception"] = c.perception;
    j["estimation"] = c.estimation;
    j["exploration"] = c.exploration;
    j["track_model"] = c.track_model;
    j["raceline"] = c.raceline;
    j["controller"] = c.controller;
    j["mission"] = c.mission;
    j["lap_counter"] = c.lap_counter;
    j["known_pose_mapper"] = c.known_pose_mapper;
    j["hold_brake"] = c.hold_brake;
    j["emergency_brake"] = c.emergency_brake;
    j["driven_path_spacing"] = c.driven_path_spacing;
}

void from_json(const nlohmann::json& j, StackConfig& c) {
    reject_unknown_keys(j,
                        {"perception", "estimation", "exploration", "track_model", "raceline",
                         "controller", "mission", "lap_counter", "known_pose_mapper", "hold_brake",
                         "emergency_brake", "driven_path_spacing"},
                        "stack");
    read_optional(j, "perception", c.perception);
    read_optional(j, "estimation", c.estimation);
    read_optional(j, "exploration", c.exploration);
    read_optional(j, "track_model", c.track_model);
    read_optional(j, "raceline", c.raceline);
    read_optional(j, "controller", c.controller);
    read_optional(j, "mission", c.mission);
    read_optional(j, "lap_counter", c.lap_counter);
    read_optional(j, "known_pose_mapper", c.known_pose_mapper);
    read_optional(j, "hold_brake", c.hold_brake);
    read_optional(j, "emergency_brake", c.emergency_brake);
    read_optional(j, "driven_path_spacing", c.driven_path_spacing);
}

}  // namespace apex::stack
