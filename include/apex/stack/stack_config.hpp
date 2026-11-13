#pragma once

#include <nlohmann/json_fwd.hpp>

#include "apex/control/controller.hpp"
#include "apex/estimation/estimator.hpp"
#include "apex/perception/perception.hpp"
#include "apex/planning/planning.hpp"
#include "apex/stack/known_pose_mapper.hpp"
#include "apex/stack/lap_counter.hpp"
#include "apex/stack/mission.hpp"

namespace apex::stack {

// Every tunable of the car's software: each module's own config, plus the stack's.
struct StackConfig {
    perception::PerceptionConfig perception;
    estimation::EstimatorConfig estimation;
    planning::ExplorationConfig exploration;
    planning::TrackModelConfig track_model;
    planning::RacelineConfig raceline;
    control::ControllerConfig controller;
    MissionConfig mission;
    LapCounterConfig lap_counter;
    KnownPoseMapperConfig known_pose_mapper;  // the perfect-state oracle's mapper

    double stop_decel = 5.0;       // m/s^2: braking to a standstill after the last lap
    double hold_brake = 2.0;       // m/s^2 commanded while staged and once finished
    double emergency_brake = 9.0;  // m/s^2 commanded once the mission has failed
    // Spacing of the lap-1 poses recorded for the track model.
    double driven_path_spacing = 0.5;  // m
};

void to_json(nlohmann::json& j, const StackConfig& config);
void from_json(const nlohmann::json& j, StackConfig& config);

}  // namespace apex::stack
