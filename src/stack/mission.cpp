#include "apex/stack/mission.hpp"

#include <algorithm>
#include <format>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"
#include "apex/core/log.hpp"

namespace apex::stack {

void to_json(nlohmann::json& j, const MissionConfig& c) {
    j = {{"laps", c.laps},
         {"start_delay", c.start_delay},
         {"raceline_delay", c.raceline_delay},
         {"standstill_speed", c.standstill_speed},
         {"max_no_path_time", c.max_no_path_time},
         {"max_lost_time", c.max_lost_time},
         {"max_position_std", c.max_position_std}};
}

void from_json(const nlohmann::json& j, MissionConfig& c) {
    reject_unknown_keys(j,
                        {"laps", "start_delay", "raceline_delay", "standstill_speed",
                         "max_no_path_time", "max_lost_time", "max_position_std"},
                        "mission");
    read_optional(j, "laps", c.laps);
    read_optional(j, "start_delay", c.start_delay);
    read_optional(j, "raceline_delay", c.raceline_delay);
    read_optional(j, "standstill_speed", c.standstill_speed);
    read_optional(j, "max_no_path_time", c.max_no_path_time);
    read_optional(j, "max_lost_time", c.max_lost_time);
    read_optional(j, "max_position_std", c.max_position_std);
}

Mission::Mission(const MissionConfig& config) : config_(config) {
    if (config_.laps < 1) throw std::invalid_argument("mission: laps must be at least 1");
    if (!(config_.start_delay >= 0.0) || !(config_.raceline_delay >= 0.0) ||
        !(config_.standstill_speed > 0.0) || !(config_.max_no_path_time > 0.0) ||
        !(config_.max_lost_time >= 0.0) || !(config_.max_position_std > 0.0)) {
        throw std::invalid_argument("mission: times and thresholds must be positive");
    }
}

bool Mission::following_exploration() const {
    return state_ == MissionState::Exploring || state_ == MissionState::MapFinalized;
}

void Mission::transition(MissionState to, Time now, std::string reason) {
    log(to == MissionState::Failed ? LogLevel::Warn : LogLevel::Info, "t = {:.2f} s: {} -> {} ({})",
        now, to_string(state_), to_string(to), reason);
    transitions_.push_back({now, state_, to, std::move(reason)});
    state_ = to;
    since_ = now;
}

MissionState Mission::update(const MissionInputs& in) {
    if (is_terminal(state_)) return state_;

    if (!in.estimate_finite) {
        transition(MissionState::Failed, in.now, "non-finite state estimate");
        return state_;
    }
    if (in.estimate_usable) {
        lost_since_ = -1.0;
    } else {
        if (lost_since_ < 0.0) lost_since_ = in.now;
        if (in.now - lost_since_ > config_.max_lost_time) {
            transition(MissionState::Failed, in.now,
                       std::format("localization lost for {:.2f} s", in.now - lost_since_));
            return state_;
        }
    }
    if (following_exploration()) {
        // The planner gets max_no_path_time from entering the state to find a path.
        const Time fresh = std::max(in.last_path_time, since_);
        if (in.now - fresh > config_.max_no_path_time) {
            transition(MissionState::Failed, in.now,
                       std::format("no exploration path for {:.2f} s", in.now - fresh));
            return state_;
        }
    }

    switch (state_) {
        case MissionState::Staged:
            if (in.now >= config_.start_delay && in.estimate_usable) {
                transition(MissionState::Exploring, in.now, "go");
            }
            break;
        case MissionState::Exploring:
        case MissionState::MapFinalized:
        case MissionState::Racing:
            if (in.laps_completed >= config_.laps) {
                transition(MissionState::Stopping, in.now,
                           std::format("{} laps completed", in.laps_completed));
            } else if (state_ == MissionState::MapFinalized &&
                       in.now - since_ >= config_.raceline_delay) {
                transition(MissionState::Racing, in.now, "raceline ready");
            }
            break;
        case MissionState::Stopping:
            if (in.speed < config_.standstill_speed) {
                transition(MissionState::Finished, in.now, "stopped");
            }
            break;
        case MissionState::Finished:
        case MissionState::Failed:
            break;
    }
    return state_;
}

void Mission::map_finalized(Time now) {
    if (state_ != MissionState::Exploring) return;
    transition(MissionState::MapFinalized, now, "lap 1 closed, track model built");
}

void Mission::fail(Time now, std::string reason) {
    if (is_terminal(state_)) return;
    transition(MissionState::Failed, now, std::move(reason));
}

}  // namespace apex::stack
