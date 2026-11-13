#include <cmath>
#include <stdexcept>

#include "apex/runner/recorders.hpp"

namespace apex::runner {

using nlohmann::json;

namespace {

// Rounded to the millimeter (and the milliradian), which an animation cannot tell apart, so the
// file stays small.
double mm(double value) {
    return std::round(value * 1000.0) / 1000.0;
}

json point(const Vec2& p) {
    return json::array({mm(p.x()), mm(p.y())});
}

json cone_entry(const Vec2& p, ConeColor color) {
    return json::array({mm(p.x()), mm(p.y()), to_string(color)});
}

bool builds_map(stack::MissionState state) {
    return state == stack::MissionState::Staged || state == stack::MissionState::Exploring ||
           state == stack::MissionState::MapFinalized;
}

}  // namespace

ReplayRecorder::ReplayRecorder(const ReplayOptions& options) : options_(options) {
    if (!(options.frame_period > 0.0) || !(options.map_period > 0.0) ||
        !(options.reference_period > 0.0) || !(options.path_spacing > 0.0)) {
        throw std::invalid_argument("replay: periods and spacing must be positive");
    }
}

void ReplayRecorder::on_start(const Track& track) {
    json centerline = json::array();
    for (const TrajectoryPoint& p : track.centerline.points())
        centerline.push_back(point(p.position));
    json cones = json::array();
    for (const Cone& cone : track.cones) cones.push_back(cone_entry(cone.position, cone.color));
    replay_ = {{"track",
                {{"name", track.name},
                 {"centerline", std::move(centerline)},
                 {"cones", std::move(cones)},
                 {"timing_line", {point(track.timing_line_left), point(track.timing_line_right)}}}},
               {"frames", json::array()},
               {"maps", json::array()},
               {"references", json::array()}};
    next_frame_ = 0.0;
    have_map_ = false;
    map_frozen_ = false;
    have_reference_ = false;
    reference_key_.clear();
}

void ReplayRecorder::on_control(const TickView& tick) {
    if (tick.time >= next_frame_ - 1e-9) {
        next_frame_ =
            (std::floor(tick.time / options_.frame_period + 1e-9) + 1.0) * options_.frame_period;
        const StateEstimate estimate = tick.driver.estimate();
        const VehicleState& truth = tick.truth;
        replay_["frames"].push_back({{"t", mm(tick.time)},
                                     {"state", stack::to_string(tick.driver.mission_state())},
                                     {"x", mm(truth.pose.x)},
                                     {"y", mm(truth.pose.y)},
                                     {"yaw", mm(truth.pose.yaw)},
                                     {"speed", mm(std::hypot(truth.vx, truth.vy))},
                                     {"est", json::array({mm(estimate.pose.x), mm(estimate.pose.y),
                                                          mm(estimate.pose.yaw)})}});
    }
    record_map(tick);
    record_reference(tick);
}

void ReplayRecorder::record_map(const TickView& tick) {
    const ConeMap* map = tick.driver.map();
    if (map == nullptr || map_frozen_) return;
    const bool building = builds_map(tick.driver.mission_state());
    const bool changed = !have_map_ || map->revision != map_revision_;
    // While building: every map_period if it changed. Once racing: the frozen map, once.
    if (building && (!changed || (have_map_ && tick.time - last_map_ < options_.map_period))) {
        return;
    }
    json cones = json::array();
    for (const MapCone& cone : map->cones) cones.push_back(cone_entry(cone.position, cone.color));
    replay_["maps"].push_back({{"t", mm(tick.time)}, {"cones", std::move(cones)}});
    have_map_ = true;
    last_map_ = tick.time;
    map_revision_ = map->revision;
    map_frozen_ = !building;
}

void ReplayRecorder::record_reference(const TickView& tick) {
    const Trajectory* reference = tick.driver.reference();
    if (reference == nullptr || reference->empty()) return;
    const auto state = static_cast<std::uint8_t>(tick.driver.mission_state());
    const TrajectoryPoint& first = reference->points().front();
    const TrajectoryPoint& last = reference->points().back();
    const std::vector<double> key = {static_cast<double>(reference->size()),
                                     reference->length(),
                                     first.position.x(),
                                     first.position.y(),
                                     last.position.x(),
                                     last.position.y()};
    const bool state_changed = !have_reference_ || state != reference_state_;
    if (!state_changed &&
        (key == reference_key_ || tick.time - last_reference_ < options_.reference_period)) {
        return;
    }
    json points = json::array();
    double next_s = 0.0;
    for (const TrajectoryPoint& p : reference->points()) {
        if (p.s + 1e-9 < next_s) continue;
        points.push_back(point(p.position));
        next_s = p.s + options_.path_spacing;
    }
    replay_["references"].push_back({{"t", mm(tick.time)},
                                     {"state", stack::to_string(tick.driver.mission_state())},
                                     {"closed", reference->closed()},
                                     {"points", std::move(points)}});
    have_reference_ = true;
    reference_state_ = state;
    reference_key_ = key;
    last_reference_ = tick.time;
}

}  // namespace apex::runner
