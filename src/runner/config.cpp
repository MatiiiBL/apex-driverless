#include "apex/runner/config.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"
#include "apex/sim/config_json.hpp"

namespace apex::runner {

using nlohmann::json;

void to_json(json& j, const TrackSpec& spec) {
    switch (spec.source) {
        case TrackSpec::Source::Generated:
            j = {{"seed", spec.seed}};
            return;
        case TrackSpec::Source::Layout:
            j = {{"layout", spec.layout}};
            return;
        case TrackSpec::Source::File:
            j = {{"file", spec.file}};
            return;
        case TrackSpec::Source::Oval:
            j = {{"oval", {{"straight", spec.oval_straight}, {"radius", spec.oval_radius}}}};
            return;
    }
}

void from_json(const json& j, TrackSpec& spec) {
    reject_unknown_keys(j, {"seed", "layout", "file", "oval"}, "track");
    if (j.size() != 1) {
        throw std::invalid_argument("track: give exactly one of seed, layout, file, oval");
    }
    spec = TrackSpec{};
    if (j.contains("seed")) {
        spec.source = TrackSpec::Source::Generated;
        spec.seed = j.at("seed").get<std::uint64_t>();
    } else if (j.contains("layout")) {
        spec.source = TrackSpec::Source::Layout;
        spec.layout = j.at("layout").get<std::string>();
    } else if (j.contains("file")) {
        spec.source = TrackSpec::Source::File;
        spec.file = j.at("file").get<std::string>();
    } else {
        spec.source = TrackSpec::Source::Oval;
        const json& oval = j.at("oval");
        reject_unknown_keys(oval, {"straight", "radius"}, "track.oval");
        read_optional(oval, "straight", spec.oval_straight);
        read_optional(oval, "radius", spec.oval_radius);
    }
}

void to_json(json& j, const Latencies& l) {
    j = {{"imu", l.imu},
         {"wheels", l.wheels},
         {"steering", l.steering},
         {"camera", l.camera},
         {"lidar", l.lidar}};
}

void from_json(const json& j, Latencies& l) {
    reject_unknown_keys(j, {"imu", "wheels", "steering", "camera", "lidar"}, "latency");
    read_optional(j, "imu", l.imu);
    read_optional(j, "wheels", l.wheels);
    read_optional(j, "steering", l.steering);
    read_optional(j, "camera", l.camera);
    read_optional(j, "lidar", l.lidar);
}

void to_json(json& j, const DnfRules& r) {
    j = {{"stall_speed", r.stall_speed},
         {"stall_time", r.stall_time},
         {"max_off_track", r.max_off_track},
         {"time_limit", r.time_limit},
         {"stop_timeout", r.stop_timeout}};
}

void from_json(const json& j, DnfRules& r) {
    reject_unknown_keys(
        j, {"stall_speed", "stall_time", "max_off_track", "time_limit", "stop_timeout"}, "dnf");
    read_optional(j, "stall_speed", r.stall_speed);
    read_optional(j, "stall_time", r.stall_time);
    read_optional(j, "max_off_track", r.max_off_track);
    read_optional(j, "time_limit", r.time_limit);
    read_optional(j, "stop_timeout", r.stop_timeout);
}

void to_json(json& j, const PerceptionOracleConfig& c) {
    j = {{"max_range", c.max_range}, {"sigma", c.sigma}, {"sigma_per_meter", c.sigma_per_meter}};
}

void from_json(const json& j, PerceptionOracleConfig& c) {
    reject_unknown_keys(j, {"max_range", "sigma", "sigma_per_meter"}, "perception_oracle");
    read_optional(j, "max_range", c.max_range);
    read_optional(j, "sigma", c.sigma);
    read_optional(j, "sigma_per_meter", c.sigma_per_meter);
}

void to_json(json& j, const RunConfig& c) {
    j = {{"name", c.name},
         {"track", c.track},
         {"vehicle", c.vehicle},
         {"nominal", c.nominal},
         {"noise", c.noise},
         {"sensor_noise_scale", c.sensor_noise_scale},
         {"friction_scale", c.friction_scale},
         {"latency", c.latency},
         {"control_period", c.control_period},
         {"seed", c.seed},
         {"oracles", {{"perception", c.oracles.perception}, {"state", c.oracles.state}}},
         {"perception_oracle", c.perception_oracle},
         {"stack", c.stack},
         {"dnf", c.dnf},
         {"log_period", c.log_period}};
}

void from_json(const json& j, RunConfig& c) {
    reject_unknown_keys(j,
                        {"name", "track", "vehicle", "nominal", "noise", "sensor_noise_scale",
                         "friction_scale", "latency", "control_period", "seed", "oracles",
                         "perception_oracle", "stack", "dnf", "log_period"},
                        "run");
    read_optional(j, "name", c.name);
    read_optional(j, "track", c.track);
    read_optional(j, "vehicle", c.vehicle);
    c.nominal = c.vehicle;
    read_optional(j, "nominal", c.nominal);
    read_optional(j, "noise", c.noise);
    read_optional(j, "sensor_noise_scale", c.sensor_noise_scale);
    read_optional(j, "friction_scale", c.friction_scale);
    read_optional(j, "latency", c.latency);
    read_optional(j, "control_period", c.control_period);
    read_optional(j, "seed", c.seed);
    if (const auto it = j.find("oracles"); it != j.end()) {
        reject_unknown_keys(*it, {"perception", "state"}, "oracles");
        read_optional(*it, "perception", c.oracles.perception);
        read_optional(*it, "state", c.oracles.state);
    }
    read_optional(j, "perception_oracle", c.perception_oracle);
    read_optional(j, "stack", c.stack);
    read_optional(j, "dnf", c.dnf);
    read_optional(j, "log_period", c.log_period);
}

std::int64_t to_ticks(double seconds, const char* what) {
    constexpr double kTick = 1e-3;
    const double ticks = seconds / kTick;
    const double rounded = std::round(ticks);
    if (!(seconds >= 0.0) || std::abs(ticks - rounded) > 1e-6) {
        throw std::invalid_argument(std::string(what) + " must be a non-negative multiple of 1 ms");
    }
    return static_cast<std::int64_t>(rounded);
}

void validate(const RunConfig& c) {
    (void)to_ticks(c.latency.imu, "latency.imu");
    (void)to_ticks(c.latency.wheels, "latency.wheels");
    (void)to_ticks(c.latency.steering, "latency.steering");
    (void)to_ticks(c.latency.camera, "latency.camera");
    (void)to_ticks(c.latency.lidar, "latency.lidar");
    if (to_ticks(c.control_period, "control_period") == 0) {
        throw std::invalid_argument("control_period must be at least 1 ms");
    }
    sim::validate_noise(c.noise);
    if (!(c.sensor_noise_scale >= 0.0) || !std::isfinite(c.sensor_noise_scale)) {
        throw std::invalid_argument("sensor_noise_scale must be finite and non-negative");
    }
    if (!(c.friction_scale > 0.0) || !std::isfinite(c.friction_scale)) {
        throw std::invalid_argument("friction_scale must be finite and positive");
    }
    if (!(c.dnf.stall_speed >= 0.0) || !(c.dnf.stall_time > 0.0) || !(c.dnf.max_off_track > 0.0) ||
        !(c.dnf.time_limit > 0.0) || !(c.dnf.stop_timeout > 0.0)) {
        throw std::invalid_argument("dnf: thresholds must be positive");
    }
    if (!(c.perception_oracle.max_range > 0.0) || !(c.perception_oracle.sigma >= 0.0) ||
        !(c.perception_oracle.sigma_per_meter >= 0.0)) {
        throw std::invalid_argument("perception_oracle: range and noise must be non-negative");
    }
    if (!(c.log_period > 0.0)) throw std::invalid_argument("log_period must be positive");
    if (c.track.source == TrackSpec::Source::Oval &&
        (!(c.track.oval_straight > 0.0) || !(c.track.oval_radius > 0.0))) {
        throw std::invalid_argument("track.oval: lengths must be positive");
    }
}

}  // namespace apex::runner
