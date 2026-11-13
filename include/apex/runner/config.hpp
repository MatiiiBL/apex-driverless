#pragma once

#include <cstdint>
#include <string>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/vehicle.hpp"
#include "apex/sim/sensors.hpp"
#include "apex/stack/driver.hpp"
#include "apex/stack/stack_config.hpp"

namespace apex::runner {

// Where a run's track comes from. JSON: exactly one of
//   {"seed": 7}                          the track generator (sim/track_gen.hpp)
//   {"layout": "name"}                   one of the generator's fixed reference layouts
//   {"file": "track.json"}               a saved track (save_track), relative to the cwd
//   {"oval": {"straight": 40, "radius": 12}}
struct TrackSpec {
    enum class Source { Oval, Generated, Layout, File };
    Source source = Source::Oval;
    std::uint64_t seed = 1;
    std::string layout;
    std::string file;
    double oval_straight = 40.0;  // m
    double oval_radius = 12.0;    // m
};

void to_json(nlohmann::json& j, const TrackSpec& spec);
void from_json(const nlohmann::json& j, TrackSpec& spec);

// Processing latency per message type: a message reaches the car's software this long after its
// stamp (docs/design.md, section 4.4). Each must be a whole number of 1 ms ticks.
struct Latencies {
    double imu = 0.0;
    double wheels = 0.0;
    double steering = 0.0;
    double camera = 0.040;  // the cone detector
    double lidar = 0.050;   // perception
};

void to_json(nlohmann::json& j, const Latencies& latencies);
void from_json(const nlohmann::json& j, Latencies& latencies);

// When a run is over before the mission is (did not finish), per docs/design.md, section 1.
struct DnfRules {
    double stall_speed = 0.5;    // m/s: slower than this counts as stalled ...
    double stall_time = 10.0;    // s: ... for this long
    double max_off_track = 5.0;  // m beyond a cone line
    double time_limit = 600.0;   // s of simulated time
    // After the last lap the run ends once the car is at rest or the driver reports Finished;
    // a car still rolling this long after the line has not stopped (RunResult::stopped).
    double stop_timeout = 30.0;  // s
};

void to_json(nlohmann::json& j, const DnfRules& rules);
void from_json(const nlohmann::json& j, DnfRules& rules);

// The perfect-perception oracle's observations: every true cone within range and the LiDAR's
// field of view, no occlusion, true colors, with isotropic Gaussian noise of standard deviation
// sigma + sigma_per_meter * range, which is also the covariance reported.
struct PerceptionOracleConfig {
    double max_range = 25.0;         // m from the LiDAR
    double sigma = 0.03;             // m
    double sigma_per_meter = 0.004;  // m per m of range
};

void to_json(nlohmann::json& j, const PerceptionOracleConfig& config);
void from_json(const nlohmann::json& j, PerceptionOracleConfig& config);

// Everything that defines one closed-loop run. JSON keys match the field names; missing keys
// keep their defaults and unknown keys are rejected. "nominal" starts from "vehicle" (the
// software knows the true car unless told otherwise) and applies its own keys on top.
struct RunConfig {
    std::string name;  // a label for results and sweeps
    TrackSpec track;
    VehicleParams vehicle;   // the true car, in the simulator
    VehicleParams nominal;   // the car's software's model of it
    sim::SensorNoise noise;  // the simulator's sensor noise models (docs/sim.md)
    // Multiplies every noise standard deviation and bias of `noise` (sim::scale_noise).
    double sensor_noise_scale = 1.0;
    double friction_scale = 1.0;  // the track's grip relative to the tires' nominal mu
    Latencies latency;
    double control_period = 0.02;  // s, a whole number of ticks
    std::uint64_t seed = 1;        // every random draw of the run derives from it
    stack::Oracles oracles;
    PerceptionOracleConfig perception_oracle;
    stack::StackConfig stack;  // module configs, controller, mission (laps)
    DnfRules dnf;
    double log_period = 0.1;  // s between rows of the CSV time series
};

void to_json(nlohmann::json& j, const RunConfig& config);
void from_json(const nlohmann::json& j, RunConfig& config);

// Throws std::invalid_argument naming the first invalid setting.
void validate(const RunConfig& config);

// A latency or period in whole 1 ms ticks; throws std::invalid_argument if it is negative or
// not a whole number of ticks.
[[nodiscard]] std::int64_t to_ticks(double seconds, const char* what);

}  // namespace apex::runner
