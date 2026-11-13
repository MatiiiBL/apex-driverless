// Closed-loop evaluation of the three controllers on the simulator's vehicle with the true
// state as the estimate (docs/control.md, "Evaluation").
//
// Reference: each track's centerline with a speed profile at 85% of the nominal car's static
// grip (references.hpp). Each run starts at rest at the staging position and drives two laps;
// the RaceMonitor times them and counts cones knocked and off-course excursions. Metrics come
// from the second, flying lap: lateral error from the centerline (RMS, max), steering
// smoothness (RMS of the commanded steering rate), time with the CoG outside the lateral
// bounds the MPC uses, and the controller's compute time per call.
//
// Sweeps on the three reference layouts, the controllers keeping the nominal parameters:
// actuator dead time of the true car, friction below nominal, and model mismatch (a heavier
// car, softer tires).
//
//   apex_control_closed_loop_eval [--out results/control] [--seeds 5]
//
// Writes closed_loop.json, closed_loop.md and a per-call trace of each controller on the
// "mixed" layout (trace_<controller>.csv) to the output directory.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/control/controller.hpp"
#include "apex/core/timing.hpp"
#include "apex/sim/race.hpp"
#include "apex/sim/track_gen.hpp"

#include "closed_loop.hpp"
#include "plant.hpp"
#include "references.hpp"

namespace {

using namespace apex;
using namespace apex::control;
using nlohmann::json;

constexpr int kLaps = 2;
constexpr double kDnfDistance = 5.0;  // m from the centerline (docs/design.md, section 1)

struct Scenario {
    std::string track_name;
    const Track* track = nullptr;
    std::string sweep = "base";
    double value = 0.0;
    VehicleParams truth;
    double friction_scale = 1.0;
};

struct Outcome {
    bool finished = false;
    std::string dnf;
    std::vector<double> laps;
    double profile_time = 0.0;  // the reference's own lap time
    double lateral_rms = 0.0;
    double lateral_max = 0.0;
    double steer_rate_rms = 0.0;
    double outside_bounds = 0.0;  // s with the CoG beyond the MPC's lateral bounds
    std::size_t cones = 0;
    std::size_t off_course = 0;
    double penalty = 0.0;
    ControllerStats stats;
    // Process CPU time per call: unlike the wall-clock compute time, it does not count the
    // time the scheduler gives other processes on a loaded machine.
    LatencyStats cpu_time;
};

struct TraceRow {
    Time time;
    double s, lateral, speed, reference_speed, steer, accel;
};

Outcome run(ControllerKind kind, const Scenario& scenario, std::vector<TraceRow>* trace) {
    const VehicleParams nominal;
    Trajectory reference = scenario.track->centerline;
    testing::apply_speed_limits(reference, testing::grip_limited(nominal));
    ControllerConfig config;
    config.kind = kind;
    const std::unique_ptr<Controller> controller = make_controller(config, nominal);
    testing::Plant plant(scenario.truth, scenario.track->start_pose, 0.0, scenario.friction_scale);
    sim::RaceMonitor monitor(*scenario.track, scenario.truth);
    plant.attach(monitor);

    Outcome out;
    for (std::size_t i = 0; i < reference.size(); ++i) {
        const double next = i + 1 < reference.size() ? reference[i + 1].s : reference.length();
        out.profile_time += (next - reference[i].s) / std::max(reference[i].speed, 0.1);
    }
    const double inset = 0.5 * nominal.body_width + config.mpc.lateral_margin;
    const double timeout = kLaps * out.profile_time + 30.0;
    double sum_lateral = 0.0;
    double sum_rate = 0.0;
    std::size_t samples = 0;
    double previous_steer = 0.0;
    while (monitor.laps_completed() < kLaps) {
        const Time now = plant.time();
        if (now > timeout) {
            out.dnf = "timeout";
            break;
        }
        const StateEstimate estimate = plant.estimate();
        const std::clock_t cpu_start = std::clock();
        const ControlCommand command = controller->compute(estimate, reference, now);
        out.cpu_time.add(static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC);
        plant.command(command);
        const bool flying = monitor.laps_completed() == kLaps - 1;
        if (flying) {
            const double lateral = monitor.lateral();
            const TrajectoryPoint here = reference.at(monitor.s());
            const double rate = (command.steer - previous_steer) / testing::kControlPeriod;
            sum_lateral += lateral * lateral;
            sum_rate += rate * rate;
            ++samples;
            out.lateral_max = std::max(out.lateral_max, std::abs(lateral));
            if (lateral > here.width_left - inset || -lateral > here.width_right - inset) {
                out.outside_bounds += testing::kControlPeriod;
            }
        }
        if (trace != nullptr && monitor.started()) {
            const TrajectoryPoint here = reference.at(monitor.s());
            trace->push_back({now, monitor.s(), monitor.lateral(), plant.truth().vx, here.speed,
                              command.steer, command.accel});
        }
        previous_steer = command.steer;
        plant.advance(testing::kControlPeriod);
        if (std::abs(monitor.lateral()) > kDnfDistance) {
            out.dnf = "left the track";
            break;
        }
    }
    out.finished = out.dnf.empty();
    for (const sim::LapRecord& lap : monitor.laps()) out.laps.push_back(lap.time());
    if (samples > 0) {
        out.lateral_rms = std::sqrt(sum_lateral / static_cast<double>(samples));
        out.steer_rate_rms = std::sqrt(sum_rate / static_cast<double>(samples));
    }
    out.cones = monitor.cone_hits().size();
    out.off_course = monitor.off_course().size();
    out.penalty = monitor.penalty_seconds();
    out.stats = controller->stats();
    return out;
}

json to_json_outcome(ControllerKind kind, const Scenario& s, const Outcome& o) {
    json j;
    j["controller"] = std::string(to_string(kind));
    j["track"] = s.track_name;
    j["sweep"] = s.sweep;
    j["value"] = s.value;
    j["finished"] = o.finished;
    j["dnf"] = o.dnf;
    j["laps"] = o.laps;
    j["profile_lap_time"] = o.profile_time;
    j["lateral_rms"] = o.lateral_rms;
    j["lateral_max"] = o.lateral_max;
    j["steer_rate_rms"] = o.steer_rate_rms;
    j["outside_bounds_s"] = o.outside_bounds;
    j["cones"] = o.cones;
    j["off_course"] = o.off_course;
    j["penalty_s"] = o.penalty;
    j["solver_failures"] = o.stats.solver_failures;
    j["calls"] = o.stats.calls;
    j["compute"] = o.stats.compute_time.summary();
    j["compute_cpu"] = o.cpu_time.summary();
    return j;
}

std::string fmt(const char* format, double value) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), format, value);
    return buffer;
}

std::string lap_cell(const json& r) {
    if (!r.at("finished").get<bool>()) return "DNF (" + r.at("dnf").get<std::string>() + ")";
    return fmt("%.2f", r.at("laps").back().get<double>());
}

std::string base_table(const json& runs) {
    std::string md =
        "| track | controller | flying lap (s) | profile (s) | lateral RMS / max (m) | steer "
        "rate RMS (rad/s) | outside bounds (s) | cones | off course | compute p50 / p99 (us) "
        "| CPU p50 / p99 (us) | QP failures |\n|---|---|---|---|---|---|---|---|---|---|---|---|\n";
    for (const json& r : runs) {
        if (r.at("sweep") != "base") continue;
        md += "| " + r.at("track").get<std::string>() + " | " +
              r.at("controller").get<std::string>() + " | " + lap_cell(r) + " | " +
              fmt("%.2f", r.at("profile_lap_time").get<double>()) + " | " +
              fmt("%.3f", r.at("lateral_rms").get<double>()) + " / " +
              fmt("%.2f", r.at("lateral_max").get<double>()) + " | " +
              fmt("%.2f", r.at("steer_rate_rms").get<double>()) + " | " +
              fmt("%.2f", r.at("outside_bounds_s").get<double>()) + " | " +
              std::to_string(r.at("cones").get<int>()) + " | " +
              std::to_string(r.at("off_course").get<int>()) + " | " +
              fmt("%.0f", r.at("compute").at("p50_us").get<double>()) + " / " +
              fmt("%.0f", r.at("compute").at("p99_us").get<double>()) + " | " +
              fmt("%.0f", r.at("compute_cpu").at("p50_us").get<double>()) + " / " +
              fmt("%.0f", r.at("compute_cpu").at("p99_us").get<double>()) + " | " +
              std::to_string(r.at("solver_failures").get<int>()) + " |\n";
    }
    return md;
}

// Per sweep value and controller: the mean over the reference layouts.
std::string sweep_table(const json& runs, const std::string& sweep, const char* value_format) {
    struct Sum {
        int runs = 0, finished = 0;
        double lap = 0.0, rms = 0.0, max = 0.0, penalty = 0.0;
    };
    std::map<std::pair<double, std::string>, Sum> sums;
    for (const json& r : runs) {
        if (r.at("sweep") != sweep) continue;
        Sum& s = sums[{r.at("value").get<double>(), r.at("controller").get<std::string>()}];
        ++s.runs;
        if (r.at("finished").get<bool>()) {
            ++s.finished;
            s.lap += r.at("laps").back().get<double>();
            s.rms += r.at("lateral_rms").get<double>();
            s.max = std::max(s.max, r.at("lateral_max").get<double>());
            s.penalty += r.at("penalty_s").get<double>();
        }
    }
    std::string md =
        "| " + sweep +
        " | controller | finished | mean flying lap (s) | mean lateral RMS (m) | worst lateral "
        "max (m) | penalties (s) |\n|---|---|---|---|---|---|---|\n";
    for (const auto& [key, s] : sums) {
        const double n = std::max(s.finished, 1);
        md += "| " + fmt(value_format, key.first) + " | " + key.second + " | " +
              std::to_string(s.finished) + "/" + std::to_string(s.runs) + " | " +
              (s.finished > 0 ? fmt("%.2f", s.lap / n) : "-") + " | " +
              (s.finished > 0 ? fmt("%.3f", s.rms / n) : "-") + " | " +
              (s.finished > 0 ? fmt("%.2f", s.max) : "-") + " | " + fmt("%.0f", s.penalty) + " |\n";
    }
    return md;
}

void write_trace(const std::filesystem::path& path, const std::vector<TraceRow>& rows) {
    std::ofstream file(path);
    file << "time,s,lateral,speed,reference_speed,steer,accel\n";
    for (const TraceRow& r : rows) {
        char line[256];
        std::snprintf(line, sizeof(line), "%.2f,%.3f,%.4f,%.3f,%.3f,%.4f,%.3f\n", r.time, r.s,
                      r.lateral, r.speed, r.reference_speed, r.steer, r.accel);
        file << line;
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path out = "results/control";
    int seeds = 5;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string_view flag = argv[i];
        if (flag == "--out") {
            out = argv[i + 1];
        } else if (flag == "--seeds") {
            seeds = std::atoi(argv[i + 1]);
        } else {
            std::fprintf(stderr, "usage: %s [--out dir] [--seeds n]\n", argv[0]);
            return 2;
        }
    }
    std::vector<std::pair<std::string, Track>> tracks;
    for (const std::string& name : sim::reference_track_names()) {
        tracks.emplace_back(name, sim::reference_track(name));
    }
    const std::size_t layouts = tracks.size();
    for (int seed = 1; seed <= seeds; ++seed) {
        tracks.emplace_back("seed " + std::to_string(seed),
                            sim::generate_track(static_cast<std::uint64_t>(seed)));
    }

    std::vector<Scenario> scenarios;
    for (const auto& [name, track] : tracks) {
        Scenario s;
        s.track_name = name;
        s.track = &track;
        scenarios.push_back(s);
    }
    for (std::size_t t = 0; t < layouts; ++t) {
        Scenario s;
        s.track_name = tracks[t].first;
        s.track = &tracks[t].second;
        for (const double dead : {0.06, 0.09}) {
            Scenario d = s;
            d.sweep = "steer dead time (s)";
            d.value = dead;
            d.truth.steer_dead_time = dead;
            scenarios.push_back(d);
        }
        for (const double friction : {0.9, 0.8}) {
            Scenario f = s;
            f.sweep = "friction scale";
            f.value = friction;
            f.friction_scale = friction;
            scenarios.push_back(f);
        }
        Scenario heavy = s;
        heavy.sweep = "model mismatch";
        heavy.value = 1.0;  // a heavier car: +20% mass and yaw inertia
        heavy.truth.mass *= 1.2;
        heavy.truth.yaw_inertia *= 1.2;
        scenarios.push_back(heavy);
        Scenario soft = s;
        soft.sweep = "model mismatch";
        soft.value = 2.0;  // softer tires: -25% cornering stiffness factor B
        soft.truth.front_tire.B *= 0.75;
        soft.truth.rear_tire.B *= 0.75;
        scenarios.push_back(soft);
    }

    json runs = json::array();
    std::filesystem::create_directories(out);
    for (const ControllerKind kind :
         {ControllerKind::PurePursuit, ControllerKind::Stanley, ControllerKind::Mpc}) {
        for (const Scenario& s : scenarios) {
            std::vector<TraceRow> trace;
            const bool traced = s.sweep == "base" && s.track_name == "mixed";
            const Outcome o = run(kind, s, traced ? &trace : nullptr);
            runs.push_back(to_json_outcome(kind, s, o));
            std::fprintf(stderr, "%-12s %-10s %-20s %.2f: %s\n",
                         std::string(to_string(kind)).c_str(), s.track_name.c_str(),
                         s.sweep.c_str(), s.value,
                         o.finished ? fmt("%.2f s", o.laps.back()).c_str() : o.dnf.c_str());
            if (traced) {
                write_trace(out / ("trace_" + std::string(to_string(kind)) + ".csv"), trace);
            }
        }
    }
    std::ofstream(out / "closed_loop.json") << json{{"runs", runs}}.dump(2) << "\n";
    const std::string md =
        "# Closed-loop evaluation\n\n"
        "Simulator vehicle, true state as the estimate, reference = centerline with a speed "
        "profile at 85% of the nominal static grip; two laps from a standing start, metrics "
        "over the flying second lap. Compute time per call, one core.\n\n## Tracks\n\n" +
        base_table(runs) +
        "\n## Robustness (means over the three reference layouts; the controllers keep the "
        "nominal parameters)\n\n" +
        sweep_table(runs, "steer dead time (s)", "%.2f") + "\n" +
        sweep_table(runs, "friction scale", "%.1f") +
        "\nModel mismatch 1: mass and yaw inertia +20%. 2: tire stiffness factor B -25%.\n\n" +
        sweep_table(runs, "model mismatch", "%.0f");
    std::ofstream(out / "closed_loop.md") << md;
    std::printf("%s", md.c_str());
    return 0;
}
