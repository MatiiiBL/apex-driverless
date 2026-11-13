// Evaluation runs and latency benchmark of the estimation module, quoted in docs/estimation.md.
//
//   apex_estimation_bench [--out DIR] [--quick]
//
// Drives generated tracks lap 1 with mapping on and laps 2-3 with it off, through the
// simulator's vehicle and sensor models (World) and through the faster synthetic drive, and
// times the cone update against the number of landmarks. Writes DIR/eval.json,
// DIR/latency.json and DIR/summary.md (default DIR: results/estimation). Release build only.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/angles.hpp"
#include "apex/core/log.hpp"
#include "apex/core/timing.hpp"
#include "apex/core/track.hpp"
#include "apex/estimation/estimator.hpp"
#include "apex/estimation/eval/batch_slam.hpp"
#include "apex/estimation/eval/consistency.hpp"
#include "apex/estimation/eval/map_metrics.hpp"
#include "apex/estimation/eval/synthetic.hpp"
#include "apex/estimation/eval/world_drive.hpp"
#include "apex/sim/track_gen.hpp"

namespace {

using apex::ConeMap;
using apex::ConeObservations;
using apex::ImuSample;
using apex::LatencyStats;
using apex::Mat3;
using apex::SteeringSample;
using apex::Stopwatch;
using apex::Time;
using apex::Track;
using apex::Vec3;
using apex::VehicleParams;
using apex::WheelSpeeds;
using nlohmann::json;
namespace est = apex::estimation;
namespace eval = apex::estimation::eval;

// Position and yaw errors and the pose NEES over one phase of a run.
class PhaseScore {
public:
    void add(const apex::StateEstimate& e, const eval::TruthSample& t, bool sample_nees) {
        const Vec3 error(e.pose.x - t.pose.x, e.pose.y - t.pose.y,
                         apex::angle_diff(e.pose.yaw, t.pose.yaw));
        position_ += error.head<2>().squaredNorm();
        yaw_ += error.z() * error.z();
        max_position_ = std::max(max_position_, error.head<2>().norm());
        ++count_;
        if (sample_nees) nees_.add(eval::nees(error, e.pose_covariance));
    }
    [[nodiscard]] json summary() const {
        const double n = std::max(1.0, static_cast<double>(count_));
        const eval::NeesSummary s = nees_.summary();
        return {{"position_rms_m", std::sqrt(position_ / n)},
                {"position_max_m", max_position_},
                {"yaw_rms_deg", apex::rad_to_deg(std::sqrt(yaw_ / n))},
                {"pose_nees_mean", s.mean},
                {"pose_nees_samples", s.samples}};
    }

private:
    double position_ = 0.0;
    double yaw_ = 0.0;
    double max_position_ = 0.0;
    std::size_t count_ = 0;
    eval::NeesAccumulator nees_{3};
};

std::vector<std::int32_t> mappable_by(const eval::SyntheticRun& run, Time t) {
    std::vector<std::int32_t> ids;
    for (std::size_t i = 0; i < run.mappable_since.size(); ++i) {
        if (run.mappable_since[i] < t) ids.push_back(static_cast<std::int32_t>(i));
    }
    return ids;
}

json map_json(const eval::MapMetrics& m) {
    return {{"landmarks", m.landmarks},   {"matched_cones", m.matched_cones},
            {"duplicates", m.duplicates}, {"spurious", m.spurious},
            {"missed", m.missed},         {"rms_error_m", m.rms_error},
            {"max_error_m", m.max_error}};
}

// The batch least-squares map over lap 1, with the true associations: the reference error.
double batch_map_rms(const eval::SyntheticRun& run, const Track& track,
                     const VehicleParams& nominal, Time until,
                     const std::vector<std::int32_t>& mappable) {
    eval::BatchRecorder recorder({}, nominal, run.truth.front().pose);
    for (const eval::TimedMessage& m : run.messages) {
        if (m.delivery >= until) break;
        eval::deliver(recorder, m);
    }
    const eval::BatchSolution solution = eval::solve_batch_slam(recorder.problem());
    ConeMap map;
    for (const auto& [key, position] : solution.landmarks) {
        apex::MapCone cone;
        cone.id = key;
        cone.truth_id = key;
        cone.position = position;
        if (std::binary_search(mappable.begin(), mappable.end(), key)) map.cones.push_back(cone);
    }
    return eval::evaluate_map(map, track, mappable).rms_error;
}

// One run: lap 1 mapping, then localization one second into lap 2 (as the stack switches once
// it has closed the loop).
json evaluate(const std::string& source, std::uint64_t seed, const Track& track,
              const eval::SyntheticRun& run, const VehicleParams& nominal) {
    est::StateEstimator estimator({}, nominal, apex::SensorRig{}, run.truth.front().pose);
    est::VelocityFilter velocity({}, nominal);
    const Time switch_time = run.lap1_end + 1.0;
    PhaseScore lap1;
    PhaseScore racing;
    eval::AssociationScore association;
    eval::NeesAccumulator velocity_nees(3);
    double velocity_sum[3] = {};
    double velocity_count = 0.0;
    Time next_pose_nees = 1.0;
    Time next_velocity_nees = 2.0;
    ConeMap lap1_map;
    for (const eval::TimedMessage& m : run.messages) {
        if (estimator.mapping() && m.delivery >= switch_time) {
            lap1_map = estimator.map();
            estimator.set_mapping(false);
        }
        if (const auto* cones = std::get_if<ConeObservations>(&m.message)) {
            const ConeMap before = estimator.map();
            estimator.on_cones(*cones);
            association.add(*cones, estimator.last_associations(), before, estimator.map());
            const apex::StateEstimate e = estimator.estimate();
            const bool sample = e.stamp >= next_pose_nees;
            if (sample) next_pose_nees += 1.0;
            (estimator.mapping() ? lap1 : racing).add(e, run.truth_at(e.stamp), sample);
            continue;
        }
        eval::deliver(estimator, m);
        if (const auto* imu = std::get_if<ImuSample>(&m.message)) {
            velocity.predict(*imu);
            if (imu->stamp < 2.0) continue;
            const eval::TruthSample& t = run.truth_at(imu->stamp);
            const Vec3 error(velocity.vx() - t.vx, velocity.vy() - t.vy,
                             velocity.yaw_rate() - t.yaw_rate);
            for (int i = 0; i < 3; ++i) velocity_sum[i] += error[i] * error[i];
            velocity_count += 1.0;
            if (imu->stamp >= next_velocity_nees) {
                velocity_nees.add(eval::nees(error, velocity.velocity_covariance()));
                next_velocity_nees += 2.0;
            }
        } else if (const auto* wheels = std::get_if<WheelSpeeds>(&m.message)) {
            velocity.update_wheels(*wheels);
        } else if (const auto* steering = std::get_if<SteeringSample>(&m.message)) {
            velocity.update_steering(*steering);
        }
    }
    const std::vector<std::int32_t> mappable = mappable_by(run, run.lap1_end - 2.0);
    const eval::MapMetrics map = eval::evaluate_map(lap1_map, track, mappable);
    const est::EstimatorStats& stats = estimator.stats();
    const eval::NeesSummary vn = velocity_nees.summary();
    const eval::NeesBounds vb = eval::nees_bounds(3, std::max<std::size_t>(vn.samples, 1));
    const double n = std::max(1.0, velocity_count);
    return {
        {"source", source},
        {"seed", seed},
        {"track_length_m", track.centerline.length()},
        {"cones", track.cones.size()},
        {"duration_s", run.truth.back().stamp},
        {"lap1_end_s", run.lap1_end},
        {"map", map_json(map)},
        {"batch_map_rms_m", batch_map_rms(run, track, nominal, switch_time, mappable)},
        {"lap1", lap1.summary()},
        {"racing", racing.summary()},
        {"association",
         {{"precision", association.precision()},
          {"recall", association.recall()},
          {"matches", association.matches()}}},
        {"velocity",
         {{"vx_rms", std::sqrt(velocity_sum[0] / n)},
          {"vy_rms", std::sqrt(velocity_sum[1] / n)},
          {"yaw_rate_rms", std::sqrt(velocity_sum[2] / n)},
          {"nees_mean", vn.mean},
          {"nees_samples", vn.samples},
          {"nees_bounds", {vb.lower, vb.upper}}}},
        {"stats",
         {{"recoveries", stats.recoveries},
          {"ambiguous_scans", stats.ambiguous_scans},
          {"search_exhausted", stats.search_exhausted},
          {"wheel_outliers", stats.wheel_outliers},
          {"tentatives_dropped", stats.tentatives_dropped},
          {"failed_updates", stats.failed_updates}}},
        {"update_time", stats.update_time.summary()},
        {"predict_time", stats.predict_time.summary()},
    };
}

eval::DriveProfile three_laps() {
    eval::DriveProfile profile;
    profile.laps = 3;
    return profile;
}

// The simulator, driven by the pure-pursuit test driver at the racing limits it holds.
json world_run(std::uint64_t seed) {
    const Track track = apex::sim::generate_track(seed);
    apex::sim::WorldConfig config;
    config.seed = seed;
    eval::DriveProfile profile = three_laps();
    profile.racing_lateral_accel = 10.0;
    profile.max_brake = 8.0;
    profile.max_accel = 6.0;
    const eval::SyntheticRun run = eval::make_world_run(track, config, profile, {});
    return evaluate("world", seed, track, run, config.vehicle);
}

// The kinematic synthetic drive at full racing speed: 22 m/s, 14 m/s^2 in the corners.
json synthetic_run(std::uint64_t seed) {
    const Track track = apex::sim::generate_track(seed);
    const eval::SyntheticRun run =
        eval::make_synthetic_run(track, VehicleParams{}, three_laps(), {}, seed);
    return evaluate("synthetic", seed, track, run, VehicleParams{});
}

// Cone update and IMU prediction cost against the landmark count, on an oval long enough for
// 400 cones, mapped at exploration speed. Each sample is binned by the map size before the call.
json latency() {
    const Track track = apex::make_oval_track(340.0, 45.0);
    eval::DriveProfile profile;
    profile.exploration_speed = 10.0;
    const eval::SyntheticRun run = eval::make_synthetic_run(track, VehicleParams{}, profile, {}, 7);
    est::StateEstimator estimator({}, VehicleParams{}, apex::SensorRig{}, run.truth.front().pose);
    constexpr std::size_t kBin = 50;
    std::map<std::size_t, LatencyStats> update;
    std::map<std::size_t, LatencyStats> predict;
    std::map<std::size_t, std::size_t> observations;
    for (const eval::TimedMessage& m : run.messages) {
        const std::size_t bin = estimator.map().cones.size() / kBin;
        if (const auto* cones = std::get_if<ConeObservations>(&m.message)) {
            const Stopwatch watch;
            estimator.on_cones(*cones);
            update[bin].add(watch.elapsed_seconds());
            observations[bin] += cones->cones.size();
        } else if (const auto* imu = std::get_if<ImuSample>(&m.message)) {
            const Stopwatch watch;
            estimator.on_imu(*imu);
            predict[bin].add(watch.elapsed_seconds());
        } else {
            eval::deliver(estimator, m);
        }
    }
    json bins = json::array();
    for (const auto& [bin, stats] : update) {
        bins.push_back({{"landmarks_from", bin * kBin},
                        {"landmarks_to", (bin + 1) * kBin},
                        {"observations_per_scan", static_cast<double>(observations[bin]) /
                                                      static_cast<double>(stats.count())},
                        {"update", stats.summary()},
                        {"predict", predict[bin].summary()}});
    }
    return {{"track_cones", track.cones.size()},
            {"final_landmarks", estimator.map().cones.size()},
            {"bins", bins}};
}

std::string fixed(double value, int digits) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.*f", digits, value);
    return buffer;
}

std::string summary_markdown(const json& runs, const json& timing) {
    std::string md =
        "# Estimation results\n\n"
        "Produced by `build/release/bench/estimation/apex_estimation_bench` "
        "(docs/estimation.md explains the setup). Lap 1 maps, laps 2-3 localize against the "
        "frozen map. `world`: the simulator's vehicle and IMU/wheel/steering models with the "
        "pure-pursuit test driver (racing at 10 m/s^2 lateral); `synthetic`: the kinematic "
        "drive at full racing speed (22 m/s, 14 m/s^2). Cone observations come from the true "
        "cones through the synthetic perception model in both.\n\n"
        "| source | seed | cones | map RMS (m) | batch RMS (m) | dup | missed | pose RMS lap 1 (m) "
        "| pose RMS racing (m) | yaw RMS racing (deg) | pose NEES lap 1 / racing | assoc. "
        "precision | recall | velocity NEES | recoveries |\n"
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n";
    for (const json& r : runs) {
        md += "| " + r["source"].get<std::string>() + " | " + std::to_string(r["seed"].get<int>()) +
              " | " + std::to_string(r["cones"].get<int>()) + " | " +
              fixed(r["map"]["rms_error_m"], 3) + " | " + fixed(r["batch_map_rms_m"], 3) + " | " +
              std::to_string(r["map"]["duplicates"].get<int>()) + " | " +
              std::to_string(r["map"]["missed"].get<int>()) + " | " +
              fixed(r["lap1"]["position_rms_m"], 3) + " | " +
              fixed(r["racing"]["position_rms_m"], 3) + " | " +
              fixed(r["racing"]["yaw_rms_deg"], 2) + " | " + fixed(r["lap1"]["pose_nees_mean"], 2) +
              " / " + fixed(r["racing"]["pose_nees_mean"], 2) + " | " +
              fixed(r["association"]["precision"], 4) + " | " +
              fixed(r["association"]["recall"], 4) + " | " + fixed(r["velocity"]["nees_mean"], 2) +
              " | " + std::to_string(r["stats"]["recoveries"].get<int>()) + " |\n";
    }
    md +=
        "\nPose NEES is 3-D (expected 3); velocity NEES over (vx, vy, yaw rate), expected 3.\n\n"
        "## Latency against landmark count\n\n"
        "One cone update (association and EKF update) and one IMU prediction, wall clock, "
        "Release build. A synthetic drive maps an oval of " +
        std::to_string(timing["track_cones"].get<int>()) +
        " cones at 10 m/s.\n\n"
        "| landmarks | obs/scan | update p50 (ms) | update p99 (ms) | predict p50 (us) | "
        "predict p99 (us) |\n|---|---|---|---|---|---|\n";
    for (const json& b : timing["bins"]) {
        md += "| " + std::to_string(b["landmarks_from"].get<int>()) + "-" +
              std::to_string(b["landmarks_to"].get<int>()) + " | " +
              fixed(b["observations_per_scan"], 1) + " | " +
              fixed(b["update"]["p50_us"].get<double>() / 1000.0, 3) + " | " +
              fixed(b["update"]["p99_us"].get<double>() / 1000.0, 3) + " | " +
              fixed(b["predict"]["p50_us"], 2) + " | " + fixed(b["predict"]["p99_us"], 2) + " |\n";
    }
    return md;
}

}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path out = "results/estimation";
    bool quick = false;
    for (int i = 1; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--out" && i + 1 < argc) {
            out = argv[++i];
        } else if (arg == "--quick") {
            quick = true;
        } else {
            std::fprintf(stderr, "usage: %s [--out DIR] [--quick]\n", argv[0]);
            return 2;
        }
    }
    apex::set_log_level(apex::LogLevel::Warn);
    const std::vector<std::uint64_t> seeds =
        quick ? std::vector<std::uint64_t>{1} : std::vector<std::uint64_t>{1, 2, 3, 4, 5};
    json runs = json::array();
    for (const std::uint64_t seed : seeds) {
        runs.push_back(world_run(seed));
        std::fprintf(stderr, "world %llu done\n", static_cast<unsigned long long>(seed));
    }
    for (const std::uint64_t seed : seeds) {
        runs.push_back(synthetic_run(seed));
        std::fprintf(stderr, "synthetic %llu done\n", static_cast<unsigned long long>(seed));
    }
    const json timing = latency();
    std::filesystem::create_directories(out);
    std::ofstream(out / "eval.json") << runs.dump(2) << "\n";
    std::ofstream(out / "latency.json") << timing.dump(2) << "\n";
    std::ofstream(out / "summary.md") << summary_markdown(runs, timing);
    std::printf("%s\n", json{{"runs", runs}, {"latency", timing}}.dump().c_str());
    return 0;
}
