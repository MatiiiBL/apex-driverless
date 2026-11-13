// Evaluates the perception pipeline on scans simulated at poses along the centerlines of
// generated tracks (sim::generate_track with seeds seed, seed + 1, ...): detection recall and
// precision, position error and NEES by range, false positives per scan, color coverage and
// accuracy, the big-cone classification, on_lidar latency, and the covariance model fitted to
// the errors (fit_variance_model). Writes eval.json and summary.md to the output directory.
// See docs/perception.md, "Evaluation".
//
//   apex_perception_eval [--tracks N] [--spacing M] [--seed S] [--out DIR]
//
// Scans and camera frames come from the simulator's sensor models (sim::simulate_lidar,
// sim::simulate_camera) with their default noise (docs/sim.md, "Sensors").
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/perception/calibration.hpp"
#include "apex/perception/detector.hpp"
#include "apex/perception/perception.hpp"
#include "apex/sim/sensors.hpp"
#include "apex/sim/track_gen.hpp"

#include "scoring.hpp"

namespace apex::perception::eval {
namespace {

using nlohmann::json;

// The car drives near the centerline (uniform lateral offset, Gaussian heading error) at up to
// 12 m/s, slowing so its lateral acceleration stays under 10 m/s^2: an exploration lap.
constexpr double kMaxSpeed = 12.0;
constexpr double kMaxLateralAccel = 10.0;
constexpr double kLateralSpread = 0.3;  // m
constexpr double kYawSpread = 0.03;     // rad
constexpr double kScanPeriod = 0.1;     // s
constexpr double kMatchGate = 0.5;      // m: farther from its cone, an observation is false

// How the camera frame fused with each scan is produced. The detection metrics and the latency
// come from the first variant: a frame 25 ms before the scan, the worst phase of a 20 Hz camera
// against a 10 Hz LiDAR. 50 ms is a dropped frame. The LiDAR pipeline is deterministic, so every
// variant reports the same cones; only their colors differ.
struct VariantSpec {
    const char* name;
    const char* description;
    double lag;        // s: the frame's stamp is this much before the scan's
    bool compensate;   // pass the yaw rate, or 0
    bool true_colors;  // the detector never confuses colors and reports no false positives
};
constexpr std::array<VariantSpec, 6> kVariants = {{
    {"lag25", "25 ms early, yaw compensated", 0.025, true, false},
    {"aligned", "at the scan's stamp", 0.0, true, false},
    {"lag25_uncompensated", "25 ms early, not compensated", 0.025, false, false},
    {"lag50", "50 ms early, yaw compensated", 0.050, true, false},
    {"lag50_uncompensated", "50 ms early, not compensated", 0.050, false, false},
    {"lag25_true_colors",
     "25 ms early, compensated; detector never confuses colors, no false positives", 0.025, true,
     true},
}};

sim::CameraNoise camera_noise(const VariantSpec& spec) {
    sim::CameraNoise noise;
    if (spec.true_colors) {
        noise.confusion_near = 0.0;
        noise.confusion_far = 0.0;
        noise.false_positives = 0.0;
    }
    return noise;
}

// The sensor models read only the pose.
VehicleState state_at(const Pose2& pose) {
    VehicleState state;
    state.pose = pose;
    return state;
}

struct Settings {
    int tracks = 8;
    double spacing = 2.5;  // m between scans along each centerline
    std::uint64_t seed = 1;
    std::filesystem::path out = "results/perception";
    std::string command = "build/release/bench/perception/apex_perception_eval";
};

Settings parse(int argc, char** argv) {
    Settings s;
    for (int i = 1; i < argc; ++i) s.command += std::string(" ") + argv[i];
    for (int i = 1; i < argc; i += 2) {
        const std::string_view key = argv[i];
        if (i + 1 >= argc) throw std::invalid_argument("missing value for " + std::string(key));
        const std::string value = argv[i + 1];
        if (key == "--tracks") {
            s.tracks = std::stoi(value);
        } else if (key == "--spacing") {
            s.spacing = std::stod(value);
        } else if (key == "--seed") {
            s.seed = std::stoull(value);
        } else if (key == "--out") {
            s.out = value;
        } else {
            throw std::invalid_argument("unknown option " + std::string(key));
        }
    }
    if (s.tracks < 1 || !(s.spacing > 0.0)) {
        throw std::invalid_argument("--tracks must be at least 1 and --spacing positive");
    }
    return s;
}

// The car at arc length s along the centerline, offset sideways and in heading; at earlier times
// it was farther back along the centerline, with the same offsets.
class Motion {
public:
    Motion(const Trajectory& centerline, double s, Rng& rng)
        : centerline_(&centerline),
          s_(s),
          offset_(rng.uniform(-kLateralSpread, kLateralSpread)),
          yaw_error_(rng.normal(0.0, kYawSpread)) {
        const double curvature = std::abs(centerline.at(s).curvature);
        speed_ = std::min(kMaxSpeed, std::sqrt(kMaxLateralAccel / std::max(curvature, 1e-6)));
    }

    // The pose `before` seconds before the scan.
    [[nodiscard]] Pose2 pose(double before = 0.0) const {
        const TrajectoryPoint p = centerline_->at(s_ - speed_ * before);
        const Vec2 position = p.position + offset_ * left_normal(heading_vector(p.yaw));
        return {position.x(), position.y(), wrap_angle(p.yaw + yaw_error_)};
    }
    // The mean yaw rate over the `lag` before the scan, as integrated from a gyro (rad/s).
    [[nodiscard]] double yaw_rate(double lag) const {
        if (!(lag > 0.0)) return speed_ * centerline_->at(s_).curvature;
        return angle_diff(pose().yaw, pose(lag).yaw) / lag;
    }

private:
    const Trajectory* centerline_;
    double s_;
    double offset_;
    double yaw_error_;
    double speed_ = 0.0;
};

// One camera variant's pipeline and color score.
struct Variant {
    const VariantSpec* spec;
    ConePerception perception;
    ColorScore color;
};

double ratio(double numerator, double denominator) {
    return denominator > 0.0 ? numerator / denominator : 0.0;
}

json bin_to_json(const DetectionBin& b, std::string label) {
    const auto tp = static_cast<double>(b.detected);
    const auto true_reported = static_cast<double>(b.reported - b.false_positives);
    return {{"range_m", std::move(label)},
            {"in_view", b.in_view},
            {"hit", b.hit},
            {"detected", b.detected},
            {"recall", ratio(tp, static_cast<double>(b.in_view))},
            {"recall_of_hit", ratio(tp, static_cast<double>(b.hit))},
            {"reported", b.reported},
            {"false_positives", b.false_positives},
            {"precision", ratio(true_reported, static_cast<double>(b.reported))},
            {"rms_error_m", std::sqrt(ratio(b.error_sq, tp))},
            {"rms_radial_m", std::sqrt(ratio(b.radial_sq, tp))},
            {"rms_tangential_m", std::sqrt(ratio(b.tangential_sq, tp))},
            {"nees", ratio(b.nees, tp)},
            {"nees_above_95", ratio(static_cast<double>(b.nees_above_95), tp)}};
}

json color_to_json(const Variant& v) {
    const ColorScore& c = v.color;
    return {
        {"name", v.spec->name},
        {"description", v.spec->description},
        {"lag_s", v.spec->lag},
        {"yaw_compensated", v.spec->compensate},
        {"visible", c.visible},
        {"colored", c.colored},
        {"correct", c.correct},
        {"coverage", ratio(static_cast<double>(c.colored), static_cast<double>(c.visible))},
        {"accuracy", ratio(static_cast<double>(c.correct), static_cast<double>(c.colored))},
        {"mean_confidence_correct", ratio(c.confidence_correct, static_cast<double>(c.correct))},
        {"mean_confidence_wrong",
         ratio(c.confidence_wrong, static_cast<double>(c.colored - c.correct))}};
}

std::string summary_markdown(const json& r) {
    std::string md = "# Perception evaluation\n\n";
    md += std::format(
        "`{}` (Release) over {} scans at poses along {} generated tracks; {}. "
        "Reproduce and read with docs/perception.md.\n\n",
        r["command"].get<std::string>(), r["scans"].get<int>(), r["tracks"].size(),
        r["sensor_model"].get<std::string>());
    md +=
        "| Range (m) | Cones in view | Recall | Recall of hit cones | Precision | "
        "Position RMS (cm) | Radial RMS (cm) | Tangential RMS (cm) | NEES | NEES > 5.99 |\n";
    md += "|---|---|---|---|---|---|---|---|---|---|\n";
    const auto row = [&](const json& b) {
        md += std::format(
            "| {} | {} | {:.3f} | {:.3f} | {:.4f} | {:.1f} | {:.1f} | {:.1f} | {:.2f} | {:.1f}% "
            "|\n",
            b["range_m"].get<std::string>(), b["in_view"].get<int>(), b["recall"].get<double>(),
            b["recall_of_hit"].get<double>(), b["precision"].get<double>(),
            100.0 * b["rms_error_m"].get<double>(), 100.0 * b["rms_radial_m"].get<double>(),
            100.0 * b["rms_tangential_m"].get<double>(), b["nees"].get<double>(),
            100.0 * b["nees_above_95"].get<double>());
    };
    for (const json& b : r["by_range"]) row(b);
    row(r["total"]);
    md += std::format(
        "\nFalse positives per scan: {:.4f}. NEES average: {:.2f} (2 expected), above the "
        "chi-square 95% bound in {:.1f}% of cases (5% expected).\n\n",
        r["false_positives_per_scan"].get<double>(), r["nees_mean"].get<double>(),
        100.0 * r["total"]["nees_above_95"].get<double>());

    md += "| Camera frame | Coverage | Accuracy | Mean confidence, right / wrong |\n";
    md += "|---|---|---|---|\n";
    for (const json& c : r["color"]) {
        md += std::format("| {} | {:.3f} | {:.4f} | {:.2f} / {:.2f} |\n",
                          c["description"].get<std::string>(), c["coverage"].get<double>(),
                          c["accuracy"].get<double>(), c["mean_confidence_correct"].get<double>(),
                          c["mean_confidence_wrong"].get<double>());
    }
    const json& big = r["big_cones"];
    md += std::format(
        "\nBig orange cones fitted as big: {} of {}; small cones fitted as big: {} of {}.\n\n",
        big["big_as_big"].get<int>(), big["big_cones"].get<int>(), big["small_as_big"].get<int>(),
        big["small_cones"].get<int>());
    const json& t = r["latency"];
    md += std::format("`on_lidar` latency: p50 {:.0f} us, p99 {:.0f} us, max {:.0f} us.\n\n",
                      t["p50_us"].get<double>(), t["p99_us"].get<double>(),
                      t["max_us"].get<double>());
    md += "Covariance model fitted to these errors (`fit_variance_model`):\n\n```json\n" +
          r["fitted_covariance"].dump(2) + "\n```\n";
    return md;
}

int run(const Settings& settings) {
    const SensorRig rig;
    const PerceptionConfig config;
    std::vector<Variant> variants;
    for (const VariantSpec& spec : kVariants) {
        variants.push_back({&spec, ConePerception(config, rig), {}});
    }
    ConeDetector detector(config.ground, config.clustering, config.cone_check, config.center_fit,
                          rig.lidar);

    const sim::SensorNoise noise;

    // The task's bins, and 2 m bins for the plots.
    RangeBins bins({10.0, 20.0, 30.0});
    std::vector<double> fine_edges;
    for (double edge = 2.0; edge <= rig.lidar.max_range; edge += 2.0) fine_edges.push_back(edge);
    RangeBins fine_bins(fine_edges);
    DetectorScore detector_score;
    json tracks = json::array();
    std::size_t scans = 0;
    std::vector<DetectedCone> detected;
    for (int t = 0; t < settings.tracks; ++t) {
        const std::uint64_t track_seed = settings.seed + static_cast<std::uint64_t>(t);
        const Track track = sim::generate_track(track_seed);
        const Rng track_rng = Rng(settings.seed).split(static_cast<std::uint64_t>(t));
        Rng pose_rng = track_rng.split(0);
        Rng lidar_rng = track_rng.split(1);
        std::vector<Rng> camera_rngs;
        for (std::size_t v = 0; v < variants.size(); ++v) {
            camera_rngs.push_back(track_rng.split(2 + v));
        }
        std::size_t track_scans = 0;
        for (double s = 0.0; s < track.centerline.length(); s += settings.spacing) {
            const Motion motion(track.centerline, s, pose_rng);
            const Time stamp = kScanPeriod * static_cast<double>(scans);
            const Truth truth{track.cones, motion.pose()};
            const LidarScan scan = sim::simulate_lidar(rig.lidar, noise.lidar, state_at(truth.body),
                                                       track.cones, stamp, lidar_rng);

            std::vector<int> matches;
            std::size_t reported = 0;
            for (std::size_t v = 0; v < variants.size(); ++v) {
                Variant& variant = variants[v];
                const VariantSpec& spec = *variant.spec;
                const Pose2 camera_body = motion.pose(spec.lag);
                variant.perception.on_camera(
                    sim::simulate_camera(rig.camera, camera_noise(spec), state_at(camera_body),
                                         track.cones, stamp - spec.lag, camera_rngs[v]));
                const double yaw_rate = spec.compensate ? motion.yaw_rate(spec.lag) : 0.0;
                const ConeObservations observations = variant.perception.on_lidar(scan, yaw_rate);
                if (v == 0) {
                    matches = match_observations(observations, truth, kMatchGate);
                    reported = observations.cones.size();
                    score_detections(observations, matches, scan, truth, rig.lidar, bins);
                    score_detections(observations, matches, scan, truth, rig.lidar, fine_bins);
                } else if (observations.cones.size() != reported) {
                    throw std::logic_error("camera variants disagree on the cones of a scan");
                }
                score_colors(observations, matches, truth, camera_body, rig.camera, variant.color);
            }

            (void)detector.detect(scan.points, detected);
            score_detector(detected, truth, rig.lidar, kMatchGate, detector_score);
            ++scans;
            ++track_scans;
        }
        tracks.push_back({{"seed", track_seed},
                          {"length_m", track.centerline.length()},
                          {"cones", track.cones.size()},
                          {"scans", track_scans}});
        std::cout << std::format("track {} ({:.0f} m, {} cones): {} scans\n", track_seed,
                                 track.centerline.length(), track.cones.size(), track_scans);
    }

    const auto bins_to_json = [](const RangeBins& b) {
        json out = json::array();
        for (std::size_t i = 0; i < b.size(); ++i) out.push_back(bin_to_json(b[i], b.label(i)));
        return out;
    };
    const DetectionBin total = bins.total();
    const PerceptionStats& stats = variants[0].perception.stats();
    json checks = json::object();
    for (std::size_t i = 0; i < kConeCheckCount; ++i) {
        checks[std::string(to_string(static_cast<ConeCheck>(i)))] = stats.cone_checks[i];
    }
    json colors = json::array();
    for (const Variant& v : variants) colors.push_back(color_to_json(v));
    const sim::CameraNoise& camera = noise.camera;
    const json camera_noise = {{"min_range_m", camera.min_range},
                               {"detection_probability", camera.detection_probability},
                               {"detection_half_range_m", camera.detection_half_range},
                               {"detection_falloff_m", camera.detection_falloff},
                               {"bearing_stddev_rad", camera.bearing_stddev},
                               {"range_stddev_m", camera.range_stddev},
                               {"range_stddev_quadratic", camera.range_stddev_quadratic},
                               {"confusion_near", camera.confusion_near},
                               {"confusion_far", camera.confusion_far},
                               {"confidence_stddev", camera.confidence_stddev},
                               {"false_positives_per_frame", camera.false_positives}};

    const json result = {
        {"command", settings.command},
        {"sensor_model",
         std::format("sim::simulate_lidar ({:.0f} cm range noise, {:.0f}% dropout) and "
                     "sim::simulate_camera with their default noise",
                     noise.lidar.range_stddev * 100.0, noise.lidar.dropout * 100.0)},
        {"tracks", tracks},
        {"scans", scans},
        {"lidar_noise",
         {{"range_stddev_m", noise.lidar.range_stddev}, {"dropout", noise.lidar.dropout}}},
        {"camera_noise", camera_noise},
        {"motion",
         {{"max_speed", kMaxSpeed},
          {"max_lateral_accel", kMaxLateralAccel},
          {"lateral_spread_m", kLateralSpread},
          {"yaw_spread_rad", kYawSpread}}},
        {"config", config},
        {"by_range", bins_to_json(bins)},
        {"by_range_2m", bins_to_json(fine_bins)},
        {"total", bin_to_json(total, "all")},
        {"false_positives_per_scan",
         ratio(static_cast<double>(total.false_positives), static_cast<double>(scans))},
        {"nees_mean", ratio(total.nees, static_cast<double>(total.detected))},
        {"color", colors},
        {"big_cones",
         {{"big_cones", detector_score.big_cones},
          {"big_as_big", detector_score.big_as_big},
          {"small_cones", detector_score.small_cones},
          {"small_as_big", detector_score.small_as_big}}},
        {"cone_checks", checks},
        {"latency", stats.lidar_time.summary()},
        {"fitted_covariance",
         {{"radial", fit_variance_model(detector_score.radial)},
          {"tangential", fit_variance_model(detector_score.tangential)}}}};

    std::filesystem::create_directories(settings.out);
    std::ofstream(settings.out / "eval.json") << result.dump(2) << '\n';
    const std::string md = summary_markdown(result);
    std::ofstream(settings.out / "summary.md") << md;
    std::cout << '\n' << md;
    return 0;
}

}  // namespace
}  // namespace apex::perception::eval

int main(int argc, char** argv) {
    try {
        return apex::perception::eval::run(apex::perception::eval::parse(argc, argv));
    } catch (const std::exception& e) {
        std::cerr << "apex_perception_eval: " << e.what() << "\n"
                  << "usage: apex_perception_eval [--tracks N] [--spacing M] [--seed S] "
                     "[--out DIR]\n";
        return 1;
    }
}
