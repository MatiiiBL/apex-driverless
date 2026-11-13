// Evaluates the perception pipeline on scans simulated at poses along the centerlines of
// generated tracks (sim::generate_track with seeds seed, seed + 1, ...): detection recall and
// precision, position error and NEES by range, false positives per scan, color coverage and
// accuracy, the big-cone classification, on_lidar latency, and the covariance model fitted to
// the errors (fit_variance_model). Writes eval.json and summary.md to the output directory.
// See docs/perception.md, "Evaluation".
//
//   apex_perception_eval [--tracks N] [--spacing M] [--seed S] [--out DIR]
//
// Scans come from the exact ray caster of the perception tests (tests/perception/scene.hpp),
// with the sensor noise below, until the simulator's sensor models are available.
#include <algorithm>
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
#include "apex/core/json.hpp"
#include "apex/core/rng.hpp"
#include "apex/perception/calibration.hpp"
#include "apex/perception/detector.hpp"
#include "apex/perception/perception.hpp"
#include "apex/sim/track_gen.hpp"

#include "scene.hpp"
#include "scoring.hpp"

namespace apex::perception::eval {
namespace {

using nlohmann::json;

// LiDAR: 2 cm range noise (1 sigma), the precision of automotive spinning LiDARs, and 2% of
// rays lost. Camera detector: bearing noise of about 4 pixels of a 1280-pixel, 100-degree image;
// 7% monocular range error; detection probability falling to 60% and color confusion rising to
// 10% at the camera's 30 m range; half a false positive per frame.
constexpr double kLidarRangeSigma = 0.02;
constexpr double kLidarDropout = 0.02;
constexpr testing::CameraSimOptions kCameraNoise{.bearing_sigma = 0.005,
                                                 .range_sigma_fraction = 0.07,
                                                 .confidence = 0.9,
                                                 .detect_at_max_range = 0.6,
                                                 .confusion_at_max_range = 0.10,
                                                 .false_positives = 0.5};

// The car drives near the centerline (uniform lateral offset, Gaussian heading error) at up to
// 12 m/s, slowing so its lateral acceleration stays under 10 m/s^2: an exploration lap.
constexpr double kMaxSpeed = 12.0;
constexpr double kMaxLateralAccel = 10.0;
constexpr double kLateralSpread = 0.3;  // m
constexpr double kYawSpread = 0.03;     // rad
// The camera frame fused with each scan is stamped this much before it: the worst phase of a
// 20 Hz camera against a 10 Hz LiDAR. A variant gets frames at the scan's own stamp.
constexpr double kCameraLag = 0.025;  // s
constexpr double kScanPeriod = 0.1;   // s
constexpr double kMatchGate = 0.5;    // m: farther from its cone, an observation is false

struct Settings {
    int tracks = 8;
    double spacing = 2.5;  // m between scans along each centerline
    std::uint64_t seed = 1;
    std::filesystem::path out = "results/perception";
    std::string command = "apex_perception_eval";
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

struct Motion {
    Pose2 body;         // at the scan's stamp
    Pose2 camera_body;  // kCameraLag earlier
    double yaw_rate;    // mean over the lag, as integrated from a gyro (rad/s)
};

Motion drive(const Trajectory& centerline, double s, Rng& rng) {
    const double offset = rng.uniform(-kLateralSpread, kLateralSpread);
    const double yaw_error = rng.normal(0.0, kYawSpread);
    const TrajectoryPoint now = centerline.at(s);
    const double speed =
        std::min(kMaxSpeed, std::sqrt(kMaxLateralAccel / std::max(std::abs(now.curvature), 1e-6)));
    const TrajectoryPoint before = centerline.at(s - speed * kCameraLag);
    const auto pose = [&](const TrajectoryPoint& p) {
        const Vec2 position = p.position + offset * left_normal(heading_vector(p.yaw));
        return Pose2{position.x(), position.y(), wrap_angle(p.yaw + yaw_error)};
    };
    return {pose(now), pose(before), angle_diff(now.yaw, before.yaw) / kCameraLag};
}

// The same pipeline fed camera frames with different timing, for the color comparison.
struct Variant {
    std::string name;
    std::string description;
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
            {"nees", ratio(b.nees, tp)}};
}

json color_to_json(const Variant& v) {
    const ColorScore& c = v.color;
    return {
        {"name", v.name},
        {"description", v.description},
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
        "Position RMS (cm) | Radial RMS (cm) | Tangential RMS (cm) | NEES |\n";
    md += "|---|---|---|---|---|---|---|---|---|\n";
    const auto row = [&](const json& b) {
        md += std::format(
            "| {} | {} | {:.3f} | {:.3f} | {:.4f} | {:.1f} | {:.1f} | {:.1f} | {:.2f} |\n",
            b["range_m"].get<std::string>(), b["in_view"].get<int>(), b["recall"].get<double>(),
            b["recall_of_hit"].get<double>(), b["precision"].get<double>(),
            100.0 * b["rms_error_m"].get<double>(), 100.0 * b["rms_radial_m"].get<double>(),
            100.0 * b["rms_tangential_m"].get<double>(), b["nees"].get<double>());
    };
    for (const json& b : r["by_range"]) row(b);
    row(r["total"]);
    md += std::format("\nFalse positives per scan: {:.4f}. NEES average: {:.2f} (2 expected).\n\n",
                      r["false_positives_per_scan"].get<double>(), r["nees_mean"].get<double>());

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
    variants.push_back({"lag_compensated",
                        std::format("{:.0f} ms before the scan, yaw compensated", kCameraLag * 1e3),
                        ConePerception(config, rig),
                        {}});
    variants.push_back({"aligned", "at the scan's stamp", ConePerception(config, rig), {}});
    variants.push_back({"lag_uncompensated",
                        std::format("{:.0f} ms before the scan, not compensated", kCameraLag * 1e3),
                        ConePerception(config, rig),
                        {}});
    ConeDetector detector(config.ground, config.clustering, config.cone_check, config.center_fit,
                          rig.lidar);

    testing::LidarSimOptions lidar_noise;
    lidar_noise.sensor_height = rig.lidar.mount.z();
    lidar_noise.range_sigma = kLidarRangeSigma;
    lidar_noise.dropout = kLidarDropout;

    DetectionBins bins{};
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
        Rng camera_rng = track_rng.split(2);
        std::size_t track_scans = 0;
        for (double s = 0.0; s < track.centerline.length(); s += settings.spacing) {
            const Motion m = drive(track.centerline, s, pose_rng);
            const Time stamp = kScanPeriod * static_cast<double>(scans);
            const LidarScan scan =
                testing::cast_lidar(rig.lidar, testing::lidar_pose(m.body, rig.lidar), track.cones,
                                    lidar_noise, lidar_rng, stamp);
            const CameraFrame lagged =
                testing::detect_cones(rig.camera, m.camera_body, track.cones, kCameraNoise,
                                      camera_rng, stamp - kCameraLag);
            const CameraFrame aligned = testing::detect_cones(rig.camera, m.body, track.cones,
                                                              kCameraNoise, camera_rng, stamp);
            variants[0].perception.on_camera(lagged);
            variants[1].perception.on_camera(aligned);
            variants[2].perception.on_camera(lagged);

            const Truth truth{track.cones, m.body};
            const ConeObservations observations = variants[0].perception.on_lidar(scan, m.yaw_rate);
            const std::vector<int> matches = match_observations(observations, truth, kMatchGate);
            score_detections(observations, matches, scan, truth, rig.lidar, bins);
            score_colors(observations, matches, truth, m.camera_body, rig.camera,
                         variants[0].color);
            // The LiDAR pipeline is deterministic, so the variants report the same cones in the
            // same order and share the matches.
            const ConeObservations at_stamp = variants[1].perception.on_lidar(scan, m.yaw_rate);
            const ConeObservations uncompensated = variants[2].perception.on_lidar(scan, 0.0);
            if (at_stamp.cones.size() != observations.cones.size() ||
                uncompensated.cones.size() != observations.cones.size()) {
                throw std::logic_error("variants disagree on the cones of a scan");
            }
            score_colors(at_stamp, matches, truth, m.body, rig.camera, variants[1].color);
            score_colors(uncompensated, matches, truth, m.camera_body, rig.camera,
                         variants[2].color);

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

    DetectionBin total;
    json by_range = json::array();
    for (std::size_t b = 0; b < kBins; ++b) {
        by_range.push_back(bin_to_json(bins[b], bin_label(b)));
        total.in_view += bins[b].in_view;
        total.hit += bins[b].hit;
        total.detected += bins[b].detected;
        total.reported += bins[b].reported;
        total.false_positives += bins[b].false_positives;
        total.error_sq += bins[b].error_sq;
        total.radial_sq += bins[b].radial_sq;
        total.tangential_sq += bins[b].tangential_sq;
        total.nees += bins[b].nees;
    }
    const PerceptionStats& stats = variants[0].perception.stats();
    json checks = json::object();
    for (std::size_t i = 0; i < kConeCheckCount; ++i) {
        checks[std::string(to_string(static_cast<ConeCheck>(i)))] = stats.cone_checks[i];
    }
    json colors = json::array();
    for (const Variant& v : variants) colors.push_back(color_to_json(v));
    json camera_noise = {{"bearing_sigma_rad", kCameraNoise.bearing_sigma},
                         {"range_sigma_fraction", kCameraNoise.range_sigma_fraction},
                         {"confidence", kCameraNoise.confidence},
                         {"detect_at_max_range", kCameraNoise.detect_at_max_range},
                         {"confusion_at_max_range", kCameraNoise.confusion_at_max_range},
                         {"false_positives_per_frame", kCameraNoise.false_positives}};

    const json result = {
        {"command", settings.command},
        {"sensor_model",
         std::format("synthetic sensors (exact ray caster, {:.0f} cm range noise, {:.0f}% dropout)",
                     kLidarRangeSigma * 100.0, kLidarDropout * 100.0)},
        {"tracks", tracks},
        {"scans", scans},
        {"lidar_noise", {{"range_sigma_m", kLidarRangeSigma}, {"dropout", kLidarDropout}}},
        {"camera_noise", camera_noise},
        {"motion",
         {{"max_speed", kMaxSpeed},
          {"max_lateral_accel", kMaxLateralAccel},
          {"lateral_spread_m", kLateralSpread},
          {"yaw_spread_rad", kYawSpread},
          {"camera_lag_s", kCameraLag}}},
        {"config", config},
        {"by_range", by_range},
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
