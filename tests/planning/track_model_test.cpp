#include <algorithm>
#include <cmath>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/angles.hpp"
#include "apex/planning/planning.hpp"
#include "apex/sim/track_gen.hpp"

#include "cone_maps.hpp"

namespace apex::planning {
namespace {

using test_support::MapErrors;

const Track& reference_track(const std::string& name) {
    static std::map<std::string, Track> cache;
    auto it = cache.find(name);
    if (it == cache.end()) it = cache.emplace(name, sim::reference_track(name)).first;
    return it->second;
}

MapErrors no_errors() {
    return {.position_rms = 0.0,
            .miss_probability = 0.0,
            .wrong_color_probability = 0.0,
            .unknown_color_probability = 0.0,
            .false_positives_per_100m = 0.0};
}

struct Lap {
    ConeMap map;
    std::vector<Pose2> drive;
};

Lap drive_lap(const Track& track, const MapErrors& errors, std::uint64_t seed,
              double wander = 0.5) {
    const Rng rng(seed);
    Rng map_rng = rng.split(1);
    Rng drive_rng = rng.split(2);
    return {test_support::make_cone_map(track, errors, map_rng),
            test_support::driven_lap(track, wander, drive_rng)};
}

// The largest distance from a point of either centerline to the other.
double centerline_error(const TrackModel& model, const Track& track) {
    double worst = 0.0;
    for (const TrajectoryPoint& p : model.centerline.points()) {
        worst = std::max(worst, track.centerline.project(p.position).distance);
    }
    for (const TrajectoryPoint& p : track.centerline.points()) {
        worst = std::max(worst, model.centerline.project(p.position).distance);
    }
    return worst;
}

TEST(TrackModel, PerfectMapOfTheOvalGivesTheTrueTrack) {
    const Track track = make_oval_track();
    const Lap lap = drive_lap(track, no_errors(), 1);
    const std::optional<TrackModel> model = build_track_model(lap.map, lap.drive, {});
    ASSERT_TRUE(model.has_value());
    EXPECT_LT(centerline_error(*model, track), 0.1);
    EXPECT_TRUE(model->centerline.closed());
    EXPECT_NEAR(model->centerline.length(), track.centerline.length(), 0.5);
    // It starts on the timekeeping line, in the driving direction.
    const TrajectoryPoint& start = model->centerline[0];
    EXPECT_LT((start.position - track.centerline[0].position).norm(), 0.3);
    EXPECT_LT(std::abs(angle_diff(start.yaw, track.centerline[0].yaw)), 0.05);
    // Every true cone is on its cone line: here the lines are the cones themselves.
    EXPECT_EQ(model->left.size() + model->right.size(), track.cones.size());
    for (const TrajectoryPoint& p : model->centerline.points()) {
        EXPECT_NEAR(p.width_left, 1.75, 0.2);
        EXPECT_NEAR(p.width_right, 1.75, 0.2);
    }
}

// RMS distance from the true centerline's points to the model's centerline.
double centerline_rms(const TrackModel& model, const Track& track) {
    double sum = 0.0;
    for (const TrajectoryPoint& p : track.centerline.points()) {
        const double d = model.centerline.project(p.position).distance;
        sum += d * d;
    }
    return std::sqrt(sum / static_cast<double>(track.centerline.size()));
}

// The task's acceptance test: on imperfect maps of generated tracks (10 cm RMS position error,
// 10% of the cones missing, 5% with a wrong or unknown color, clutter), the model's centerline
// stays within 0.3 m of the truth everywhere, and the cone lines are on the correct sides.
TEST(TrackModel, ImperfectMapsOfGeneratedTracksStayWithin30cm) {
    for (const std::string& name : sim::reference_track_names()) {
        const Track& track = reference_track(name);
        for (const std::uint64_t seed : {1U, 2U, 3U}) {
            const Lap lap = drive_lap(track, MapErrors{}, seed);
            const std::optional<TrackModel> model = build_track_model(lap.map, lap.drive, {});
            ASSERT_TRUE(model.has_value()) << name << " seed " << seed;
            EXPECT_LT(centerline_error(*model, track), 0.3) << name << " seed " << seed;
            for (const Vec2& p : model->left) {
                EXPECT_GT(track.centerline.project(p).lateral, 1.2) << name << " seed " << seed;
            }
            for (const Vec2& p : model->right) {
                EXPECT_LT(track.centerline.project(p).lateral, -1.2) << name << " seed " << seed;
            }
        }
    }
}

// At the worst imperfection levels (15 cm RMS, 20% missing, 10% colors, twice the clutter), a
// cone two or three sigma off inside a long one-sided gap moves the model further:
// docs/planning.md gives the distribution over many maps. This guards it against regressions.
TEST(TrackModel, WorstCaseMapsStayCloseOnAverage) {
    for (const std::string& name : sim::reference_track_names()) {
        const Track& track = reference_track(name);
        for (const std::uint64_t seed : {1U, 2U}) {
            const Lap lap = drive_lap(track, test_support::worst_case_errors(), seed);
            const std::optional<TrackModel> model = build_track_model(lap.map, lap.drive, {});
            ASSERT_TRUE(model.has_value()) << name << " seed " << seed;
            EXPECT_LT(centerline_rms(*model, track), 0.15) << name << " seed " << seed;
            EXPECT_LT(centerline_error(*model, track), 0.6) << name << " seed " << seed;
        }
    }
}

// With every cone mapped exactly, what is left is the method's own bias. Generated track 18's
// lap-1 drive passes its start 0.5 m to the side, which once left a 0.7 m kink in the model.
TEST(TrackModel, PerfectMapsOfGeneratedTracksGiveTheirCenterline) {
    for (const std::uint64_t seed : {3U, 18U}) {
        const Track track = sim::generate_track(seed);
        const Lap lap = drive_lap(track, no_errors(), seed);
        const std::optional<TrackModel> model = build_track_model(lap.map, lap.drive, {});
        ASSERT_TRUE(model.has_value()) << "seed " << seed;
        EXPECT_LT(centerline_error(*model, track), 0.1) << "seed " << seed;
    }
}

TEST(TrackModel, FillsAGapAlongTheTrackShape) {
    // Take out every left cone over 12 m of a curve: the gap's fill follows the curve.
    const Track& track = reference_track("technical");
    Lap lap = drive_lap(track, no_errors(), 3);
    double s_curve = 0.0;
    for (const TrajectoryPoint& p : track.centerline.points()) {
        if (std::abs(p.curvature) > std::abs(track.centerline.at(s_curve).curvature)) {
            s_curve = p.s;
        }
    }
    std::erase_if(lap.map.cones, [&](const MapCone& cone) {
        const Trajectory::Projection proj = track.centerline.project(cone.position);
        return proj.lateral > 0.0 && std::abs(track.centerline.delta_s(s_curve, proj.s)) < 6.0;
    });
    const std::optional<TrackModel> model = build_track_model(lap.map, lap.drive, {});
    ASSERT_TRUE(model.has_value());
    EXPECT_LT(centerline_error(*model, track), 0.3);
    for (const Vec2& p : model->left) {
        EXPECT_NEAR(track.centerline.project(p).lateral, 1.75, 0.3);
    }
}

TEST(TrackModel, IgnoresTruthIds) {
    const Track& track = reference_track("mixed");
    Lap lap = drive_lap(track, MapErrors{}, 4);
    const std::optional<TrackModel> before = build_track_model(lap.map, lap.drive, {});
    for (std::size_t i = 0; i < lap.map.cones.size(); ++i) {
        lap.map.cones[i].truth_id = static_cast<std::int32_t>((i * 7919) % 1000);
    }
    const std::optional<TrackModel> after = build_track_model(lap.map, lap.drive, {});
    ASSERT_TRUE(before.has_value());
    ASSERT_TRUE(after.has_value());
    ASSERT_EQ(before->centerline.size(), after->centerline.size());
    for (std::size_t i = 0; i < before->centerline.size(); ++i) {
        EXPECT_EQ(before->centerline[i].position, after->centerline[i].position);
    }
}

TEST(TrackModel, FailsWithoutALapOrCones) {
    const Track track = make_oval_track();
    const Lap lap = drive_lap(track, MapErrors{}, 5);

    const std::vector<Pose2> half(
        lap.drive.begin(), lap.drive.begin() + static_cast<std::ptrdiff_t>(lap.drive.size() / 2));
    EXPECT_FALSE(build_track_model(lap.map, half, {}).has_value());
    EXPECT_FALSE(build_track_model(lap.map, {}, {}).has_value());
    EXPECT_FALSE(build_track_model(ConeMap{}, lap.drive, {}).has_value());

    ConeMap one_side = lap.map;
    std::erase_if(one_side.cones, [&](const MapCone& cone) {
        return track.centerline.project(cone.position).lateral < 0.0;
    });
    EXPECT_FALSE(build_track_model(one_side, lap.drive, {}).has_value());

    // 40 m with no cones on either side is more than a gap to fill.
    ConeMap gap = lap.map;
    std::erase_if(gap.cones, [&](const MapCone& cone) {
        const double s = track.centerline.project(cone.position).s;
        return s > 40.0 && s < 80.0;
    });
    EXPECT_FALSE(build_track_model(gap, lap.drive, {}).has_value());
}

TEST(TrackModel, RejectsInvalidConfig) {
    const Track track = make_oval_track();
    const Lap lap = drive_lap(track, MapErrors{}, 6);
    TrackModelConfig config;
    config.reference_spacing = 0.0;
    EXPECT_THROW((void)build_track_model(lap.map, lap.drive, config), std::invalid_argument);
    config = {};
    config.max_lateral = config.min_lateral;
    EXPECT_THROW((void)build_track_model(lap.map, lap.drive, config), std::invalid_argument);
}

}  // namespace
}  // namespace apex::planning
