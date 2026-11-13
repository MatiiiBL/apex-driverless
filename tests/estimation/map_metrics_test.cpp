#include "apex/estimation/eval/map_metrics.hpp"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/track.hpp"

namespace apex::estimation::eval {
namespace {

MapCone landmark(std::int32_t id, std::int32_t truth_id, const Vec2& position) {
    MapCone cone;
    cone.id = id;
    cone.truth_id = truth_id;
    cone.position = position;
    return cone;
}

TEST(MapMetrics, ScoresErrorsDuplicatesSpuriousAndMissedCones) {
    const Track track = make_oval_track();
    const Vec2 c0 = track.cones[0].position;
    const Vec2 c1 = track.cones[1].position;
    ConeMap map;
    map.cones = {landmark(0, 0, c0 + Vec2(0.1, 0.0)), landmark(1, 1, c1),
                 landmark(2, 0, c0 + Vec2(0.0, 0.3)), landmark(3, -1, {100.0, 100.0})};
    const std::vector<std::int32_t> mappable = {0, 1, 2};
    const MapMetrics m = evaluate_map(map, track, mappable);
    EXPECT_EQ(m.landmarks, 4U);
    EXPECT_EQ(m.matched_cones, 2U);
    EXPECT_EQ(m.duplicates, 1U);
    EXPECT_EQ(m.spurious, 1U);
    EXPECT_EQ(m.missed, 1U);  // cone 2
    EXPECT_NEAR(m.rms_error, std::sqrt((0.01 + 0.0 + 0.09) / 3.0), 1e-12);
    EXPECT_NEAR(m.max_error, 0.3, 1e-12);
}

TEST(MapMetrics, AnEmptyMapMissesEverything) {
    const Track track = make_oval_track();
    const std::vector<std::int32_t> mappable = {0, 5, 9};
    const MapMetrics m = evaluate_map({}, track, mappable);
    EXPECT_EQ(m.missed, 3U);
    EXPECT_EQ(m.rms_error, 0.0);
}

ConeObservations observations(std::vector<std::int32_t> truth_ids) {
    ConeObservations obs;
    for (const std::int32_t id : truth_ids) {
        ConeObservation o;
        o.truth_id = id;
        obs.cones.push_back(o);
    }
    return obs;
}

TEST(AssociationScore, CountsPrecisionAndRecallByTruthId) {
    ConeMap before;
    // Positions do not matter here; Vec2{} would leave Eigen's coefficients uninitialized.
    before.cones = {landmark(0, 10, Vec2::Zero()), landmark(1, 11, Vec2::Zero()),
                    landmark(2, 12, Vec2::Zero())};
    ConeMap after = before;
    after.cones.push_back(landmark(3, 13, Vec2::Zero()));
    AssociationScore score;
    // Correct, wrong (cone 11 matched to landmark 2 of cone 12), unmatched but mappable,
    // clutter matched, and an observation of a cone with no landmark yet.
    const std::vector<std::int32_t> associations = {0, 2, -1, 1, -1};
    score.add(observations({10, 11, 12, -1, 13}), associations, before, after);
    EXPECT_EQ(score.matches(), 3U);
    EXPECT_EQ(score.correct(), 1U);
    EXPECT_NEAR(score.precision(), 1.0 / 3.0, 1e-12);
    EXPECT_NEAR(score.recall(), 1.0 / 3.0, 1e-12);  // 10 of {10, 11, 12}
}

TEST(AssociationScore, IsPerfectWithNothingToScore) {
    const AssociationScore score;
    EXPECT_EQ(score.precision(), 1.0);
    EXPECT_EQ(score.recall(), 1.0);
}

}  // namespace
}  // namespace apex::estimation::eval
