#include "apex/stack/known_pose_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "apex/core/rng.hpp"
#include "apex/core/track.hpp"

namespace apex::stack {
namespace {

// Observations of every cone within `range` of `pose`, in its frame, with Gaussian noise.
ConeObservations observe(const Track& track, const Pose2& pose, double range, double sigma,
                         Rng& rng) {
    ConeObservations obs;
    for (const Cone& cone : track.cones) {
        const Vec2 local = pose.to_local(cone.position);
        if (local.norm() > range || local.x() < 0.0) continue;
        ConeObservation o;
        o.position = local + Vec2{rng.normal(0.0, sigma), rng.normal(0.0, sigma)};
        o.covariance = Mat2::Identity() * std::max(sigma * sigma, 1e-6);
        o.color = cone.color;
        o.color_confidence = 1.0;
        o.truth_id = cone.id;
        obs.cones.push_back(o);
    }
    return obs;
}

Pose2 pose_at(const Track& track, double s) {
    const TrajectoryPoint p = track.centerline.at(s);
    return {p.position.x(), p.position.y(), p.yaw};
}

TEST(KnownPoseMapper, MapsEveryConeOnceWithItsColor) {
    const Track track = make_oval_track();
    KnownPoseMapper mapper(KnownPoseMapperConfig{});
    Rng rng(3);
    for (double s = 0.0; s < track.centerline.length(); s += 1.0) {
        mapper.add(observe(track, pose_at(track, s), 12.0, 0.0, rng), pose_at(track, s));
    }
    const ConeMap& map = mapper.map();
    ASSERT_EQ(map.cones.size(), track.cones.size());
    std::vector<int> seen(track.cones.size(), 0);
    for (std::size_t i = 0; i < map.cones.size(); ++i) {
        const MapCone& cone = map.cones[i];
        EXPECT_EQ(cone.id, static_cast<std::int32_t>(i));
        ASSERT_GE(cone.truth_id, 0);
        const Cone& truth = track.cones[static_cast<std::size_t>(cone.truth_id)];
        EXPECT_LT((cone.position - truth.position).norm(), 1e-9);
        EXPECT_EQ(cone.color, truth.color);
        EXPECT_GT(cone.observations, 2U);
        ++seen[static_cast<std::size_t>(cone.truth_id)];
    }
    for (const int count : seen) EXPECT_EQ(count, 1);
}

TEST(KnownPoseMapper, AveragingBeatsASingleObservation) {
    const Track track = make_oval_track();
    KnownPoseMapper mapper(KnownPoseMapperConfig{});
    Rng rng(11);
    constexpr double kSigma = 0.1;
    for (double s = 0.0; s < 40.0; s += 0.5) {
        mapper.add(observe(track, pose_at(track, s), 15.0, kSigma, rng), pose_at(track, s));
    }
    double squared = 0.0;
    int matched = 0;
    for (const MapCone& cone : mapper.map().cones) {
        if (cone.observations < 10) continue;
        const Cone& truth = track.cones[static_cast<std::size_t>(cone.truth_id)];
        squared += (cone.position - truth.position).squaredNorm();
        ++matched;
        EXPECT_LT(cone.covariance(0, 0), kSigma * kSigma / 5.0);
    }
    ASSERT_GT(matched, 10);
    // One observation has an RMS 2D error of sigma * sqrt(2); ten or more give under half that.
    EXPECT_LT(std::sqrt(squared / matched), 0.5 * kSigma * std::sqrt(2.0));
}

TEST(KnownPoseMapper, TentativeUntilConfirmedAndAssociationsAfter) {
    KnownPoseMapperConfig config;
    config.confirm_after = 3;
    KnownPoseMapper mapper(config);
    ConeObservations obs;
    ConeObservation o;
    o.position = {5.0, 1.0};
    o.covariance = Mat2::Identity() * 0.01;
    o.truth_id = 42;
    obs.cones.push_back(o);
    const Pose2 pose{1.0, 2.0, 0.3};

    mapper.add(obs, pose);
    EXPECT_TRUE(mapper.map().cones.empty());
    EXPECT_EQ(mapper.last_associations(), std::vector<std::int32_t>{-1});
    mapper.add(obs, pose);
    EXPECT_TRUE(mapper.map().cones.empty());
    mapper.add(obs, pose);
    ASSERT_EQ(mapper.map().cones.size(), 1U);
    // Matched to a landmark that was still tentative: not a confirmed association.
    EXPECT_EQ(mapper.last_associations(), std::vector<std::int32_t>{-1});
    mapper.add(obs, pose);
    EXPECT_EQ(mapper.last_associations(), std::vector<std::int32_t>{0});
    EXPECT_EQ(mapper.map().cones[0].truth_id, 42);
    EXPECT_LT((mapper.map().cones[0].position - pose.to_parent(o.position)).norm(), 1e-12);
}

TEST(KnownPoseMapper, OneLandmarkPerObservationPerScan) {
    KnownPoseMapperConfig config;
    config.confirm_after = 1;
    KnownPoseMapper mapper(config);
    ConeObservations first;
    ConeObservation o;
    o.position = {5.0, 0.0};
    o.covariance = Mat2::Identity() * 0.01;
    first.cones.push_back(o);
    mapper.add(first, Pose2{});
    ASSERT_EQ(mapper.map().cones.size(), 1U);

    // Two returns near the same landmark in one scan: the closer one takes it.
    ConeObservations second = first;
    second.cones[0].position = {5.4, 0.0};
    second.cones.push_back(o);
    second.cones[1].position = {5.1, 0.0};
    mapper.add(second, Pose2{});
    EXPECT_EQ(mapper.last_associations(), (std::vector<std::int32_t>{-1, 0}));
    EXPECT_EQ(mapper.map().cones.size(), 2U);
}

TEST(KnownPoseMapper, FrozenMapOnlyAssociates) {
    KnownPoseMapperConfig config;
    config.confirm_after = 1;
    KnownPoseMapper mapper(config);
    ConeObservations obs;
    ConeObservation o;
    o.position = {5.0, 0.0};
    o.covariance = Mat2::Identity() * 0.01;
    obs.cones.push_back(o);
    mapper.add(obs, Pose2{});
    const std::uint64_t revision = mapper.map().revision;

    mapper.set_mapping(false);
    EXPECT_FALSE(mapper.mapping());
    obs.cones.push_back(o);
    obs.cones[1].position = {20.0, 3.0};  // a new cone: not added any more
    mapper.add(obs, Pose2{});
    EXPECT_EQ(mapper.last_associations(), (std::vector<std::int32_t>{0, -1}));
    EXPECT_EQ(mapper.map().cones.size(), 1U);
    EXPECT_EQ(mapper.map().revision, revision);
    EXPECT_EQ(mapper.map().cones[0].observations, 1U);
}

TEST(KnownPoseMapper, SkipsUnusableObservations) {
    KnownPoseMapperConfig config;
    config.confirm_after = 1;
    KnownPoseMapper mapper(config);
    ConeObservations obs;
    ConeObservation degenerate;
    degenerate.position = {5.0, 0.0};
    degenerate.covariance = Mat2::Zero();
    ConeObservation nan;
    nan.position = {std::nan(""), 0.0};
    obs.cones = {degenerate, nan};
    mapper.add(obs, Pose2{});
    EXPECT_TRUE(mapper.map().cones.empty());
    EXPECT_EQ(mapper.last_associations(), (std::vector<std::int32_t>{-1, -1}));
}

TEST(KnownPoseMapper, RejectsInvalidConfigs) {
    KnownPoseMapperConfig config;
    config.gate = 0.0;
    EXPECT_THROW(KnownPoseMapper{config}, std::invalid_argument);
    config.gate = 1.0;
    config.confirm_after = 0;
    EXPECT_THROW(KnownPoseMapper{config}, std::invalid_argument);
}

}  // namespace
}  // namespace apex::stack
