#include "apex/stack/known_pose_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <tuple>

#include <Eigen/LU>
#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"

namespace apex::stack {

void to_json(nlohmann::json& j, const KnownPoseMapperConfig& c) {
    j = {{"gate", c.gate}, {"confirm_after", c.confirm_after}};
}

void from_json(const nlohmann::json& j, KnownPoseMapperConfig& c) {
    reject_unknown_keys(j, {"gate", "confirm_after"}, "known_pose_mapper");
    read_optional(j, "gate", c.gate);
    read_optional(j, "confirm_after", c.confirm_after);
}

KnownPoseMapper::KnownPoseMapper(const KnownPoseMapperConfig& config) : config_(config) {
    if (!(config_.gate > 0.0) || config_.confirm_after < 1) {
        throw std::invalid_argument("known_pose_mapper: gate must be positive, confirm_after >= 1");
    }
}

void KnownPoseMapper::fuse(Landmark& landmark, const Vec2& position, const Mat2& covariance,
                           const ConeObservation& observation) {
    const Mat2 information = covariance.inverse();
    landmark.information += information;
    landmark.information_vector += information * position;
    landmark.mean = landmark.information.inverse() * landmark.information_vector;
    if (observation.color != ConeColor::Unknown) ++landmark.votes[index_of(observation.color)];
    ++landmark.sightings;
    auto vote = std::find_if(landmark.truth_votes.begin(), landmark.truth_votes.end(),
                             [&](const auto& v) { return v.first == observation.truth_id; });
    if (vote == landmark.truth_votes.end()) {
        landmark.truth_votes.emplace_back(observation.truth_id, 1U);
    } else {
        ++vote->second;
    }
    if (landmark.id < 0 &&
        landmark.sightings >= static_cast<std::uint32_t>(config_.confirm_after)) {
        landmark.id = next_id_++;
    }
}

void KnownPoseMapper::add(const ConeObservations& observations, const Pose2& pose) {
    const std::size_t n = observations.cones.size();
    associations_.assign(n, -1);
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    Mat2 rotation;
    rotation << c, -s, s, c;

    std::vector<Vec2> positions(n);
    std::vector<Mat2> covariances(n);
    // Observations that cannot be fused (non-finite, or a covariance that is not positive
    // definite) are skipped as if already used.
    std::vector<bool> observation_used(n, false);
    std::vector<std::tuple<double, std::size_t, std::size_t>> pairs;  // distance, obs, landmark
    for (std::size_t i = 0; i < n; ++i) {
        const ConeObservation& o = observations.cones[i];
        positions[i] = pose.to_parent(o.position);
        covariances[i] = rotation * o.covariance * rotation.transpose();
        if (!positions[i].allFinite() || !covariances[i].allFinite() ||
            !(covariances[i](0, 0) > 0.0) || !(covariances[i].determinant() > 0.0)) {
            observation_used[i] = true;
            continue;
        }
        for (std::size_t l = 0; l < landmarks_.size(); ++l) {
            const double d = (positions[i] - landmarks_[l].mean).norm();
            if (d < config_.gate) pairs.emplace_back(d, i, l);
        }
    }
    // Greedy global nearest neighbour: closest pairs first, each observation and landmark used
    // at most once per scan (one cone cannot produce two returns in one scan).
    std::sort(pairs.begin(), pairs.end());
    std::vector<bool> landmark_used(landmarks_.size(), false);
    std::vector<std::pair<std::size_t, std::size_t>> matches;
    for (const auto& [distance, i, l] : pairs) {
        if (observation_used[i] || landmark_used[l]) continue;
        observation_used[i] = true;
        landmark_used[l] = true;
        associations_[i] = landmarks_[l].id;
        matches.emplace_back(i, l);
    }
    if (!mapping_) return;

    for (const auto& [i, l] : matches) {
        fuse(landmarks_[l], positions[i], covariances[i], observations.cones[i]);
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (observation_used[i]) continue;
        landmarks_.emplace_back();
        fuse(landmarks_.back(), positions[i], covariances[i], observations.cones[i]);
    }
    if (n > 0) publish();
}

void KnownPoseMapper::publish() {
    map_.cones.clear();
    for (const Landmark& landmark : landmarks_) {
        if (landmark.id < 0) continue;
        MapCone cone;
        cone.id = landmark.id;
        cone.position = landmark.mean;
        cone.covariance = landmark.information.inverse();
        cone.color_votes = landmark.votes;
        cone.observations = landmark.sightings;
        const auto best = std::max_element(landmark.votes.begin(), landmark.votes.end());
        cone.color = *best == 0
                         ? ConeColor::Unknown
                         : static_cast<ConeColor>(std::distance(landmark.votes.begin(), best));
        const auto truth =
            std::max_element(landmark.truth_votes.begin(), landmark.truth_votes.end(),
                             [](const auto& a, const auto& b) { return a.second < b.second; });
        cone.truth_id = truth == landmark.truth_votes.end() ? -1 : truth->first;
        map_.cones.push_back(cone);
    }
    // Landmarks are confirmed out of creation order; the map lists them by id.
    std::sort(map_.cones.begin(), map_.cones.end(),
              [](const MapCone& a, const MapCone& b) { return a.id < b.id; });
    ++map_.revision;
}

}  // namespace apex::stack
