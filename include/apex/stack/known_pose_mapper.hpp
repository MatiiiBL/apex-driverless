#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/types.hpp"

namespace apex::stack {

struct KnownPoseMapperConfig {
    // An observation this close to a landmark's mean is that landmark. Cones on one side of a
    // track are at least 1.5 m apart and the two sides 3 m or more.
    double gate = 0.75;  // m
    // Sightings before a landmark appears in the map: suppresses one-off false positives.
    int confirm_after = 2;
};

void to_json(nlohmann::json& j, const KnownPoseMapperConfig& config);
void from_json(const nlohmann::json& j, KnownPoseMapperConfig& config);

// Cone mapping with known poses, used instead of SLAM under the perfect-state oracle: with the
// true pose given, each observation is moved into the world frame and fused into the landmark
// it falls on, by greedy nearest neighbour within a gate, as an information-form average
// (mapping with known poses; S. Thrun, W. Burgard and D. Fox, "Probabilistic Robotics", MIT
// Press, 2005, ch. 9). Replacing the estimator's pose with the truth while keeping its map would
// misalign the two, since an EKF-SLAM map is consistent with its own pose estimate, not with the
// truth.
//
// The published ConeMap follows the estimator's conventions: confirmed landmarks only, stable
// ids, color by vote, truth_id by majority (passed along for the evaluation, never read here to
// decide anything).
class KnownPoseMapper {
public:
    // Throws std::invalid_argument for a non-positive gate or confirm_after < 1.
    explicit KnownPoseMapper(const KnownPoseMapperConfig& config);

    // Fuses observations taken at `pose`, the car's pose at their stamp. With mapping off the
    // map is final: observations are only associated (for last_associations).
    void add(const ConeObservations& observations, const Pose2& pose);

    void set_mapping(bool enabled) { mapping_ = enabled; }
    [[nodiscard]] bool mapping() const { return mapping_; }
    [[nodiscard]] const ConeMap& map() const { return map_; }
    // For the latest add(): the id of the confirmed landmark each observation was matched to,
    // or -1 (same convention as StateEstimator::last_associations).
    [[nodiscard]] const std::vector<std::int32_t>& last_associations() const {
        return associations_;
    }

private:
    struct Landmark {
        Mat2 information = Mat2::Zero();  // sum of the inverse observation covariances
        Vec2 information_vector = Vec2::Zero();
        Vec2 mean = Vec2::Zero();
        std::array<std::uint32_t, kConeColorCount> votes{};
        std::uint32_t sightings = 0;
        std::int32_t id = -1;  // assigned when confirmed
        std::vector<std::pair<std::int32_t, std::uint32_t>> truth_votes;
    };

    void fuse(Landmark& landmark, const Vec2& position, const Mat2& covariance,
              const ConeObservation& observation);
    void publish();

    KnownPoseMapperConfig config_;
    bool mapping_ = true;
    std::vector<Landmark> landmarks_;  // tentative and confirmed, in creation order
    std::int32_t next_id_ = 0;
    ConeMap map_;
    std::vector<std::int32_t> associations_;
};

}  // namespace apex::stack
