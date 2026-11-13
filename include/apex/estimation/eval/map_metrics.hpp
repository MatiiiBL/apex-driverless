#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "apex/core/messages.hpp"
#include "apex/core/track.hpp"

// Scores of a map and of the data association against the ground truth, through the truth_id
// evaluation fields (docs/design.md, section 8).
namespace apex::estimation::eval {

struct MapMetrics {
    std::size_t landmarks = 0;
    std::size_t matched_cones = 0;  // distinct true cones with a landmark
    std::size_t duplicates = 0;     // landmarks beyond the first for the same true cone
    std::size_t spurious = 0;       // landmarks of no true cone (truth id -1)
    std::size_t missed = 0;         // mappable true cones without a landmark
    double rms_error = 0.0;         // m, landmark to its true cone, over all non-spurious ones
    double max_error = 0.0;
};

// `mappable` lists the true cones a complete map must contain (by id).
[[nodiscard]] MapMetrics evaluate_map(const ConeMap& map, const Track& track,
                                      std::span<const std::int32_t> mappable);

// Precision and recall of observation-to-landmark matches by truth id. A match is correct when
// the landmark's (majority) truth id is the observation's. Recall counts the observations of
// cones that had a landmark before the scan.
class AssociationScore {
public:
    // `before` is the map before the scan was processed, `after` the map after it.
    void add(const ConeObservations& observations, std::span<const std::int32_t> associations,
             const ConeMap& before, const ConeMap& after);

    [[nodiscard]] std::size_t matches() const { return matches_; }
    [[nodiscard]] std::size_t correct() const { return correct_; }
    [[nodiscard]] double precision() const;
    [[nodiscard]] double recall() const;

private:
    std::size_t matches_ = 0;
    std::size_t correct_ = 0;
    std::size_t matchable_ = 0;
    std::size_t matched_matchable_ = 0;
};

}  // namespace apex::estimation::eval
