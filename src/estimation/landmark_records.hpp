#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "apex/core/messages.hpp"
#include "apex/core/types.hpp"

namespace apex::estimation {

// Counts of the truth ids behind a landmark's observations, for the evaluation only: nothing
// in the estimator reads them. Ties go to the id seen first, so a bijective relabelling of the
// truth ids relabels the result and changes nothing else.
class TruthTally {
public:
    void add(std::int32_t truth_id);
    [[nodiscard]] std::int32_t majority() const;

private:
    std::vector<std::pair<std::int32_t, std::uint32_t>> counts_;  // in first-seen order
};

// What the estimator keeps beside each landmark's filter state.
struct LandmarkRecord {
    std::int32_t id = -1;  // assigned on confirmation, never reused
    bool confirmed = false;
    std::uint32_t detections = 0;  // scans it was observed in
    Time last_seen = 0.0;
    std::array<std::uint32_t, kConeColorCount> votes{};
    TruthTally truth;

    void observe(const ConeObservation& observation, Time stamp);
    // The color with the most votes; Unknown when no color was seen or the top two tie.
    [[nodiscard]] ConeColor color() const;
};

}  // namespace apex::estimation
