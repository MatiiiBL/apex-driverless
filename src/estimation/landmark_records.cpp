#include "landmark_records.hpp"

namespace apex::estimation {

void TruthTally::add(std::int32_t truth_id) {
    for (auto& [id, count] : counts_) {
        if (id == truth_id) {
            ++count;
            return;
        }
    }
    counts_.emplace_back(truth_id, 1U);
}

std::int32_t TruthTally::majority() const {
    std::int32_t best = -1;
    std::uint32_t best_count = 0;
    for (const auto& [id, count] : counts_) {
        if (count > best_count) {
            best = id;
            best_count = count;
        }
    }
    return best;
}

void LandmarkRecord::observe(const ConeObservation& observation, Time stamp) {
    ++detections;
    last_seen = stamp;
    ++votes[index_of(observation.color)];
    truth.add(observation.truth_id);
}

ConeColor LandmarkRecord::color() const {
    ConeColor best = ConeColor::Unknown;
    std::uint32_t best_votes = 0;
    bool tie = false;
    for (std::size_t i = 0; i < kConeColorCount; ++i) {
        const auto color = static_cast<ConeColor>(i);
        if (color == ConeColor::Unknown) continue;
        if (votes[i] > best_votes) {
            best = color;
            best_votes = votes[i];
            tie = false;
        } else if (votes[i] == best_votes && best_votes > 0) {
            tie = true;
        }
    }
    return tie ? ConeColor::Unknown : best;
}

}  // namespace apex::estimation
