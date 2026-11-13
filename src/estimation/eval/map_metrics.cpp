#include "apex/estimation/eval/map_metrics.hpp"

#include <algorithm>
#include <cmath>

namespace apex::estimation::eval {

MapMetrics evaluate_map(const ConeMap& map, const Track& track,
                        std::span<const std::int32_t> mappable) {
    MapMetrics m;
    m.landmarks = map.cones.size();
    std::vector<std::uint32_t> per_cone(track.cones.size(), 0);
    double sum_sq = 0.0;
    std::size_t measured = 0;
    for (const MapCone& cone : map.cones) {
        if (cone.truth_id < 0 || static_cast<std::size_t>(cone.truth_id) >= track.cones.size()) {
            ++m.spurious;
            continue;
        }
        const auto id = static_cast<std::size_t>(cone.truth_id);
        if (per_cone[id]++ > 0) ++m.duplicates;
        const double error = (cone.position - track.cones[id].position).norm();
        sum_sq += error * error;
        m.max_error = std::max(m.max_error, error);
        ++measured;
    }
    for (const std::uint32_t count : per_cone) {
        if (count > 0) ++m.matched_cones;
    }
    for (const std::int32_t id : mappable) {
        if (id >= 0 && static_cast<std::size_t>(id) < per_cone.size() &&
            per_cone[static_cast<std::size_t>(id)] == 0) {
            ++m.missed;
        }
    }
    if (measured > 0) m.rms_error = std::sqrt(sum_sq / static_cast<double>(measured));
    return m;
}

namespace {

const MapCone* find(const ConeMap& map, std::int32_t id) {
    const auto it = std::lower_bound(map.cones.begin(), map.cones.end(), id,
                                     [](const MapCone& c, std::int32_t v) { return c.id < v; });
    return it != map.cones.end() && it->id == id ? &*it : nullptr;
}

}  // namespace

void AssociationScore::add(const ConeObservations& observations,
                           std::span<const std::int32_t> associations, const ConeMap& before,
                           const ConeMap& after) {
    std::vector<std::int32_t> mapped;
    for (const MapCone& cone : before.cones) {
        if (cone.truth_id >= 0) mapped.push_back(cone.truth_id);
    }
    std::sort(mapped.begin(), mapped.end());
    const std::size_t n = std::min(observations.cones.size(), associations.size());
    for (std::size_t i = 0; i < n; ++i) {
        const std::int32_t truth = observations.cones[i].truth_id;
        const MapCone* landmark = associations[i] >= 0 ? find(after, associations[i]) : nullptr;
        const bool correct = landmark != nullptr && truth >= 0 && landmark->truth_id == truth;
        if (landmark != nullptr) {
            ++matches_;
            if (correct) ++correct_;
        }
        if (truth >= 0 && std::binary_search(mapped.begin(), mapped.end(), truth)) {
            ++matchable_;
            if (correct) ++matched_matchable_;
        }
    }
}

double AssociationScore::precision() const {
    return matches_ > 0 ? static_cast<double>(correct_) / static_cast<double>(matches_) : 1.0;
}

double AssociationScore::recall() const {
    return matchable_ > 0
               ? static_cast<double>(matched_matchable_) / static_cast<double>(matchable_)
               : 1.0;
}

}  // namespace apex::estimation::eval
