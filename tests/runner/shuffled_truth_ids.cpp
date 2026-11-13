#include "shuffled_truth_ids.hpp"

#include <numeric>
#include <utility>
#include <variant>

#include "apex/core/rng.hpp"

namespace apex::runner::test_drivers {

ShuffledTruthIds::ShuffledTruthIds(stack::Driver& inner, std::size_t cone_count, std::uint64_t seed)
    : inner_(inner), permutation_(cone_count) {
    std::iota(permutation_.begin(), permutation_.end(), 0);
    // Fisher-Yates.
    Rng rng(seed);
    for (std::size_t i = permutation_.size(); i > 1; --i) {
        std::swap(permutation_[i - 1], permutation_[rng.index(i)]);
    }
}

std::int32_t ShuffledTruthIds::relabel(std::int32_t id) const {
    if (id < 0 || static_cast<std::size_t>(id) >= permutation_.size()) return id;
    return permutation_[static_cast<std::size_t>(id)];
}

void ShuffledTruthIds::on_sensor(const SensorMessage& message) {
    if (const auto* scan = std::get_if<LidarScan>(&message)) {
        LidarScan relabelled = *scan;
        for (LidarPoint& point : relabelled.points) point.truth_id = relabel(point.truth_id);
        inner_.on_sensor(relabelled);
    } else if (const auto* frame = std::get_if<CameraFrame>(&message)) {
        CameraFrame relabelled = *frame;
        for (CameraDetection& d : relabelled.detections) d.truth_id = relabel(d.truth_id);
        inner_.on_sensor(relabelled);
    } else {
        inner_.on_sensor(message);
    }
}

void ShuffledTruthIds::on_oracle_observations(const ConeObservations& observations) {
    ConeObservations relabelled = observations;
    for (ConeObservation& o : relabelled.cones) o.truth_id = relabel(o.truth_id);
    inner_.on_oracle_observations(relabelled);
}

}  // namespace apex::runner::test_drivers
