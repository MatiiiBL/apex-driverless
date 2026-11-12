#include "apex/core/timing.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <nlohmann/json.hpp>

namespace apex {

void LatencyStats::merge(const LatencyStats& other) {
    samples_.insert(samples_.end(), other.samples_.begin(), other.samples_.end());
}

double LatencyStats::mean() const {
    if (samples_.empty()) return 0.0;
    return std::accumulate(samples_.begin(), samples_.end(), 0.0) /
           static_cast<double>(samples_.size());
}

double LatencyStats::percentile(double q) const {
    if (samples_.empty()) return 0.0;
    std::vector<double> sorted = samples_;
    const double rank = std::ceil(std::clamp(q, 0.0, 1.0) * static_cast<double>(sorted.size()));
    const std::size_t index =
        std::clamp<std::size_t>(static_cast<std::size_t>(rank), 1, sorted.size()) - 1;
    std::nth_element(sorted.begin(), sorted.begin() + static_cast<std::ptrdiff_t>(index),
                     sorted.end());
    return sorted[index];
}

double LatencyStats::max() const {
    if (samples_.empty()) return 0.0;
    return *std::max_element(samples_.begin(), samples_.end());
}

nlohmann::json LatencyStats::summary() const {
    constexpr double kMicro = 1e6;
    return {{"count", count()},
            {"mean_us", mean() * kMicro},
            {"p50_us", percentile(0.50) * kMicro},
            {"p90_us", percentile(0.90) * kMicro},
            {"p99_us", percentile(0.99) * kMicro},
            {"max_us", max() * kMicro}};
}

}  // namespace apex
