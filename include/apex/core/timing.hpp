#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace apex {

// Wall-clock time of the code itself, as opposed to simulated time. Used to check every module
// against its real-time budget; never used to make a decision, so runs stay deterministic.
class Stopwatch {
public:
    Stopwatch() : start_(std::chrono::steady_clock::now()) {}
    void restart() { start_ = std::chrono::steady_clock::now(); }
    [[nodiscard]] double elapsed_seconds() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    }

private:
    std::chrono::steady_clock::time_point start_;
};

// Compute-time samples of one function, summarized as the percentiles the budgets are set on.
class LatencyStats {
public:
    void add(double seconds) { samples_.push_back(seconds); }
    void merge(const LatencyStats& other);
    [[nodiscard]] std::size_t count() const { return samples_.size(); }
    [[nodiscard]] double mean() const;
    // Nearest-rank percentile, q in [0, 1]; 0 when empty.
    [[nodiscard]] double percentile(double q) const;
    [[nodiscard]] double max() const;
    // {"count", "mean_us", "p50_us", "p90_us", "p99_us", "max_us"}
    [[nodiscard]] nlohmann::json summary() const;

private:
    std::vector<double> samples_;
};

}  // namespace apex
