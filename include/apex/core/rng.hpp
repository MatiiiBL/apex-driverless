#pragma once

#include <cstddef>
#include <cstdint>

namespace apex {

// Deterministic pseudo-random numbers: xoshiro256++ seeded through splitmix64. Every random
// choice in apex goes through one of these, never std::random_device or the <random>
// distributions, whose output differs between standard libraries. The same seed gives the same
// stream on every platform; results that pass through libm (normal()) can differ in the last
// bits between platforms.
class Rng {
public:
    explicit Rng(std::uint64_t seed = 1);
    // An independent stream for `stream`: Rng(seed).split(k) never overlaps split(j), j != k, in
    // practice. Give each consumer (each sensor, the track generator, ...) its own stream, so
    // adding draws in one never shifts another's numbers.
    [[nodiscard]] Rng split(std::uint64_t stream) const;

    std::uint64_t next_u64();
    double uniform();                      // [0, 1)
    double uniform(double lo, double hi);  // [lo, hi)
    double normal(double mean = 0.0, double stddev = 1.0);
    bool bernoulli(double p);
    std::size_t index(std::size_t n);      // uniform in [0, n), n > 0
    std::uint32_t poisson(double lambda);  // Knuth's method; for small lambda (< ~30)

private:
    std::uint64_t seed_ = 0;
    std::uint64_t state_[4] = {};
    double spare_normal_ = 0.0;
    bool has_spare_ = false;
};

}  // namespace apex
