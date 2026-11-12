#include "apex/core/rng.hpp"

#include <algorithm>
#include <cmath>

namespace apex {

namespace {

std::uint64_t splitmix64(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

std::uint64_t rotl(std::uint64_t x, int k) {
    return (x << k) | (x >> (64 - k));
}

}  // namespace

Rng::Rng(std::uint64_t seed) : seed_(seed) {
    std::uint64_t sm = seed;
    for (std::uint64_t& word : state_) word = splitmix64(sm);
}

Rng Rng::split(std::uint64_t stream) const {
    std::uint64_t sm = stream ^ 0xD1B54A32D192ED03ULL;
    const std::uint64_t mixed = splitmix64(sm);
    return Rng(seed_ ^ mixed);
}

std::uint64_t Rng::next_u64() {
    const std::uint64_t result = rotl(state_[0] + state_[3], 23) + state_[0];
    const std::uint64_t t = state_[1] << 17;
    state_[2] ^= state_[0];
    state_[3] ^= state_[1];
    state_[1] ^= state_[2];
    state_[0] ^= state_[3];
    state_[2] ^= t;
    state_[3] = rotl(state_[3], 45);
    return result;
}

double Rng::uniform() {
    return static_cast<double>(next_u64() >> 11) * 0x1.0p-53;
}

double Rng::uniform(double lo, double hi) {
    return lo + (hi - lo) * uniform();
}

double Rng::normal(double mean, double stddev) {
    if (has_spare_) {
        has_spare_ = false;
        return mean + stddev * spare_normal_;
    }
    // Marsaglia's polar method: two independent normals per accepted pair.
    double u = 0.0;
    double v = 0.0;
    double s = 0.0;
    do {
        u = 2.0 * uniform() - 1.0;
        v = 2.0 * uniform() - 1.0;
        s = u * u + v * v;
    } while (s >= 1.0 || s == 0.0);
    const double factor = std::sqrt(-2.0 * std::log(s) / s);
    spare_normal_ = v * factor;
    has_spare_ = true;
    return mean + stddev * u * factor;
}

bool Rng::bernoulli(double p) {
    return uniform() < p;
}

std::size_t Rng::index(std::size_t n) {
    const auto i = static_cast<std::size_t>(uniform() * static_cast<double>(n));
    return std::min(i, n - 1);
}

std::uint32_t Rng::poisson(double lambda) {
    if (!(lambda > 0.0)) return 0;
    const double limit = std::exp(-lambda);
    std::uint32_t k = 0;
    double product = uniform();
    while (product > limit) {
        ++k;
        product *= uniform();
    }
    return k;
}

}  // namespace apex
