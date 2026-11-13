#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "apex/core/track.hpp"
#include "apex/sim/track_rules.hpp"

// Random trackdrive circuits and a few fixed reference layouts (docs/sim.md, "Tracks").
namespace apex::sim {

struct TrackGenOptions {
    TrackRules rules;
    // Cone placement and staging. The default half width (1.75 m) gives the 3.5 m apex track.
    TrackBuildOptions build;
    // Relative frequency of each feature. Every track also gets at least one hairpin, one
    // chicane and one sweeper; these weights draw the rest.
    double hairpin_weight = 0.20;
    double corner_weight = 0.35;
    double sweeper_weight = 0.25;
    double chicane_weight = 0.20;
    // Candidates tried before giving up. About one in fourteen passes every rule.
    int max_attempts = 1000;
};

// A random closed circuit meeting `options.rules` and validate_track, the same for the same
// seed and options on every platform (up to libm's last bits).
//
// Each candidate is a sequence of features (hairpins, medium corners, sweepers, chicanes)
// separated by straights, laid out as a curvature profile with clothoid transitions. The turn
// angles are scaled so the heading closes (total turning +/-360 degrees), the straight lengths
// are solved for so the position closes, and the result is checked against the rules; failing
// candidates are dropped and the next one drawn. The timekeeping line is on the start straight,
// at the origin, heading +x. Throws std::invalid_argument for inconsistent options and
// std::runtime_error if max_attempts candidates all fail.
[[nodiscard]] Track generate_track(std::uint64_t seed, const TrackGenOptions& options = {});

struct GeneratedTrack {
    Track track;
    int attempts = 0;  // candidates drawn, including the accepted one
};
// generate_track, also reporting how many candidates it drew.
[[nodiscard]] GeneratedTrack generate_track_with_stats(std::uint64_t seed,
                                                       const TrackGenOptions& options = {});

// Fixed layouts for repeatable experiments: "technical" (short and tight: hairpins and
// chicanes), "flowing" (long and fast: sweepers and long straights) and "mixed".
[[nodiscard]] std::vector<std::string> reference_track_names();
// Throws std::invalid_argument for an unknown name.
[[nodiscard]] Track reference_track(std::string_view name, const TrackBuildOptions& build = {});

}  // namespace apex::sim
