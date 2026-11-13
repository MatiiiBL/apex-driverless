#include <format>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"
#include "apex/core/track.hpp"
#include "apex/sim/track_gen.hpp"
#include "apex/sim/track_rules.hpp"

#include "cli.hpp"
#include "commands.hpp"

namespace apex::cli {

namespace {

std::string count(int n, std::string_view noun) {
    return std::format("{} {}{}", n, noun, n == 1 ? "" : "s");
}

int run_track(std::span<const std::string_view> args) {
    const Options options(args, {"seed", "layout", "out"}, {"oval"});
    const int sources = static_cast<int>(options.has("seed")) +
                        static_cast<int>(options.has("layout")) +
                        static_cast<int>(options.has("oval"));
    if (sources != 1) throw UsageError("give exactly one of --seed, --layout and --oval");
    const std::string out = options.required("out");

    Track track;
    if (const auto seed = options.get_unsigned("seed")) {
        track = sim::generate_track(*seed);
    } else if (const auto layout = options.get("layout")) {
        track = sim::reference_track(*layout);
    } else {
        track = make_oval_track();
    }
    write_file(out, nlohmann::json(track).dump(1) + "\n");

    const sim::TrackStats stats = sim::analyze_track(track);
    std::cout << std::format(
        "{}: {:.1f} m, {} cones; tightest radius {:.1f} m, longest straight {:.1f} m; {}, {}, "
        "{}, {} -> {}\n",
        track.name.empty() ? "track" : track.name, stats.length, track.cones.size(),
        stats.min_radius, stats.longest_straight, count(stats.hairpins, "hairpin"),
        count(stats.chicanes, "chicane"), count(stats.sweepers, "sweeper"),
        count(stats.corners, "other corner"), out);
    return 0;
}

}  // namespace

const Command kTrackCommand{"track", "generate a trackdrive circuit and save it as JSON",
                            R"(apex track: generate a trackdrive circuit and save it as JSON.

Usage:
  apex track --seed N --out track.json
  apex track --layout NAME --out track.json
  apex track --oval --out track.json

Options:
  --seed N        a random circuit from the generator; the same N always gives the same track
  --layout NAME   a fixed reference layout: technical, flowing or mixed
  --oval          the 40 m x 12 m stadium oval used in tests
  --out PATH      where to write the track (the JSON form of apex::save_track)

The file can be raced with {"track": {"file": "track.json"}} in a run config.
)",
                            run_track};

}  // namespace apex::cli
