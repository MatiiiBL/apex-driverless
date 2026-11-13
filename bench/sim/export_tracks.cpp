// Writes the reference layouts and the generated tracks of seeds 1-6 as JSON, for the gallery
// in docs/sim.md:
//
//   ./build/release/bench/sim/apex_export_tracks build/tracks
//   uv run --project tools python tools/sim/plot_tracks.py build/tracks docs/img/sim-tracks.png
#include <cstdint>
#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "apex/core/track.hpp"
#include "apex/sim/track_gen.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "usage: apex_export_tracks <directory>\n";
        return 2;
    }
    try {
        const std::filesystem::path directory = argv[1];
        std::filesystem::create_directories(directory);
        for (const std::string& name : apex::sim::reference_track_names()) {
            apex::save_track(apex::sim::reference_track(name), directory / (name + ".json"));
        }
        for (std::uint64_t seed = 1; seed <= 6; ++seed) {
            const apex::Track track = apex::sim::generate_track(seed);
            apex::save_track(track, directory / (track.name + ".json"));
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << "\n";
        return 1;
    }
    return 0;
}
