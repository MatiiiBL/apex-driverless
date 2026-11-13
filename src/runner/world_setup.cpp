#include <stdexcept>

#include "apex/core/track.hpp"
#include "apex/runner/run.hpp"
#include "apex/sim/sensors.hpp"
#include "apex/sim/track_gen.hpp"

namespace apex::runner {

Track make_track(const TrackSpec& spec) {
    switch (spec.source) {
        case TrackSpec::Source::Oval:
            return make_oval_track(spec.oval_straight, spec.oval_radius);
        case TrackSpec::Source::Generated:
            return sim::generate_track(spec.seed);
        case TrackSpec::Source::Layout:
            return sim::reference_track(spec.layout);
        case TrackSpec::Source::File:
            return load_track(spec.file);
    }
    throw std::invalid_argument("track: unknown source");
}

sim::WorldConfig make_world_config(const RunConfig& config) {
    sim::WorldConfig world;
    world.vehicle = config.vehicle;
    world.noise = sim::scale_noise(config.noise, config.sensor_noise_scale);
    world.friction_scale = config.friction_scale;
    world.seed = config.seed;
    return world;
}

}  // namespace apex::runner
