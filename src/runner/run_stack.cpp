#include "apex/runner/run_stack.hpp"

#include "apex/stack/stack.hpp"

namespace apex::runner {

RunResult run_stack(const RunConfig& config, RunObserver* observer) {
    validate(config);
    const Track track = make_track(config.track);
    const sim::WorldConfig world = make_world_config(config);
    // The car knows its sensors' geometry (not their noise) and its staging pose, which defines
    // the map frame (docs/design.md, section 3); everything else it learns by driving.
    stack::Stack driver(config.stack, config.nominal, world.rig, track.start_pose, config.oracles);
    return run_closed_loop(config, track, driver, observer);
}

}  // namespace apex::runner
