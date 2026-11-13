#pragma once

#include "apex/runner/config.hpp"
#include "apex/runner/result.hpp"
#include "apex/runner/run.hpp"

namespace apex::runner {

// A closed-loop run of the full stack: the track the config names, a World built from it and a
// stack::Stack that knows the sensor geometry, the nominal car and its staging pose. Throws
// std::invalid_argument for an invalid config and std::runtime_error for an unreadable track
// file.
[[nodiscard]] RunResult run_stack(const RunConfig& config, RunObserver* observer = nullptr);

}  // namespace apex::runner
