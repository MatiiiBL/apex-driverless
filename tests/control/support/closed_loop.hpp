#pragma once

#include <cmath>
#include <functional>
#include <vector>

#include "apex/control/controller.hpp"
#include "apex/core/trajectory.hpp"

#include "test_plant.hpp"

// A controller and the test plant in a loop at the stack's 50 Hz, with the true state as the
// estimate: the tests' closed-loop harness.
namespace apex::control::testing {

inline constexpr double kControlPeriod = 0.02;  // s

struct LoopSample {
    Time time = 0.0;
    VehicleState truth;
    ControlCommand command;
    double lateral = 0.0;  // signed distance of the CoG from the reference
    double s = 0.0;        // station of the CoG on the reference
};

// Runs for `duration` seconds. `reference_at` gives the reference for each call, so a test can
// replan it.
inline std::vector<LoopSample> run_loop(Controller& controller, TestPlant& plant,
                                        const std::function<const Trajectory&(Time)>& reference_at,
                                        double duration) {
    std::vector<LoopSample> samples;
    const auto steps = static_cast<long>(std::lround(duration / kControlPeriod));
    samples.reserve(static_cast<std::size_t>(steps));
    for (long i = 0; i < steps; ++i) {
        const Time now = plant.time();
        const Trajectory& reference = reference_at(now);
        const ControlCommand command = controller.compute(plant.estimate(), reference, now);
        plant.command(command);
        LoopSample sample;
        sample.time = now;
        sample.truth = plant.truth();
        sample.command = command;
        const Trajectory::Projection p = reference.project(plant.truth().pose.position());
        sample.lateral = p.lateral;
        sample.s = p.s;
        samples.push_back(sample);
        plant.advance(kControlPeriod);
    }
    return samples;
}

inline std::vector<LoopSample> run_loop(Controller& controller, TestPlant& plant,
                                        const Trajectory& reference, double duration) {
    return run_loop(
        controller, plant, [&reference](Time) -> const Trajectory& { return reference; }, duration);
}

// The largest |lateral| among samples at or after time `from`.
[[nodiscard]] inline double max_lateral_after(const std::vector<LoopSample>& samples, Time from) {
    double worst = 0.0;
    for (const LoopSample& s : samples) {
        if (s.time >= from) worst = std::max(worst, std::abs(s.lateral));
    }
    return worst;
}

}  // namespace apex::control::testing
