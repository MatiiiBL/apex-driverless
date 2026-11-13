#pragma once

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/runner/run.hpp"

// Observers that record a run for later inspection: a CSV time series for analysis and a
// compact replay for animations. Both sample at their own period, never every tick, so a
// ten-lap run stays a few megabytes.
namespace apex::runner {

// One CSV row every `period` seconds of simulated time (at the first control step at or after
// it): time, mission state, the true state, the driver's estimate, the command, the officials'
// centerline position, laps and cone hits, and the size of the driver's map. Units are SI;
// angles in radians.
class CsvLog final : public RunObserver {
public:
    // Throws std::invalid_argument for a non-positive period.
    CsvLog(std::ostream& out, double period);

    void on_start(const Track& track) override;
    void on_control(const TickView& tick) override;

    [[nodiscard]] std::size_t rows() const { return rows_; }

private:
    std::ostream& out_;
    double period_;
    double next_ = 0.0;
    std::size_t rows_ = 0;
};

struct ReplayOptions {
    double frame_period = 0.05;     // s between car poses (20 frames per second)
    double map_period = 1.0;        // s between map snapshots while the map is being built
    double reference_period = 0.5;  // s between snapshots of a changing reference
    double path_spacing = 1.0;      // m between the reference points kept
};

// Everything an animation of the run needs, as JSON (positions rounded to the millimeter):
//   {"track":      {"name", "centerline": [[x, y], ...], "cones": [[x, y, "blue"], ...],
//                   "timing_line": [[x, y], [x, y]]},
//    "frames":     [{"t", "state", "x", "y", "yaw", "speed", "est": [x, y, yaw]}, ...],
//    "maps":       [{"t", "cones": [[x, y, "blue"], ...]}, ...],
//    "references": [{"t", "state", "closed", "points": [[x, y], ...]}, ...]}
// Maps are recorded while the mission is building them (until it races on the frozen map, of
// which one last snapshot is kept); references whenever the one being tracked changes, at most
// once per reference_period unless the mission state changed.
class ReplayRecorder final : public RunObserver {
public:
    // Throws std::invalid_argument for a non-positive period or spacing.
    explicit ReplayRecorder(const ReplayOptions& options = {});

    void on_start(const Track& track) override;
    void on_control(const TickView& tick) override;

    [[nodiscard]] const nlohmann::json& replay() const { return replay_; }

private:
    void record_map(const TickView& tick);
    void record_reference(const TickView& tick);

    ReplayOptions options_;
    nlohmann::json replay_;
    double next_frame_ = 0.0;
    double last_map_ = 0.0;
    bool have_map_ = false;
    bool map_frozen_ = false;
    std::uint64_t map_revision_ = 0;
    double last_reference_ = 0.0;
    bool have_reference_ = false;
    std::uint8_t reference_state_ = 0;
    std::vector<double> reference_key_;  // what identifies the last reference recorded
};

}  // namespace apex::runner
