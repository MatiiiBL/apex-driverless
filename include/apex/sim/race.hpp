#pragma once

#include <cstdint>
#include <vector>

#include "apex/core/track.hpp"
#include "apex/core/vehicle.hpp"

namespace apex::sim {

struct LapRecord {
    int lap = 0;       // 1-based
    Time start = 0.0;  // timing-line crossing that started the lap
    Time end = 0.0;    // crossing that ended it
    [[nodiscard]] double time() const { return end - start; }
};

struct ConeHit {
    Time time = 0.0;
    std::int32_t cone_id = -1;
};

struct OffCourse {
    Time time = 0.0;
    double s = 0.0;  // centerline position where all four wheels left the track
};

// The race officials: timing and penalties per the FS Driverless trackdrive rules, from the
// ground truth. A lap is timed between crossings of the timekeeping line in the driving
// direction. Each cone knocked (the body footprint overlapping its base) costs 2 s, counted
// once per cone; each excursion with all four wheels outside the cone lines costs 10 s.
// Knocked cones stay where they are for the sensors (a simplification).
class RaceMonitor {
public:
    RaceMonitor(const Track& track, const VehicleParams& vehicle);

    // Called by World after every tick.
    void update(const VehicleState& state, Time time);

    [[nodiscard]] int laps_completed() const;
    [[nodiscard]] const std::vector<LapRecord>& laps() const;
    [[nodiscard]] const std::vector<ConeHit>& cone_hits() const;
    [[nodiscard]] const std::vector<OffCourse>& off_course() const;
    [[nodiscard]] double penalty_seconds() const;
    [[nodiscard]] bool started() const;  // crossed the timekeeping line at least once
    // Ground-truth progress: centerline arc length and signed lateral offset of the CoG.
    [[nodiscard]] double s() const;
    [[nodiscard]] double lateral() const;
    // Total distance driven along the centerline since the first crossing (laps * length + s).
    [[nodiscard]] double progress() const;

private:
    // Implementation-defined state; the sim module may add members here.
    const Track* track_;
    VehicleParams vehicle_;
    std::vector<LapRecord> laps_;
    std::vector<ConeHit> cone_hits_;
    std::vector<OffCourse> off_course_;
    std::vector<bool> knocked_;
    bool started_ = false;
    bool outside_ = false;
    Time lap_start_ = 0.0;
    double s_ = 0.0;
    double lateral_ = 0.0;
    double progress_ = 0.0;
};

}  // namespace apex::sim
