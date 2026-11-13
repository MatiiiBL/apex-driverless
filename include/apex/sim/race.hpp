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
//
// The lap timer is a light barrier on the timekeeping line, tripped by the CoG between the big
// orange cones (widened by half the body width). It counts net crossings: one against the
// driving direction cancels the last forward one, so a car hovering on the line starts its lap
// once and finishes it once. An off-course excursion begins when all four wheel contact points
// are outside the cone lines and ends when the CoG is back between them.
class RaceMonitor {
public:
    // Keeps a reference to `track`, which must outlive the monitor. Throws std::invalid_argument
    // for a track without a centerline.
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
    void check_timing_line(const Vec2& position, Time time);
    void check_cones(const VehicleState& state, Time time);
    void check_off_course(const VehicleState& state, Time time);

    // Implementation-defined state; the sim module may add members here.
    const Track* track_;
    Vec2 line_center_ = Vec2::Zero();   // midpoint of the timekeeping line
    Vec2 line_forward_ = Vec2::Zero();  // unit, the driving direction across it
    Vec2 line_along_ = Vec2::Zero();    // unit, from its right end to its left end
    double line_half_length_ = 0.0;     // m, including half the body width
    Vec2 previous_position_ = Vec2::Zero();
    Time previous_time_ = 0.0;
    bool has_previous_ = false;
    int net_crossings_ = 0;   // forward minus backward crossings of the timekeeping line
    int best_crossings_ = 0;  // the most net_crossings_ has been: a lap ends when it grows
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
