#pragma once

#include <optional>
#include <string>

#include "apex/core/track.hpp"

// The geometric rules a trackdrive circuit must meet, and the measurements they are checked on.
// The FS Driverless trackdrive rules (closed loop, 3 m minimum width, straights up to 80 m,
// hairpins, chicanes and constant turns) tightened to the apex limits of docs/design.md, 5.1.
namespace apex::sim {

struct TrackRules {
    double min_length = 200.0;   // centerline length (m)
    double max_length = 500.0;   // m
    double min_radius = 6.0;     // centerline radius of curvature (m)
    double max_straight = 80.0;  // longest flat-out section (m), see straight_curvature
    double min_clearance = 4.0;  // between the cone lines of non-adjacent parts of the track (m)
    // Two centerline points closer than this along the track belong to the same part of it
    // (m). 20 m exceeds the half circle of a 6 m hairpin, so the legs of a hairpin must keep
    // their clearance, and a turn that keeps turning past ~280 degrees fails.
    double adjacency_length = 20.0;
    // Sections with |curvature| below this (1/m) count as straight: at a 50 m radius the car is
    // flat out, so for top speed it is a straight.
    double straight_curvature = 0.02;
};

// What a circuit looks like, measured on its sampled centerline.
struct TrackStats {
    double length = 0.0;            // m
    double min_radius = 0.0;        // smallest radius of curvature (m)
    double longest_straight = 0.0;  // longest run below TrackRules::straight_curvature (m)
    double min_clearance = 0.0;     // smallest gap between cone lines of non-adjacent parts (m)
    // Turns are maximal runs of same-signed curvature above straight_curvature, classified as:
    int hairpins = 0;  // turning at least 130 degrees at a radius of 10 m or less
    int chicanes = 0;  // two opposite turns of 15-100 degrees within 10 m (counted once)
    int sweepers = 0;  // turning at least 30 degrees with radius 14 m or more throughout
    int corners = 0;   // every other turn
};

[[nodiscard]] TrackStats analyze_track(const Track& track, const TrackRules& rules = {});

// The first rule the track breaks, as a readable message, or nothing if it meets them all.
// Covers length, radius, straights and clearance; validate_track covers the rest.
[[nodiscard]] std::optional<std::string> check_track_rules(const Track& track,
                                                           const TrackRules& rules = {});

}  // namespace apex::sim
