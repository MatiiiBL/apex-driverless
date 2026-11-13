#pragma once

#include <vector>

#include "apex/core/messages.hpp"
#include "apex/core/rng.hpp"
#include "apex/core/track.hpp"

// Cone maps like the ones EKF-SLAM builds, made from a track's true cones with the errors a
// real map has, and the driving that reveals them. For the planning module's tests and
// evaluations: the planners only ever see the ConeMap.
namespace apex::planning::test_support {

struct MapErrors {
    // RMS of a landmark's 2D position error (m): isotropic Gaussian, rms / sqrt(2) per axis.
    double position_rms = 0.10;
    double miss_probability = 0.10;         // a cone never mapped
    double color_error_probability = 0.05;  // mapped with a wrong color or as Unknown, 50/50
    // Landmarks where there is no cone (clutter that survived SLAM's tentative stage), placed
    // outside the track, between these distances beyond a cone line.
    double false_positives_per_100m = 2.0;
    double false_positive_min_offset = 1.0;  // m
    double false_positive_max_offset = 5.0;  // m
};

// The worst map the planning module is required to cope with: 15 cm RMS position error, 20% of
// the cones missing, 10% with a wrong or unknown color, and twice the default clutter.
[[nodiscard]] MapErrors worst_case_errors();

// The map of the whole track, as after lap 1. Landmark ids are their index; truth_id is the
// true cone's id (-1 for clutter).
[[nodiscard]] ConeMap make_cone_map(const Track& track, const MapErrors& errors, Rng& rng);

// What a car has mapped so far as it drives: the landmarks of a full map that were within
// `range` and in the front half-plane of at least one pose observed.
class Sightings {
public:
    Sightings(ConeMap full, double range);
    void observe(const Pose2& pose);
    [[nodiscard]] ConeMap map() const;

private:
    ConeMap full_;
    double range_ = 0.0;
    std::vector<bool> seen_;
};

// The arc length of the track's staging pose on its centerline, in [0, length).
[[nodiscard]] double staging_s(const Track& track);

// A pose on the centerline at arc length s (wrapped), moved `lateral` meters to the left and
// turned by `yaw_error` from the centerline's heading.
[[nodiscard]] Pose2 centerline_pose(const Track& track, double s, double lateral = 0.0,
                                    double yaw_error = 0.0);

// A lap-1 drive: poses every `step` meters from the staging pose through the whole lap and
// `overrun` meters past the timing line, wandering around the centerline by a smooth random
// lateral offset of at most `wander` meters.
[[nodiscard]] std::vector<Pose2> driven_lap(const Track& track, double wander, Rng& rng,
                                            double step = 0.5, double overrun = 5.0);

}  // namespace apex::planning::test_support
