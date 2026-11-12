#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

#include "apex/core/trajectory.hpp"
#include "apex/core/types.hpp"

namespace apex {

// A closed trackdrive circuit: the ground truth a run is scored against. Only the simulator,
// the runner and the evaluation see a Track; the car discovers it through its sensors.
struct Track {
    std::string name;
    std::uint64_t seed = 0;  // generator seed, 0 for hand-made tracks
    // Closed centerline sampled every ~0.5 m in the driving direction, with s = 0 on the
    // timekeeping line. width_left / width_right are the distances to the lines through the
    // blue (left) and yellow (right) cone centers.
    Trajectory centerline;
    std::vector<Cone> cones;  // cones[i].id == i
    Pose2 start_pose;         // where the car is staged, at rest, behind the timekeeping line
    Vec2 timing_line_left = Vec2::Zero();  // the timekeeping line runs between these points
    Vec2 timing_line_right = Vec2::Zero();

    [[nodiscard]] std::vector<Vec2> left_boundary() const;   // through the blue cone line
    [[nodiscard]] std::vector<Vec2> right_boundary() const;  // through the yellow cone line
};

struct TrackBuildOptions {
    double half_width = 1.75;       // centerline to cone line (FS minimum track width: 3 m)
    double max_cone_spacing = 5.0;  // FS rules: at most 5 m between cones on one side
    double min_cone_spacing = 1.5;
    double max_chord_sagitta = 0.15;  // cones get denser in corners so a straight line
                                      // between neighbours stays within this of the edge
    double centerline_spacing = 0.5;  // sampling of Track::centerline
    double staging_distance = 6.0;    // car starts this far behind the timekeeping line
    double big_cone_offset = 0.5;     // big orange cones at s = +/- this, on both sides
};

// Builds a track from waypoints along a closed centerline, in the driving direction (the first
// waypoint becomes the timekeeping line). Cones go on both edges at curvature-dependent spacing,
// four big orange cones mark the timekeeping line, and the car is staged on the centerline
// `staging_distance` before it. Throws std::invalid_argument if the result fails validate_track.
[[nodiscard]] Track track_from_centerline(std::span<const Vec2> waypoints,
                                          const TrackBuildOptions& options = {},
                                          std::string name = {}, std::uint64_t seed = 0);

// A stadium-shaped circuit (two straights joined by half circles), driven counter-clockwise.
// Handy for tests: every module can make one without the generator.
[[nodiscard]] Track make_oval_track(double straight_length = 40.0, double radius = 12.0,
                                    const TrackBuildOptions& options = {});

// Throws std::invalid_argument naming the first violated rule: centerline closed and simple,
// boundaries simple and disjoint, radius of curvature everywhere above the half width plus
// 0.5 m, cone ids equal to their index, consecutive cones on a side at most max spacing apart.
void validate_track(const Track& track, const TrackBuildOptions& options = {});

void save_track(const Track& track, const std::filesystem::path& path);  // JSON
[[nodiscard]] Track load_track(const std::filesystem::path& path);

}  // namespace apex
