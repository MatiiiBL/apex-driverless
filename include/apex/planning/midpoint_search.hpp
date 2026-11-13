#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "apex/core/types.hpp"
#include "apex/planning/planning.hpp"

// The search at the heart of the exploration planner (docs/planning.md, "Exploration").
namespace apex::planning {

// A mapped cone as the search sees it.
struct SearchCone {
    Vec2 position = Vec2::Zero();
    // Which side of the track its color puts it on: +1 for blue (left), -1 for yellow (right),
    // scaled by the share of the map's color votes behind that color; 0 for orange or unknown.
    double side = 0.0;
};

// The best path found: the midpoints of the triangulation edges it crosses, in order, and the
// cones on either side of each.
struct MidpointPath {
    std::vector<Vec2> midpoints;
    std::vector<std::int32_t> left;  // index into the cones, per midpoint
    std::vector<std::int32_t> right;
    double length = 0.0;  // from the car through every midpoint (m)
    double cost = 0.0;
};

// Beam search for the sequence of edge midpoints that best looks like a track ahead of `car`.
//
// The cones are triangulated (delaunay_triangulation). A path is a walk through adjacent
// triangles: it starts by crossing an edge ahead of the car within config.start_radius, enters
// the triangle beyond, and leaves each triangle through one of its two other edges, so the
// edges it crosses are the rungs and diagonals of a ladder between the two cone lines, and the
// third edge of each triangle it traverses is a segment of one cone line. Each crossed edge
// adds to the cost (ExplorationConfig, "Scores"):
//   - width: the edge's extent across the direction of travel against the nominal width,
//   - curvature: the turn between consecutive midpoints over the distance between them, or,
//     for the first edge, the turn from the car's heading,
//   - color: a cone whose color puts it on the other side of the path,
//   - spacing: a cone-line segment longer than the nominal spacing (missing cones),
// and subtracts a reward per meter, so that a longer path wins unless its extra edges look
// wrong. The hard limits of ExplorationConfig prune a candidate outright. At each depth the
// beam_width cheapest candidates are kept, and a candidate entering a triangle through the same
// edge as a cheaper one already found is dropped (its future is nearly the same). The scheme
// follows the boundary estimation of Kabzan et al., "AMZ Driverless: The full autonomous racing
// system", Journal of Field Robotics 37(7), 2020, with these scoring terms.
//
// Nothing if no edge ahead of the car can start a path.
[[nodiscard]] std::optional<MidpointPath> search_midpoint_path(std::span<const SearchCone> cones,
                                                               const Pose2& car,
                                                               const ExplorationConfig& config);

}  // namespace apex::planning
