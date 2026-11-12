#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "apex/core/types.hpp"

namespace apex::planning {

// A triangulation of a point set, as indices into the points it was built from.
struct Triangulation {
    // Counter-clockwise triangles.
    std::vector<std::array<std::int32_t, 3>> triangles;
    // neighbors[t][i] is the triangle across the edge opposite vertex i of triangle t (the edge
    // from vertex i + 1 to vertex i + 2), or -1 where that edge is on the convex hull.
    std::vector<std::array<std::int32_t, 3>> neighbors;
};

// The Delaunay triangulation of `points` (finite coordinates): no point lies strictly inside
// any triangle's circumcircle, and the triangles cover the convex hull. Where four or more
// points are cocircular the triangulation is one of the valid ones, chosen deterministically.
//
// Bowyer-Watson insertion (Bowyer, "Computing Dirichlet tessellations", and Watson, "Computing
// the n-dimensional Delaunay tessellation...", both The Computer Journal 24(2), 1981), with the
// hull closed by ghost triangles to a vertex at infinity instead of a large super-triangle, as
// in Cheng, Dey and Shewchuk, "Delaunay Mesh Generation", CRC Press, 2012, chapter 3. Each new
// point is located by a visibility walk from the last triangle created, and every decision is an
// exact predicate (apex/planning/predicates.hpp), so degenerate inputs (collinear points,
// cocircular points) are handled without tolerances. A point equal to an earlier one is left out
// of every triangle. Fewer than three distinct points, or only collinear ones, give no
// triangles.
[[nodiscard]] Triangulation delaunay_triangulation(std::span<const Vec2> points);

}  // namespace apex::planning
