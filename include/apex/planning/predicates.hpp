#pragma once

#include "apex/core/types.hpp"

// Exact geometric predicates on points with double coordinates, after Shewchuk, "Adaptive
// Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates", Discrete &
// Computational Geometry 18(3), 1997.
//
// Each predicate first evaluates its determinant in plain floating point together with a
// forward error bound; when the result's magnitude exceeds the bound (nearly always), its sign
// is certain. Otherwise the determinant is evaluated exactly in expansion arithmetic (a sum of
// non-overlapping doubles), so the sign is always the sign of the exact real determinant: zero
// means exactly collinear or cocircular, and no epsilon is involved anywhere.
namespace apex::planning {

// +1 when a, b, c run counter-clockwise, -1 when clockwise, 0 when collinear.
[[nodiscard]] int orientation(const Vec2& a, const Vec2& b, const Vec2& c);

// For a, b, c counter-clockwise: +1 when d lies strictly inside their circumcircle, -1 strictly
// outside, 0 on it. The sign flips for a clockwise triangle.
[[nodiscard]] int in_circle(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d);

}  // namespace apex::planning
