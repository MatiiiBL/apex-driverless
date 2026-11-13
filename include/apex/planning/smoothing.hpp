#pragma once

#include <optional>
#include <span>
#include <vector>

#include "apex/core/types.hpp"

// Polyline utilities shared by the exploration planner and the track model.
namespace apex::planning {

// Points at equal arc-length steps along a polyline, the step as close to `spacing` as divides
// the length evenly, interpolated linearly between vertices. An open polyline keeps both ends;
// a closed one wraps through its first point, which is not repeated at the end. Throws
// std::invalid_argument for a non-positive spacing.
[[nodiscard]] std::vector<Vec2> resample_polyline(std::span<const Vec2> points, bool closed,
                                                  double spacing);

struct SmoothingOptions {
    double spacing = 1.0;  // arc length between consecutive input points (m)
    // Wiggles of this wavelength (m) are halved; longer ones pass almost unchanged, shorter
    // ones are damped like (wavelength / cutoff)^4.
    double cutoff_wavelength = 10.0;
    bool closed = false;
    // Open polylines only: keep the first point where it is and make the path leave it along
    // this unit direction, as a car leaves its current pose (the second point is moved one
    // spacing along it).
    std::optional<Vec2> start_direction;
};

// The Whittaker smoother (Whittaker, "On a new method of graduation", Proceedings of the
// Edinburgh Mathematical Society 41, 1923; as a penalized least-squares smoother in Eilers, "A
// perfect smoother", Analytical Chemistry 75(14), 2003) on equally spaced points m_i:
//
//   minimize  sum_i |p_i - m_i|^2 + lambda sum_i |p_{i-1} - 2 p_i + p_{i+1}|^2,
//
// a discrete smoothing spline. Its response to a sinusoid of wavenumber k at spacing h is
// 1 / (1 + lambda k^4 h^4), so lambda = (cutoff / (2 pi h))^4 sets the half-gain wavelength
// independently of the sampling. A circle of radius R shrinks by (cutoff / 2 pi)^4 / R^3:
// 3 cm for a 10 m cutoff on a 6 m hairpin. The normal equations are banded (cyclic when
// closed) and solved with a sparse Cholesky factorization. Fewer than three points (four if
// closed) are returned unchanged.
[[nodiscard]] std::vector<Vec2> smooth_polyline(std::span<const Vec2> points,
                                                const SmoothingOptions& options);

}  // namespace apex::planning
