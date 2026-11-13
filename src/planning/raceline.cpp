#include "apex/planning/raceline.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <Eigen/Cholesky>

#include "apex/core/geometry.hpp"
#include "apex/core/log.hpp"
#include "apex/core/spline.hpp"
#include "apex/core/timing.hpp"

namespace apex::planning {

namespace {

using Eigen::Index;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void check_config(const RacelineConfig& c) {
    const bool ok = c.step > 0.0 && c.safety_margin >= 0.0 && c.iterations >= 1 &&
                    c.max_curvature >= 0.0 && c.regularization >= 0.0 && c.qp_max_iterations > 0 &&
                    c.qp_tolerance > 0.0 && c.path_spacing > 0.0;
    if (!ok) throw std::invalid_argument("optimize_raceline: invalid RacelineConfig");
}

// Points every ~step meters of arc length along the closed spline through `points`.
std::vector<Vec2> resample_closed(std::span<const Vec2> points, double step) {
    std::vector<Vec2> result;
    for (const Spline2D::Sample& sample : Spline2D(points, true).resample(step)) {
        result.push_back(sample.position);
    }
    return result;
}

struct Bounds {
    std::vector<double> lower;
    std::vector<double> upper;
};

// The offsets that keep the car's body `keep_out` meters inside both cone lines. The distance
// to a line is the shortest one, never more than the free space along the normal, so the bounds
// are conservative. Where the track is narrower than twice keep_out the offset is pinned to
// the middle of the two.
Bounds offset_bounds(std::span<const Vec2> reference, const TrackModel& track, double keep_out) {
    Bounds bounds;
    for (const Vec2& r : reference) {
        double lower = keep_out - distance_to_polyline(r, track.right, true);
        double upper = distance_to_polyline(r, track.left, true) - keep_out;
        if (lower > upper) lower = upper = 0.5 * (lower + upper);
        bounds.lower.push_back(lower);
        bounds.upper.push_back(upper);
    }
    return bounds;
}

std::vector<double> distances_to(const Trajectory& path, std::span<const Vec2> line) {
    std::vector<double> result;
    result.reserve(path.size());
    for (const TrajectoryPoint& p : path.points()) {
        result.push_back(distance_to_polyline(p.position, line, true));
    }
    return result;
}

}  // namespace

MinCurvatureQp min_curvature_qp(std::span<const Vec2> reference, std::span<const double> lower,
                                std::span<const double> upper, double max_curvature,
                                double regularization) {
    const std::size_t count = reference.size();
    if (count < 4) throw std::invalid_argument("min_curvature_qp: fewer than 4 points");
    if (lower.size() != count || upper.size() != count) {
        throw std::invalid_argument("min_curvature_qp: bounds do not match the points");
    }
    if (!(max_curvature >= 0.0) || !(regularization >= 0.0)) {
        throw std::invalid_argument("min_curvature_qp: negative curvature bound or weight");
    }
    const auto n = static_cast<Index>(count);
    const auto point = [&](Index i) -> const Vec2& {
        return reference[static_cast<std::size_t>(i)];
    };
    const auto next = [n](Index i) { return (i + 1) % n; };
    const auto prev = [n](Index i) { return (i + n - 1) % n; };

    // Knot spacing and the closed spline's C2 conditions, A p'' = B p, per coordinate.
    VectorXd h(n);
    for (Index i = 0; i < n; ++i) {
        h[i] = (point(next(i)) - point(i)).norm();
        if (!(h[i] > 1e-9)) throw std::invalid_argument("min_curvature_qp: repeated points");
    }
    MatrixXd A = MatrixXd::Zero(n, n);
    MatrixXd B = MatrixXd::Zero(n, n);
    for (Index i = 0; i < n; ++i) {
        A(i, prev(i)) += h[prev(i)];
        A(i, i) += 2.0 * (h[prev(i)] + h[i]);
        A(i, next(i)) += h[i];
        B(i, prev(i)) += 6.0 / h[prev(i)];
        B(i, i) -= 6.0 / h[prev(i)] + 6.0 / h[i];
        B(i, next(i)) += 6.0 / h[i];
    }
    const MatrixXd second = A.llt().solve(B);  // p'' = second * p; A is symmetric and SPD

    VectorXd x(n);
    VectorXd y(n);
    for (Index i = 0; i < n; ++i) {
        x[i] = point(i).x();
        y[i] = point(i).y();
    }
    const VectorXd xpp = second * x;
    const VectorXd ypp = second * y;

    MinCurvatureQp out;
    out.reference.assign(reference.begin(), reference.end());
    out.normals.resize(count);
    out.arc_weights.resize(n);
    out.curvature.resize(n);
    for (Index i = 0; i < n; ++i) {
        // The spline's first derivative at knot i, from the segment that starts there, and the
        // curvature (d x p'') / |d|^3.
        const double dx = (x[next(i)] - x[i]) / h[i] - h[i] * (2.0 * xpp[i] + xpp[next(i)]) / 6.0;
        const double dy = (y[next(i)] - y[i]) / h[i] - h[i] * (2.0 * ypp[i] + ypp[next(i)]) / 6.0;
        const double speed = std::hypot(dx, dy);
        out.curvature[i] = (dx * ypp[i] - dy * xpp[i]) / (speed * speed * speed);
        out.normals[static_cast<std::size_t>(i)] = left_normal(Vec2(dx, dy) / speed);
        out.arc_weights[i] = 0.5 * (h[prev(i)] + h[i]);
    }
    // The first-order change of curvature for offsets alpha(s): alpha'' (the same spline's
    // second derivative, now of the offsets) plus kappa^2 alpha.
    out.jacobian = second;
    out.jacobian.diagonal() += out.curvature.cwiseAbs2();

    const auto W = out.arc_weights.asDiagonal();
    const MatrixXd weighted = W * out.jacobian;  // W M
    QpProblem& qp = out.qp;
    qp.P = 2.0 * out.jacobian.transpose() * weighted;
    qp.P.diagonal() += 2.0 * regularization * out.arc_weights;
    qp.q = 2.0 * weighted.transpose() * out.curvature;
    qp.lower = Eigen::Map<const VectorXd>(lower.data(), n);
    qp.upper = Eigen::Map<const VectorXd>(upper.data(), n);
    if (max_curvature > 0.0) {
        qp.G.resize(2 * n, n);
        qp.G.topRows(n) = out.jacobian;
        qp.G.bottomRows(n) = -out.jacobian;
        qp.h.resize(2 * n);
        qp.h.head(n) = (max_curvature - out.curvature.array()).matrix();
        qp.h.tail(n) = (max_curvature + out.curvature.array()).matrix();
    } else {
        qp.G.resize(0, n);
        qp.h.resize(0);
    }
    return out;
}

Raceline optimize_raceline(const TrackModel& track, const VehicleParams& nominal,
                           const RacelineConfig& config, RacelineTrace* trace) {
    check_config(config);
    if (!track.centerline.closed() || track.left.size() < 3 || track.right.size() < 3) {
        throw std::invalid_argument("optimize_raceline: the track model is not a closed track");
    }
    const Stopwatch stopwatch;
    const double keep_out = 0.5 * nominal.body_width + config.safety_margin;
    const double length = track.centerline.length();
    const auto count =
        std::max<std::size_t>(4, static_cast<std::size_t>(std::round(length / config.step)));
    std::vector<Vec2> reference;
    reference.reserve(count);
    for (std::size_t k = 0; k < count; ++k) {
        const double s = length * static_cast<double>(k) / static_cast<double>(count);
        reference.push_back(track.centerline.at(s).position);
    }

    // Each QP is linearized around the previous solution, resampled to even spacing.
    const QpSettings settings{.max_iterations = config.qp_max_iterations,
                              .tolerance = config.qp_tolerance};
    for (std::size_t iteration = 0; iteration < config.iterations; ++iteration) {
        const Bounds bounds = offset_bounds(reference, track, keep_out);
        MinCurvatureQp problem = min_curvature_qp(reference, bounds.lower, bounds.upper,
                                                  config.max_curvature, config.regularization);
        QpResult result = solve_qp(problem.qp, settings);
        const bool solved = result.status == QpStatus::Solved && result.x.allFinite();
        std::vector<Vec2> moved;
        if (solved) {
            for (std::size_t i = 0; i < problem.reference.size(); ++i) {
                moved.push_back(problem.reference[i] +
                                result.x[static_cast<Index>(i)] * problem.normals[i]);
            }
        } else {
            log(LogLevel::Warn,
                "optimize_raceline: QP {} not solved after {} iterations; keeping the line "
                "before it",
                iteration, result.iterations);
        }
        if (trace != nullptr) {
            trace->problems.push_back(std::move(problem));
            trace->results.push_back(std::move(result));
        }
        if (!solved) break;
        reference = iteration + 1 < config.iterations ? resample_closed(moved, config.step)
                                                      : std::move(moved);
    }

    Raceline raceline;
    raceline.path = Trajectory::from_waypoints(reference, true, config.path_spacing);
    raceline.path.set_widths(distances_to(raceline.path, track.left),
                             distances_to(raceline.path, track.right));
    apply_speed_profile(raceline.path, nominal, config.speed);
    raceline.predicted_lap_time = profile_time(raceline.path);
    raceline.optimize_seconds = stopwatch.elapsed_seconds();
    return raceline;
}

Raceline optimize_raceline(const TrackModel& track, const VehicleParams& nominal,
                           const RacelineConfig& config) {
    return optimize_raceline(track, nominal, config, nullptr);
}

}  // namespace apex::planning
