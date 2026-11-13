#include "track_layout.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

#include <Eigen/Dense>

#include "apex/core/geometry.hpp"

namespace apex::sim::detail {

namespace {

constexpr double kChunk = 1.0;  // m, longest quadrature interval

// 5-point Gauss-Legendre nodes and weights on [-1, 1].
constexpr std::array<double, 5> kNodes = {-0.9061798459386640, -0.5384693101056831, 0.0,
                                          0.5384693101056831, 0.9061798459386640};
constexpr std::array<double, 5> kWeights = {0.2369268850561891, 0.4786286704993665,
                                            0.5688888888888889, 0.4786286704993665,
                                            0.2369268850561891};

double curvature_at(const Piece& piece, double u) {
    return piece.k0 + (piece.k1 - piece.k0) * u / piece.length;
}

// Heading at distance u into a piece that starts at heading theta0: the integral of the linear
// curvature.
double heading_at(const Piece& piece, double theta0, double u) {
    return theta0 + piece.k0 * u + 0.5 * (piece.k1 - piece.k0) * u * u / piece.length;
}

// Displacement from u0 to u1 along a piece that starts at heading theta0.
Vec2 displacement(const Piece& piece, double theta0, double u0, double u1) {
    if (piece.k0 == 0.0 && piece.k1 == 0.0) return (u1 - u0) * heading_vector(theta0);
    const double span = u1 - u0;
    const int chunks = std::max(1, static_cast<int>(std::ceil(span / kChunk)));
    const double h = span / chunks;
    Vec2 sum = Vec2::Zero();
    for (int c = 0; c < chunks; ++c) {
        const double mid = u0 + (c + 0.5) * h;
        for (std::size_t q = 0; q < kNodes.size(); ++q) {
            const double u = mid + 0.5 * h * kNodes[q];
            sum += kWeights[q] * heading_vector(heading_at(piece, theta0, u));
        }
    }
    return 0.5 * h * sum;
}

}  // namespace

double transition_length(double angle, double radius) {
    return std::min(0.5 * radius, 0.3 * std::abs(angle) * radius);
}

Layout::Layout(const std::vector<Element>& elements) {
    for (const Element& element : elements) {
        if (const auto* turn = std::get_if<TurnSpec>(&element)) {
            if (!(turn->radius > 0.0) || turn->angle == 0.0) {
                throw std::invalid_argument("Layout: a turn needs a radius and an angle");
            }
            const double k = std::copysign(1.0 / turn->radius, turn->angle);
            const double arc = std::abs(turn->angle) * turn->radius;
            const double transition = transition_length(turn->angle, turn->radius);
            // Each clothoid turns k * transition / 2; the arc takes the rest.
            pieces_.push_back({transition, 0.0, k, -1});
            if (arc > transition) pieces_.push_back({arc - transition, k, k, -1});
            pieces_.push_back({transition, k, 0.0, -1});
        } else {
            const auto& straight = std::get<StraightSpec>(element);
            if (!(straight.length > 0.0)) {
                throw std::invalid_argument("Layout: a straight needs a positive length");
            }
            int index = -1;
            if (straight.adjustable) {
                index = static_cast<int>(bounds_.size());
                bounds_.push_back({straight.min_length, straight.max_length});
            }
            pieces_.push_back({straight.length, 0.0, 0.0, index});
        }
    }
    if (pieces_.empty()) throw std::invalid_argument("Layout: no elements");
}

double Layout::length() const {
    double total = 0.0;
    for (const Piece& piece : pieces_) total += piece.length;
    return total;
}

double Layout::total_turning() const {
    double total = 0.0;
    for (const Piece& piece : pieces_) total += 0.5 * (piece.k0 + piece.k1) * piece.length;
    return total;
}

std::vector<double> Layout::start_headings() const {
    std::vector<double> headings;
    headings.reserve(pieces_.size());
    double theta = 0.0;
    for (const Piece& piece : pieces_) {
        headings.push_back(theta);
        theta = heading_at(piece, theta, piece.length);
    }
    return headings;
}

Vec2 Layout::end_point() const {
    const std::vector<double> headings = start_headings();
    Vec2 end = Vec2::Zero();
    for (std::size_t i = 0; i < pieces_.size(); ++i) {
        end += displacement(pieces_[i], headings[i], 0.0, pieces_[i].length);
    }
    return end;
}

bool Layout::close() {
    // Lengthening adjustable straight k by delta_k moves the end point by delta_k t_k, with t_k
    // its direction, so closing is: minimize |delta|^2 / 2 subject to sum_k delta_k t_k = b
    // (b = -end) and lo_k <= delta_k <= hi_k. Its Lagrangian dual has just two variables: for a
    // multiplier lambda, the best delta_k is t_k . lambda clamped to its box, and the dual
    // gradient is the remaining closure error b - sum_k delta_k t_k. The dual is concave and
    // piecewise quadratic, so a semi-smooth Newton ascent (Hessian from the unclamped straights)
    // with backtracking converges in a few steps when a solution exists; when none does, the
    // error stalls above zero. See Boyd & Vandenberghe, "Convex Optimization", Cambridge
    // University Press, 2004, chapter 5.
    const std::vector<double> headings = start_headings();
    const std::size_t count = bounds_.size();
    std::vector<Vec2> directions(count, Vec2::Zero());
    std::vector<double> lo(count, 0.0);
    std::vector<double> hi(count, 0.0);
    for (std::size_t i = 0; i < pieces_.size(); ++i) {
        if (pieces_[i].adjustable < 0) continue;
        const auto k = static_cast<std::size_t>(pieces_[i].adjustable);
        directions[k] = heading_vector(headings[i]);
        lo[k] = bounds_[k].min_length - pieces_[i].length;
        hi[k] = bounds_[k].max_length - pieces_[i].length;
        if (lo[k] > 0.0 || hi[k] < 0.0) return false;  // starts outside its own bounds
    }
    const Vec2 b = -end_point();
    std::vector<double> delta(count, 0.0);
    auto evaluate = [&](const Vec2& lambda, double& dual) {
        Vec2 residual = b;
        dual = lambda.dot(b);
        for (std::size_t k = 0; k < count; ++k) {
            delta[k] = std::clamp(directions[k].dot(lambda), lo[k], hi[k]);
            residual -= delta[k] * directions[k];
            dual += 0.5 * delta[k] * delta[k] - delta[k] * directions[k].dot(lambda);
        }
        return residual;
    };
    Vec2 lambda = Vec2::Zero();
    double dual = 0.0;
    Vec2 residual = evaluate(lambda, dual);
    constexpr double kTolerance = 1e-9;
    // With few unclamped straights the Hessian is near singular; the small ridge keeps the step
    // finite and the backtracking finds how far it can go.
    constexpr double kRidge = 1e-6;
    constexpr double kMinStep = 1e-14;
    for (int iteration = 0; iteration < 100 && residual.norm() > kTolerance; ++iteration) {
        Mat2 hessian = kRidge * Mat2::Identity();
        for (std::size_t k = 0; k < count; ++k) {
            const double projection = directions[k].dot(lambda);
            if (projection > lo[k] && projection < hi[k]) {
                hessian += directions[k] * directions[k].transpose();
            }
        }
        const Vec2 step = hessian.ldlt().solve(residual);
        double trial_dual = 0.0;
        Vec2 trial = residual;
        double alpha = 1.0;
        for (; alpha > kMinStep; alpha *= 0.5) {
            trial = evaluate(lambda + alpha * step, trial_dual);
            if (trial_dual > dual) break;
        }
        if (alpha <= kMinStep) break;
        lambda += alpha * step;
        dual = trial_dual;
        residual = trial;
    }
    residual = evaluate(lambda, dual);
    if (residual.norm() > 1e-6) return false;
    for (Piece& piece : pieces_) {
        if (piece.adjustable >= 0)
            piece.length += delta[static_cast<std::size_t>(piece.adjustable)];
    }
    return end_point().norm() < 1e-6;
}

std::vector<ProfileSample> Layout::sample(double spacing, double half_width) const {
    const double total = length();
    const auto count =
        std::max<std::size_t>(3, static_cast<std::size_t>(std::round(total / spacing)));
    const double step = total / static_cast<double>(count);
    const std::vector<double> headings = start_headings();
    std::vector<ProfileSample> samples;
    samples.reserve(count);
    std::size_t piece = 0;
    double piece_start = 0.0;  // s at the start of the current piece
    double u = 0.0;            // position within it
    Vec2 position = Vec2::Zero();
    for (std::size_t i = 0; i < count; ++i) {
        const double s = step * static_cast<double>(i);
        // Walk to s, integrating over whole pieces and then the part of the last one.
        while (piece + 1 < pieces_.size() && s >= piece_start + pieces_[piece].length) {
            position += displacement(pieces_[piece], headings[piece], u, pieces_[piece].length);
            piece_start += pieces_[piece].length;
            ++piece;
            u = 0.0;
        }
        const double target = std::min(s - piece_start, pieces_[piece].length);
        position += displacement(pieces_[piece], headings[piece], u, target);
        u = target;
        samples.push_back({s, position, curvature_at(pieces_[piece], u), half_width});
    }
    return samples;
}

}  // namespace apex::sim::detail
