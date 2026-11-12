#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "apex/planning/ggv.hpp"
#include "apex/planning/planning.hpp"

namespace apex::planning {

namespace {

// One integration step, shared by both passes: from speed v at curvature k0, the speed reached
// after `ds` at the envelope's limit `rate(speed, curvature)` (acceleration for the forward
// pass, deceleration for the backward one), capped at `cap1` (curvature k1).
//
// Heun's method on u = v^2, whose slope du/ds is twice the rate, is second-order accurate where
// the rate varies smoothly. Next to the friction ellipse's edge the rate falls steeply with
// speed and Heun's averaged slope can overshoot: the step's constant rate (v1^2 - v^2) / (2 ds)
// then exceeds what either end allows. Such a step is cut back by bisection to the largest
// speed whose step rate one of its ends can sustain.
template <class Rate>
double integrate_step(const Rate& rate, double v, double k0, double k1, double cap1, double ds) {
    const double rate0 = rate(v, k0);
    const double predicted = std::min(std::sqrt(std::max(0.0, v * v + 2.0 * rate0 * ds)), cap1);
    const double u = v * v + (rate0 + rate(predicted, k1)) * ds;
    const double heun = std::min(std::sqrt(std::max(0.0, u)), cap1);
    const auto sustainable = [&](double v1) {
        return (v1 * v1 - v * v) / (2.0 * ds) <= std::max(rate0, rate(v1, k1));
    };
    if (sustainable(heun)) return heun;
    double lo = v;  // a zero rate is always sustainable below the cornering speed
    double hi = heun;
    for (int i = 0; i < 60 && hi - lo > 1e-12 * hi; ++i) {
        const double mid = 0.5 * (lo + hi);
        (sustainable(mid) ? lo : hi) = mid;
    }
    return lo;
}

// The forward pass's step: the speed reached after `ds` of full acceleration from speed v.
double accelerate(const GgvEnvelope& ggv, double v, double k0, double k1, double cap1, double ds) {
    const auto rate = [&ggv](double speed, double k) { return ggv.max_acceleration(speed, k); };
    return integrate_step(rate, v, k0, k1, cap1, ds);
}

// The backward pass's step: the highest speed `ds` before a point with speed v (curvature k1)
// from which full braking reaches v, capped at `cap0` (curvature k0).
double decelerate(const GgvEnvelope& ggv, double v, double k1, double k0, double cap0, double ds) {
    const auto rate = [&ggv](double speed, double k) { return ggv.max_deceleration(speed, k); };
    return integrate_step(rate, v, k1, k0, cap0, ds);
}

// Length of each segment i -> i + 1 (for a closed path, the last one closes the loop).
std::vector<double> segment_lengths(const Trajectory& path) {
    const auto& points = path.points();
    const std::size_t n = points.size();
    const std::size_t segments = path.closed() ? n : n - 1;
    std::vector<double> ds(segments);
    for (std::size_t i = 0; i < segments; ++i) {
        const double end = i + 1 < n ? points[i + 1].s : path.length();
        ds[i] = end - points[i].s;
    }
    return ds;
}

std::size_t argmin(const std::vector<double>& values) {
    return static_cast<std::size_t>(
        std::distance(values.begin(), std::min_element(values.begin(), values.end())));
}

}  // namespace

// Forward-backward integration (e.g. Velenis and Tsiotras, "Minimum-time travel for a vehicle
// with acceleration limits: theoretical analysis and receding-horizon implementation", JOTA,
// 2008): the speed at each point is capped by its steady cornering speed, the forward pass
// limits how fast the car can have accelerated into it, the backward pass how fast it can be
// and still brake for what follows. Every cap is a feasible steady state, so the forward pass
// never has to slow down: starting a closed path's passes at the slowest point makes one lap of
// each exact and periodic.
void apply_speed_profile(Trajectory& path, const VehicleParams& nominal,
                         const SpeedProfileConfig& config, double start_speed, double end_speed) {
    if (path.empty()) throw std::invalid_argument("apply_speed_profile: empty path");
    const GgvEnvelope ggv(nominal, config);
    const auto& points = path.points();
    const std::size_t n = points.size();
    const std::vector<double> ds = segment_lengths(path);

    std::vector<double> cap(n);
    for (std::size_t i = 0; i < n; ++i) cap[i] = ggv.cornering_speed(points[i].curvature);

    std::vector<double> forward(n);
    std::vector<double> speed(n);
    if (path.closed()) {
        const std::size_t first = argmin(cap);
        forward[first] = cap[first];
        for (std::size_t k = 1; k < n; ++k) {
            const std::size_t i = (first + k) % n;
            const std::size_t prev = (i + n - 1) % n;
            forward[i] = accelerate(ggv, forward[prev], points[prev].curvature, points[i].curvature,
                                    cap[i], ds[prev]);
        }
        const std::size_t last = argmin(forward);
        speed[last] = forward[last];
        for (std::size_t k = 1; k < n; ++k) {
            const std::size_t i = (last + n - k) % n;
            const std::size_t next = (i + 1) % n;
            speed[i] = decelerate(ggv, speed[next], points[next].curvature, points[i].curvature,
                                  forward[i], ds[i]);
        }
    } else {
        forward[0] = std::clamp(start_speed, 0.0, cap[0]);
        for (std::size_t i = 1; i < n; ++i) {
            forward[i] = accelerate(ggv, forward[i - 1], points[i - 1].curvature,
                                    points[i].curvature, cap[i], ds[i - 1]);
        }
        speed[n - 1] = std::clamp(end_speed, 0.0, forward[n - 1]);
        for (std::size_t i = n - 1; i-- > 0;) {
            speed[i] = decelerate(ggv, speed[i + 1], points[i + 1].curvature, points[i].curvature,
                                  forward[i], ds[i]);
        }
    }

    // The acceleration that takes each point's speed to the next one's: dv/dt = d(v^2)/ds / 2.
    std::vector<double> accel(n, 0.0);
    for (std::size_t i = 0; i < ds.size(); ++i) {
        const double next = speed[(i + 1) % n];
        accel[i] = (next * next - speed[i] * speed[i]) / (2.0 * ds[i]);
    }
    if (!path.closed()) accel[n - 1] = accel[n - 2];
    path.set_speed_profile(speed, accel);
}

double profile_time(const Trajectory& path) {
    if (path.size() < 2) return 0.0;
    const auto& points = path.points();
    const std::vector<double> ds = segment_lengths(path);
    double time = 0.0;
    for (std::size_t i = 0; i < ds.size(); ++i) {
        // Exact for constant acceleration over the segment.
        const double mean_speed = 0.5 * (points[i].speed + points[(i + 1) % points.size()].speed);
        if (!(mean_speed > 0.0)) return std::numeric_limits<double>::infinity();
        time += ds[i] / mean_speed;
    }
    return time;
}

}  // namespace apex::planning
