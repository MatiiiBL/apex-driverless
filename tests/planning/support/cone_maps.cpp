#include "cone_maps.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "apex/core/angles.hpp"
#include "apex/core/geometry.hpp"

namespace apex::planning::test_support {

namespace {

// A color the camera might have voted for instead: one of the other small-cone colors.
ConeColor wrong_color(ConeColor truth, Rng& rng) {
    std::array<ConeColor, 3> others{};
    std::size_t count = 0;
    for (const ConeColor c : {ConeColor::Blue, ConeColor::Yellow, ConeColor::Orange}) {
        if (c != truth) others[count++] = c;
    }
    return others[rng.index(count)];
}

// A point `offset` meters beyond the cone line on `side` (+1 left, -1 right) at arc length s.
Vec2 beyond_cone_line(const Trajectory& center, double s, double side, double offset) {
    const TrajectoryPoint p = center.at(s);
    const double width = side > 0.0 ? p.width_left : p.width_right;
    return p.position + side * (width + offset) * left_normal(heading_vector(p.yaw));
}

}  // namespace

MapErrors worst_case_errors() {
    MapErrors errors;
    errors.position_rms = 0.15;
    errors.miss_probability = 0.20;
    errors.wrong_color_probability = 0.10;
    errors.unknown_color_probability = 0.50;
    errors.false_positives_per_100m = 4.0;
    return errors;
}

ConeMap make_cone_map(const Track& track, const MapErrors& errors, Rng& rng) {
    ConeMap map;
    const double sigma = errors.position_rms / std::sqrt(2.0);  // per axis
    const double variance = std::max(sigma * sigma, 1e-6);
    // A landmark with `color` winning `share` of its observations' votes and `other` the rest.
    const auto add = [&](const Vec2& position, ConeColor color, ConeColor other, double share,
                         std::int32_t truth_id) {
        MapCone cone;
        cone.id = static_cast<std::int32_t>(map.cones.size());
        cone.position = position;
        cone.covariance = variance * Mat2::Identity();
        cone.color = color;
        cone.observations = 4 + static_cast<std::uint32_t>(rng.index(12));
        if (color != ConeColor::Unknown) {
            const auto won = static_cast<std::uint32_t>(
                std::ceil(share * static_cast<double>(cone.observations)));
            cone.color_votes[index_of(color)] = won;
            cone.color_votes[index_of(other)] += cone.observations - won;
        }
        cone.truth_id = truth_id;
        map.cones.push_back(cone);
    };

    for (const Cone& cone : track.cones) {
        if (rng.bernoulli(errors.miss_probability)) continue;
        const Vec2 noise(rng.normal(0.0, sigma), rng.normal(0.0, sigma));
        const double draw = rng.uniform();
        if (draw < errors.wrong_color_probability) {
            add(cone.position + noise, wrong_color(cone.color, rng), cone.color, 2.0 / 3.0,
                cone.id);
        } else if (draw < errors.wrong_color_probability + errors.unknown_color_probability) {
            add(cone.position + noise, ConeColor::Unknown, ConeColor::Unknown, 0.0, cone.id);
        } else {
            add(cone.position + noise, cone.color, cone.color, 1.0, cone.id);
        }
    }

    // Clutter, drawn per 100 m (the Poisson sampler is meant for small rates). A point that
    // lands near another part of the track is drawn again: it would be inside the track there.
    const Trajectory& center = track.centerline;
    const double length = center.length();
    for (double block = 0.0; block < length; block += 100.0) {
        const double block_length = std::min(100.0, length - block);
        const std::uint32_t count =
            rng.poisson(errors.false_positives_per_100m * block_length / 100.0);
        for (std::uint32_t k = 0; k < count; ++k) {
            for (int attempt = 0; attempt < 20; ++attempt) {
                const double s = block + rng.uniform(0.0, block_length);
                const double side = rng.bernoulli(0.5) ? 1.0 : -1.0;
                const double offset =
                    rng.uniform(errors.false_positive_min_offset, errors.false_positive_max_offset);
                const Vec2 p = beyond_cone_line(center, s, side, offset);
                const Trajectory::Projection proj = center.project(p);
                const TrajectoryPoint nearest = center.at(proj.s);
                const double clearance =
                    proj.distance - std::max(nearest.width_left, nearest.width_right);
                if (clearance < 0.9 * errors.false_positive_min_offset) continue;
                const ConeColor color = rng.bernoulli(0.7)   ? ConeColor::Unknown
                                        : rng.bernoulli(0.5) ? ConeColor::Blue
                                                             : ConeColor::Yellow;
                add(p, color, color, 1.0, -1);
                break;
            }
        }
    }
    map.revision = 1;
    return map;
}

Sightings::Sightings(ConeMap full, double range)
    : full_(std::move(full)), range_(range), seen_(full_.cones.size(), false) {}

void Sightings::observe(const Pose2& pose) {
    for (std::size_t i = 0; i < full_.cones.size(); ++i) {
        if (seen_[i]) continue;
        const Vec2 local = pose.to_local(full_.cones[i].position);
        seen_[i] = local.x() >= 0.0 && local.norm() <= range_;
    }
}

ConeMap Sightings::map() const {
    ConeMap map;
    for (std::size_t i = 0; i < full_.cones.size(); ++i) {
        if (seen_[i]) map.cones.push_back(full_.cones[i]);
    }
    map.revision = full_.revision;
    return map;
}

double staging_s(const Track& track) {
    return track.centerline.wrap_s(track.centerline.project(track.start_pose.position()).s);
}

Pose2 centerline_pose(const Track& track, double s, double lateral, double yaw_error) {
    const TrajectoryPoint p = track.centerline.at(track.centerline.wrap_s(s));
    const Vec2 position = p.position + lateral * left_normal(heading_vector(p.yaw));
    return {position.x(), position.y(), wrap_angle(p.yaw + yaw_error)};
}

std::vector<Pose2> driven_lap(const Track& track, double wander, Rng& rng, double step,
                              double overrun) {
    // Three sinusoids of random wavelength and phase, faded in over the first 10 m so the drive
    // starts at the staging pose.
    struct Wave {
        double amplitude;
        double wavenumber;
        double phase;
    };
    std::array<Wave, 3> waves{};
    for (Wave& wave : waves) {
        wave.amplitude = wander / 3.0;
        wave.wavenumber = kTwoPi / rng.uniform(25.0, 80.0);
        wave.phase = rng.uniform(0.0, kTwoPi);
    }
    const double length = track.centerline.length();
    const double start = staging_s(track);
    const double end = 2.0 * length + overrun;  // through the timing line twice
    std::vector<Pose2> poses;
    for (double u = start; u <= end; u += step) {
        const double fade = std::min(1.0, (u - start) / 10.0);
        double lateral = 0.0;
        double slope = 0.0;
        for (const Wave& wave : waves) {
            lateral += wave.amplitude * std::sin(wave.wavenumber * u + wave.phase);
            slope += wave.amplitude * wave.wavenumber * std::cos(wave.wavenumber * u + wave.phase);
        }
        poses.push_back(centerline_pose(track, u, fade * lateral, std::atan(fade * slope)));
    }
    return poses;
}

}  // namespace apex::planning::test_support
