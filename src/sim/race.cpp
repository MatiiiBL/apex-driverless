#include "apex/sim/race.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "apex/core/geometry.hpp"

namespace apex::sim {

namespace {

constexpr double kConePenalty = 2.0;        // s per cone knocked
constexpr double kOffCoursePenalty = 10.0;  // s per excursion
// The car moves 3 cm per tick; projections search this far along the centerline around the
// previous position, which also keeps them from jumping to a nearby part of the track.
constexpr double kSearchWindow = 10.0;  // m

}  // namespace

RaceMonitor::RaceMonitor(const Track& track, const VehicleParams& vehicle)
    : track_(&track), vehicle_(vehicle), knocked_(track.cones.size(), false) {
    if (track.centerline.size() < 3 || !track.centerline.closed()) {
        throw std::invalid_argument("RaceMonitor: the track needs a closed centerline");
    }
    const Vec2 across = track.timing_line_left - track.timing_line_right;
    if (!(across.norm() > 0.0)) {
        throw std::invalid_argument("RaceMonitor: the timekeeping line has no length");
    }
    line_center_ = 0.5 * (track.timing_line_left + track.timing_line_right);
    line_along_ = across.normalized();
    line_forward_ = Vec2(line_along_.y(), -line_along_.x());  // left to right turned clockwise
    line_half_length_ = 0.5 * across.norm() + 0.5 * vehicle.body_width;
}

void RaceMonitor::update(const VehicleState& state, Time time) {
    const Trajectory& center = track_->centerline;
    const Vec2 position = state.pose.position();
    const Trajectory::Projection projection =
        has_previous_ ? center.project(position, s_, kSearchWindow) : center.project(position);
    progress_ = has_previous_ ? progress_ + center.delta_s(s_, projection.s)
                              : center.delta_s(0.0, projection.s);
    s_ = projection.s;
    lateral_ = projection.lateral;
    if (has_previous_) check_timing_line(position, time);
    check_cones(state, time);
    check_off_course(state, time);
    previous_position_ = position;
    previous_time_ = time;
    has_previous_ = true;
}

void RaceMonitor::check_timing_line(const Vec2& position, Time time) {
    // Signed distances past the line; a crossing is a sign change, counted on the half-open
    // convention (before < 0 <= after) so a position exactly on the line is never counted twice.
    const double before = (previous_position_ - line_center_).dot(line_forward_);
    const double after = (position - line_center_).dot(line_forward_);
    const bool forward = before < 0.0 && after >= 0.0;
    const bool backward = before >= 0.0 && after < 0.0;
    if (!forward && !backward) return;
    const double fraction = before / (before - after);
    const Vec2 crossing = previous_position_ + fraction * (position - previous_position_);
    if (std::abs((crossing - line_center_).dot(line_along_)) > line_half_length_) return;
    if (backward) {
        --net_crossings_;
        return;
    }
    ++net_crossings_;
    if (net_crossings_ <= best_crossings_) return;
    best_crossings_ = net_crossings_;
    const Time crossed = previous_time_ + fraction * (time - previous_time_);
    if (started_) {
        laps_.push_back({static_cast<int>(laps_.size()) + 1, lap_start_, crossed});
    }
    started_ = true;
    lap_start_ = crossed;
}

void RaceMonitor::check_cones(const VehicleState& state, Time time) {
    const double half_width = 0.5 * vehicle_.body_width;
    const double max_radius = cone_shape(ConeColor::BigOrange).base_radius;
    const double reach =
        std::hypot(std::max(vehicle_.body_front, vehicle_.body_rear), half_width) + max_radius;
    const Vec2 position = state.pose.position();
    for (std::size_t i = 0; i < track_->cones.size(); ++i) {
        const Cone& cone = track_->cones[i];
        if (knocked_[i] || (cone.position - position).squaredNorm() > reach * reach) continue;
        // Distance from the cone's center to the footprint rectangle, in the body frame.
        const Vec2 local = state.pose.to_local(cone.position);
        const double dx =
            std::max({0.0, local.x() - vehicle_.body_front, -vehicle_.body_rear - local.x()});
        const double dy = std::max(0.0, std::abs(local.y()) - half_width);
        const double radius = cone_shape(cone.color).base_radius;
        if (dx * dx + dy * dy <= radius * radius) {
            knocked_[i] = true;
            cone_hits_.push_back({time, cone.id});
        }
    }
}

void RaceMonitor::check_off_course(const VehicleState& state, Time time) {
    const Trajectory& center = track_->centerline;
    auto inside = [&center](double s, double lateral) {
        const TrajectoryPoint at = center.at(s);
        return lateral <= at.width_left && lateral >= -at.width_right;
    };
    if (outside_) {
        if (inside(s_, lateral_)) outside_ = false;
        return;
    }
    const double half_track = 0.5 * vehicle_.track_width;
    for (const Vec2& wheel : {Vec2(vehicle_.lf, half_track), Vec2(vehicle_.lf, -half_track),
                              Vec2(-vehicle_.lr, half_track), Vec2(-vehicle_.lr, -half_track)}) {
        const Trajectory::Projection p =
            center.project(state.pose.to_parent(wheel), s_, kSearchWindow);
        if (inside(p.s, p.lateral)) return;
    }
    outside_ = true;
    off_course_.push_back({time, s_});
}

int RaceMonitor::laps_completed() const {
    return static_cast<int>(laps_.size());
}
const std::vector<LapRecord>& RaceMonitor::laps() const {
    return laps_;
}
const std::vector<ConeHit>& RaceMonitor::cone_hits() const {
    return cone_hits_;
}
const std::vector<OffCourse>& RaceMonitor::off_course() const {
    return off_course_;
}
double RaceMonitor::penalty_seconds() const {
    return kConePenalty * static_cast<double>(cone_hits_.size()) +
           kOffCoursePenalty * static_cast<double>(off_course_.size());
}
bool RaceMonitor::started() const {
    return started_;
}
double RaceMonitor::s() const {
    return s_;
}
double RaceMonitor::lateral() const {
    return lateral_;
}
double RaceMonitor::progress() const {
    return progress_;
}

}  // namespace apex::sim
