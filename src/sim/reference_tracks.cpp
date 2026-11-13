#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "apex/core/angles.hpp"
#include "apex/sim/track_gen.hpp"

#include "track_layout.hpp"

// The reference layouts were drawn by the generator once (with weights favouring their
// character) and frozen here as element lists, so later changes to the generator do not move
// them. Straights keep the generator's bounds and are re-solved for closure.
namespace apex::sim {

namespace {

using detail::Element;
using detail::StraightSpec;
using detail::TurnSpec;

Element turn(double degrees, double radius) {
    return TurnSpec{deg_to_rad(degrees), radius, false};
}
Element start_after(double length) {  // the start straight after the timekeeping line
    return StraightSpec{length, true, 8.0, 30.0};
}
Element start_before(double length) {  // and before it, where the car is staged
    return StraightSpec{length, true, 16.0, 45.0};
}
Element straight(double length) {
    return StraightSpec{length, true, 2.0, 60.0};
}
Element gap(double length) {  // inside a chicane: fixed
    return StraightSpec{length, false};
}

// The angles above are rounded to 0.1 degree; the last turn absorbs the rounding so the heading
// closes exactly at +/-360 degrees.
void close_heading_exactly(std::vector<Element>& elements) {
    double total = 0.0;
    TurnSpec* last = nullptr;
    for (Element& element : elements) {
        if (auto* t = std::get_if<TurnSpec>(&element)) {
            total += t->angle;
            last = t;
        }
    }
    last->angle += std::copysign(kTwoPi, total) - total;
}

// 315 m, clockwise: three hairpins, two chicanes, nothing longer than a 30 m straight.
std::vector<Element> technical() {
    return {start_after(16.4), turn(72.0, 19.8),  straight(15.3),    turn(-165.2, 6.7),
            straight(25.5),    turn(-53.5, 10.6), gap(4.0),          turn(50.9, 8.8),
            straight(28.8),    turn(-49.4, 10.9), gap(3.8),          turn(44.5, 11.4),
            straight(5.0),     turn(-154.3, 6.6), straight(8.6),     turn(74.5, 12.8),
            straight(9.9),     turn(-179.2, 8.3), start_before(31.1)};
}

// 474 m, clockwise: sweepers of 20-25 m radius between long straights, one hairpin.
std::vector<Element> flowing() {
    return {start_after(19.1), turn(-50.7, 23.2), straight(6.6),     turn(-47.0, 24.9),
            straight(21.5),    turn(-65.6, 20.2), straight(6.3),     turn(-50.4, 16.0),
            straight(25.4),    turn(32.0, 12.8),  gap(3.4),          turn(-38.4, 12.2),
            straight(18.3),    turn(-153.0, 7.1), straight(23.0),    turn(31.2, 23.5),
            straight(25.5),    turn(-34.2, 19.2), straight(14.9),    turn(-47.7, 9.1),
            straight(10.3),    turn(67.1, 23.9),  straight(22.8),    turn(43.2, 23.8),
            straight(8.2),     turn(-46.5, 23.8), start_before(24.2)};
}

// 415 m, counter-clockwise: two hairpins, a chicane, three sweepers and medium corners.
std::vector<Element> mixed() {
    return {start_after(13.0), turn(52.0, 11.1),  straight(29.1),    turn(-68.8, 9.1),
            straight(27.9),    turn(156.8, 6.9),  straight(9.0),     turn(41.8, 21.7),
            straight(17.7),    turn(32.0, 12.1),  gap(3.9),          turn(-31.2, 10.0),
            straight(26.2),    turn(68.5, 8.2),   straight(28.4),    turn(50.0, 23.1),
            straight(26.5),    turn(-31.9, 14.3), straight(19.4),    turn(153.9, 7.1),
            straight(21.4),    turn(-63.1, 16.9), start_before(27.5)};
}

}  // namespace

std::vector<std::string> reference_track_names() {
    return {"technical", "flowing", "mixed"};
}

Track reference_track(std::string_view name, const TrackBuildOptions& build) {
    std::vector<Element> elements;
    if (name == "technical") {
        elements = technical();
    } else if (name == "flowing") {
        elements = flowing();
    } else if (name == "mixed") {
        elements = mixed();
    } else {
        throw std::invalid_argument("reference_track: unknown layout '" + std::string(name) + "'");
    }
    close_heading_exactly(elements);
    detail::Layout layout(elements);
    if (!layout.close()) throw std::logic_error("reference_track: the layout does not close");
    std::vector<Vec2> waypoints;
    for (const detail::ProfileSample& sample :
         layout.sample(detail::kWaypointSpacing, build.half_width)) {
        waypoints.push_back(sample.position);
    }
    return track_from_centerline(waypoints, build, std::string(name));
}

}  // namespace apex::sim
