#include <cmath>
#include <format>
#include <limits>
#include <stdexcept>

#include "apex/runner/recorders.hpp"

namespace apex::runner {

CsvLog::CsvLog(std::ostream& out, double period) : out_(out), period_(period) {
    if (!(period > 0.0)) throw std::invalid_argument("csv log: period must be positive");
}

void CsvLog::on_start(const Track& /*track*/) {
    out_ << "t,state,x,y,yaw,vx,vy,yaw_rate,steer,est_valid,est_x,est_y,est_yaw,est_vx,"
            "cmd_steer,cmd_accel,s,lateral,laps,cones_hit,landmarks,ref_speed,ref_ahead\n";
    next_ = 0.0;
    rows_ = 0;
}

void CsvLog::on_control(const TickView& tick) {
    // A small tolerance so a period of 0.1 s logs at 0.1, 0.2, ... despite rounding in time.
    if (tick.time < next_ - 1e-9) return;
    next_ = (std::floor(tick.time / period_ + 1e-9) + 1.0) * period_;
    const VehicleState& truth = tick.truth;
    const StateEstimate estimate = tick.driver.estimate();
    const ConeMap* map = tick.driver.map();
    // The reference at the estimated position, where the controller tracks it: its speed, and
    // for an open reference (an exploration path) how much of it is left ahead of the car.
    double ref_speed = std::numeric_limits<double>::quiet_NaN();
    double ref_ahead = std::numeric_limits<double>::quiet_NaN();
    const Trajectory* reference = tick.driver.reference();
    if (reference != nullptr && !reference->empty() && estimate.pose.position().allFinite()) {
        const Trajectory::Projection at = reference->project(estimate.pose.position());
        ref_speed = reference->at(at.s).speed;
        if (!reference->closed()) ref_ahead = reference->length() - at.s;
    }
    out_ << std::format(
        "{:.3f},{},{:.4f},{:.4f},{:.5f},{:.4f},{:.4f},{:.5f},{:.5f},{:d},{:.4f},{:.4f},{:.5f},"
        "{:.4f},{:.5f},{:.4f},{:.3f},{:.4f},{},{},{},{:.4f},{:.3f}\n",
        tick.time, stack::to_string(tick.driver.mission_state()), truth.pose.x, truth.pose.y,
        truth.pose.yaw, truth.vx, truth.vy, truth.yaw_rate, truth.steer, estimate.valid,
        estimate.pose.x, estimate.pose.y, estimate.pose.yaw, estimate.vx, tick.command.steer,
        tick.command.accel, tick.race.s(), tick.race.lateral(), tick.race.laps_completed(),
        tick.race.cone_hits().size(), map == nullptr ? 0 : map->cones.size(), ref_speed, ref_ahead);
    ++rows_;
}

}  // namespace apex::runner
