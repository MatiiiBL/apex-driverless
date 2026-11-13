#pragma once

#include <cstddef>
#include <deque>

#include "apex/core/types.hpp"

// Dead reckoning between SLAM updates: the motion the velocity filter integrates, kept as a
// short history so that cone observations stamped in the past can be fused at their own stamp.
namespace apex::estimation {

// A rigid planar motion expressed in the frame of its start pose, with the covariance of
// (dx, dy, dyaw) in that frame, and how long it took (for the yaw-rate bias the SLAM corrects it
// by, ekf_slam.hpp).
struct RelativeMotion {
    Pose2 delta;
    Mat3 covariance = Mat3::Zero();
    double duration = 0.0;  // s
};

// `first` followed by `second` (expressed in the frame where `first` ends), with first-order
// covariance propagation of two independent motions:
//   Sigma = A Sigma1 A^T + B Sigma2 B^T,  A = d(compose)/d(first),  B = d(compose)/d(second).
// Durations add.
[[nodiscard]] RelativeMotion compose(const RelativeMotion& first, const RelativeMotion& second);

// The pose reached from the origin by moving with constant body twist (vx, vy, yaw rate) for dt:
// the exponential map of SE(2), exact for a constant twist.
[[nodiscard]] Pose2 integrate_twist(double vx, double vy, double yaw_rate, double dt);

// One IMU interval of dead reckoning: a constant twist over [start, end] and the spectral
// density of its error (covariance per second of (vx, vy, yaw rate) integrated over time).
struct OdometryIncrement {
    Time start = 0.0;
    Time end = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double yaw_rate = 0.0;
    Mat3 noise_density = Mat3::Zero();

    // The motion over [start, until], until clamped into the interval.
    [[nodiscard]] RelativeMotion motion(Time until) const;
};

// Increments covering a contiguous time span [start(), end()], with the composite of all of them
// cached for the estimate at the newest time.
class OdometryBuffer {
public:
    // Empties the buffer and sets both ends of its span to `time`.
    void reset(Time time);
    // Appends an increment. Its start is moved to end() so the span stays contiguous.
    void push(OdometryIncrement increment);
    // Removes the motion over [start(), t] (splitting an increment that straddles t) and returns
    // it composed. Afterwards start() == t, or end() if t is beyond it.
    RelativeMotion take_until(Time t);

    [[nodiscard]] Time start() const { return start_; }
    [[nodiscard]] Time end() const { return end_; }
    [[nodiscard]] bool empty() const { return increments_.empty(); }
    [[nodiscard]] std::size_t size() const { return increments_.size(); }
    // The composite motion over [start(), end()].
    [[nodiscard]] const RelativeMotion& pending() const { return pending_; }

private:
    std::deque<OdometryIncrement> increments_;
    RelativeMotion pending_;
    Time start_ = 0.0;
    Time end_ = 0.0;
};

}  // namespace apex::estimation
