#include "apex/estimation/eval/synthetic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "apex/core/rng.hpp"
#include "apex/core/sensor_rig.hpp"

namespace apex::estimation::eval {

namespace {

constexpr double kTick = 1e-3;
constexpr std::int64_t kImuTicks = 5;
constexpr std::int64_t kWheelTicks = 10;
constexpr std::int64_t kScanTicks = 100;

// The magic formula's force magnitude per unit capacity (mu Fz) at slip angle magnitude alpha.
double force_ratio(const TireParams& tire, double alpha) {
    const double b = tire.B * alpha;
    return std::sin(tire.C * std::atan(b - tire.E * (b - std::atan(b))));
}

// The inverse of the forward formula on its rising branch, by search and bisection: written
// independently of the estimator's analytic inverse, so the drive checks it rather than
// repeats it.
class TireInverse {
public:
    explicit TireInverse(const TireParams& tire) : tire_(tire) {
        for (double alpha = 0.0; alpha <= 0.6; alpha += 1e-4) {
            const double ratio = force_ratio(tire, alpha);
            if (ratio > peak_ratio_) {
                peak_ratio_ = ratio;
                peak_ = alpha;
            }
        }
    }

    // Signed slip angle (Fy = -mu Fz sin(...) convention) for lateral force fy under load fz.
    [[nodiscard]] double slip_angle(double fz, double fy) const {
        const double u = std::abs(fy) / (tire_.mu * fz);
        double alpha = peak_;
        if (u < peak_ratio_) {
            double lo = 0.0;
            double hi = peak_;
            while (hi - lo > 1e-10) {  // rad: far below any effect on the drive
                const double mid = 0.5 * (lo + hi);
                (force_ratio(tire_, mid) < u ? lo : hi) = mid;
            }
            alpha = 0.5 * (lo + hi);
        }
        return fy > 0.0 ? -alpha : alpha;
    }

private:
    TireParams tire_;
    double peak_ = 0.0;
    double peak_ratio_ = 0.0;
};

struct Loads {
    double front;
    double rear;
};

Loads normal_loads(const VehicleParams& v, double speed, double ax) {
    const double l = v.wheelbase();
    const double downforce = v.downforce(speed);
    const double transfer = v.mass * ax * v.cg_height / l;
    return {std::max(0.05 * v.mass * kGravity,
                     v.mass * kGravity * v.lr / l + downforce * v.aero_balance_front - transfer),
            std::max(0.05 * v.mass * kGravity, v.mass * kGravity * v.lf / l +
                                                   downforce * (1.0 - v.aero_balance_front) +
                                                   transfer)};
}

// Speed limits along the unrolled distance, then the forward-backward passes of a point mass
// with constant acceleration and braking limits.
struct SpeedProfile {
    double step = 0.05;
    std::vector<double> speed;
    std::vector<double> time;
};

SpeedProfile speed_profile(const Trajectory& center, double s_start, double staging, double total,
                           const DriveProfile& profile) {
    SpeedProfile p;
    const auto n = static_cast<std::size_t>(std::ceil(total / p.step)) + 1;
    p.speed.resize(n);
    const double length = center.length();
    for (std::size_t i = 0; i < n; ++i) {
        const double d = std::min(total, static_cast<double>(i) * p.step);
        const bool exploring = d < staging + length;
        const double v_max = exploring ? profile.exploration_speed : profile.racing_speed;
        const double a_lat =
            exploring ? profile.exploration_lateral_accel : profile.racing_lateral_accel;
        const double kappa = std::abs(center.at(center.wrap_s(s_start + d)).curvature);
        p.speed[i] = kappa > 1e-9 ? std::min(v_max, std::sqrt(a_lat / kappa)) : v_max;
    }
    p.speed[0] = 0.0;
    for (std::size_t i = 1; i < n; ++i) {
        p.speed[i] = std::min(p.speed[i], std::sqrt(p.speed[i - 1] * p.speed[i - 1] +
                                                    2.0 * profile.max_accel * p.step));
    }
    for (std::size_t i = n - 1; i-- > 0;) {
        p.speed[i] = std::min(p.speed[i], std::sqrt(p.speed[i + 1] * p.speed[i + 1] +
                                                    2.0 * profile.max_brake * p.step));
    }
    p.time.resize(n);
    p.time[0] = profile.hold_at_start;
    for (std::size_t i = 1; i < n; ++i) {
        p.time[i] = p.time[i - 1] + 2.0 * p.step / (p.speed[i - 1] + p.speed[i]);
    }
    return p;
}

// One tick of the kinematic truth before differentiation.
struct Kinematics {
    double distance = 0.0;
    double speed = 0.0;
    double course = 0.0;  // direction of travel
    double curvature = 0.0;
    double sideslip = 0.0;
    double yaw = 0.0;
    Vec2 position = Vec2::Zero();
};

}  // namespace

ConeObservations observe_cones(const Track& track, const Pose2& pose, Time stamp,
                               const SyntheticSensors& sensors, Rng& rng) {
    ConeObservations out;
    out.stamp = stamp;
    const Vec2 mount(LidarGeometry{}.mount.x(), LidarGeometry{}.mount.y());
    auto noisy = [&](const Vec2& body, std::int32_t truth_id, ConeColor true_color) {
        const Vec2 ray = body - mount;
        const double range = ray.norm();
        const double bearing = std::atan2(ray.y(), ray.x());
        const double sr = sensors.range_noise + sensors.range_noise_per_m * range;
        const double st = sensors.tangential_noise + sensors.bearing_noise * range;
        const double c = std::cos(bearing);
        const double s = std::sin(bearing);
        Mat2 rot;
        rot << c, -s, s, c;
        ConeObservation obs;
        obs.position = body + rot * Vec2(sr * rng.normal(), st * rng.normal());
        obs.covariance = sensors.reported_covariance_scale * rot *
                         Vec2(sr * sr, st * st).asDiagonal() * rot.transpose();
        obs.truth_id = truth_id;
        const double camera_range = body.norm();
        const double camera_bearing = std::atan2(body.y(), body.x());
        if (camera_range <= sensors.camera_range &&
            std::abs(camera_bearing) <= 0.5 * sensors.camera_fov) {
            const double f = camera_range / sensors.camera_range;
            const double p_error =
                sensors.color_error_near + (sensors.color_error_far - sensors.color_error_near) * f;
            obs.color = true_color;
            if (true_color == ConeColor::Unknown || rng.bernoulli(p_error)) {
                obs.color = rng.bernoulli(0.5) ? ConeColor::Blue : ConeColor::Yellow;
                if (obs.color == true_color) obs.color = ConeColor::Orange;
            }
            obs.color_confidence = 0.95 - 0.5 * f;
        }
        out.cones.push_back(obs);
    };
    for (const Cone& cone : track.cones) {
        const Vec2 body = pose.to_local(cone.position);
        const Vec2 ray = body - mount;
        const double range = ray.norm();
        if (range > sensors.cone_range || range < 0.5) continue;
        if (std::abs(std::atan2(ray.y(), ray.x())) > 0.5 * sensors.cone_fov) continue;
        const double f = range / sensors.cone_range;
        const double p_detect =
            sensors.detection_near + (sensors.detection_far - sensors.detection_near) * f * f;
        if (!rng.bernoulli(p_detect)) continue;
        noisy(body, cone.id, cone.color);
    }
    const std::uint32_t false_positives = rng.poisson(sensors.false_positive_rate);
    for (std::uint32_t i = 0; i < false_positives; ++i) {
        const double range = rng.uniform(2.0, sensors.cone_range);
        const double bearing = rng.uniform(-0.5 * sensors.cone_fov, 0.5 * sensors.cone_fov);
        noisy(mount + range * Vec2(std::cos(bearing), std::sin(bearing)), -1, ConeColor::Unknown);
    }
    return out;
}

void note_detections(const ConeObservations& scan, std::vector<std::uint32_t>& detections,
                     std::vector<Time>& mappable_since) {
    for (const ConeObservation& obs : scan.cones) {
        if (obs.truth_id < 0) continue;
        const auto id = static_cast<std::size_t>(obs.truth_id);
        if (++detections[id] == 3) mappable_since[id] = scan.stamp;
    }
}

const TruthSample& SyntheticRun::truth_at(Time t) const {
    const auto k = static_cast<std::int64_t>(std::llround(t / kTick));
    const auto i = std::clamp<std::int64_t>(k, 0, static_cast<std::int64_t>(truth.size()) - 1);
    return truth[static_cast<std::size_t>(i)];
}

SyntheticRun make_synthetic_run(const Track& track, const VehicleParams& vehicle,
                                const DriveProfile& profile, const SyntheticSensors& sensors,
                                std::uint64_t seed) {
    if (profile.laps < 1 || !(profile.exploration_speed > 0.0) || !(profile.racing_speed > 0.0) ||
        !(profile.max_accel > 0.0) || !(profile.max_brake > 0.0) ||
        !(profile.exploration_lateral_accel > 0.0) || !(profile.racing_lateral_accel > 0.0) ||
        profile.hold_at_start < 0.0) {
        throw std::invalid_argument("make_synthetic_run: invalid drive profile");
    }
    const Trajectory& center = track.centerline;
    const double length = center.length();
    const double s_start = center.project(track.start_pose.position()).s;
    const double staging = center.wrap_s(length - s_start);
    const double total = staging + profile.laps * length;
    const SpeedProfile sp = speed_profile(center, s_start, staging, total, profile);
    const double l = vehicle.wheelbase();
    const TireInverse front_tire(vehicle.front_tire);
    const TireInverse rear_tire(vehicle.rear_tire);

    // Kinematic truth every tick: the course follows the centerline's heading, the body is
    // turned from it by the sideslip a rear axle at its steady-state slip angle produces.
    std::vector<Kinematics> kin;
    const double end_time = sp.time.back();
    const auto ticks = static_cast<std::size_t>(std::floor(end_time / kTick)) + 1;
    kin.reserve(ticks);
    std::size_t seg = 0;
    for (std::size_t k = 0; k < ticks; ++k) {
        const double t = static_cast<double>(k) * kTick;
        Kinematics q;
        if (t > sp.time[0]) {
            while (seg + 2 < sp.time.size() && sp.time[seg + 1] <= t) ++seg;
            const double v0 = sp.speed[seg];
            const double v1 = sp.speed[seg + 1];
            const double a = (v1 * v1 - v0 * v0) / (2.0 * sp.step);
            const double tau = std::min(t - sp.time[seg], sp.time[seg + 1] - sp.time[seg]);
            q.speed = std::max(0.0, v0 + a * tau);
            q.distance = std::min(
                total, static_cast<double>(seg) * sp.step + v0 * tau + 0.5 * a * tau * tau);
        }
        const TrajectoryPoint p = center.at(center.wrap_s(s_start + q.distance));
        q.course = p.yaw;
        q.curvature = p.curvature;
        const double lateral_force =
            vehicle.mass * q.speed * q.speed * q.curvature * vehicle.lf / l;
        const double rear_slip =
            rear_tire.slip_angle(normal_loads(vehicle, q.speed, 0.0).rear, lateral_force);
        q.sideslip = std::atan(vehicle.lr * q.curvature + std::tan(rear_slip));
        q.yaw = wrap_angle(q.course - q.sideslip);
        if (k == 0) {
            q.position = track.start_pose.position();
        } else {
            const Kinematics& prev = kin.back();
            q.position = prev.position +
                         0.5 * kTick *
                             (prev.speed * Vec2(std::cos(prev.course), std::sin(prev.course)) +
                              q.speed * Vec2(std::cos(q.course), std::sin(q.course)));
        }
        kin.push_back(q);
    }

    SyntheticRun run;
    run.truth.resize(ticks);
    auto central = [&](std::size_t k, auto&& value) {
        const std::size_t a = k == 0 ? 0 : k - 1;
        const std::size_t b = std::min(k + 1, ticks - 1);
        return (value(b) - value(a)) / (static_cast<double>(b - a) * kTick);
    };
    for (std::size_t k = 0; k < ticks; ++k) {
        const Kinematics& q = kin[k];
        TruthSample& truth = run.truth[k];
        truth.stamp = static_cast<double>(k) * kTick;
        truth.pose = {q.position.x(), q.position.y(), q.yaw};
        truth.vx = q.speed * std::cos(q.sideslip);
        truth.vy = q.speed * std::sin(q.sideslip);
        const std::size_t a = k == 0 ? 0 : k - 1;
        const std::size_t b = std::min(k + 1, ticks - 1);
        truth.yaw_rate = angle_diff(kin[b].yaw, kin[a].yaw) / (static_cast<double>(b - a) * kTick);
        truth.distance = q.distance;
        truth.lap = q.distance < staging + length
                        ? 1
                        : 2 + static_cast<int>((q.distance - staging - length) / length);
        truth.lap = std::min(truth.lap, profile.laps);
        if (q.distance >= staging + length &&
            run.lap1_end == std::numeric_limits<Time>::infinity()) {
            run.lap1_end = truth.stamp;
        }
    }
    for (std::size_t k = 0; k < ticks; ++k) {
        TruthSample& truth = run.truth[k];
        const double vx_dot = central(k, [&](std::size_t i) { return run.truth[i].vx; });
        const double vy_dot = central(k, [&](std::size_t i) { return run.truth[i].vy; });
        truth.ax = vx_dot - truth.yaw_rate * truth.vy;
        truth.ay = vy_dot + truth.yaw_rate * truth.vx;
    }
    for (std::size_t k = 0; k < ticks; ++k) {
        TruthSample& truth = run.truth[k];
        const double yaw_accel = central(k, [&](std::size_t i) { return run.truth[i].yaw_rate; });
        if (truth.vx < 0.1) {
            truth.steer = std::atan(l * kin[k].curvature);
            continue;
        }
        const double front_force =
            (vehicle.mass * truth.ay * vehicle.lr + vehicle.yaw_inertia * yaw_accel) / l;
        const Loads loads = normal_loads(vehicle, truth.vx, truth.ax);
        const double front_slip = front_tire.slip_angle(loads.front, front_force);
        truth.steer = std::atan2(truth.vy + vehicle.lf * truth.yaw_rate, truth.vx) - front_slip;
    }

    // Sensors, each on its own random stream.
    const Rng base(seed);
    Rng imu_rng = base.split(1);
    Rng wheel_rng = base.split(2);
    Rng steer_rng = base.split(3);
    Rng cone_rng = base.split(4);
    Rng bias_rng = base.split(5);
    run.mappable_since.assign(track.cones.size(), std::numeric_limits<Time>::infinity());
    std::vector<std::uint32_t> detections(track.cones.size(), 0);
    struct Pending {
        std::int64_t delivery_tick;
        int kind;
        EstimatorMessage message;
    };
    std::vector<Pending> pending;
    const auto latency_ticks = static_cast<std::int64_t>(std::llround(sensors.latency / kTick));
    double bias = sensors.gyro_bias;
    for (std::size_t k = 1; k < ticks; ++k) {
        const auto tick = static_cast<std::int64_t>(k);
        TruthSample& truth = run.truth[k];
        if (tick % kImuTicks == 0) {
            bias += sensors.gyro_bias_walk * std::sqrt(kImuTicks * kTick) * bias_rng.normal();
            ImuSample imu;
            imu.stamp = truth.stamp;
            imu.ax = truth.ax + sensors.accel_noise * imu_rng.normal();
            imu.ay = truth.ay + sensors.accel_noise * imu_rng.normal();
            imu.yaw_rate = truth.yaw_rate + bias + sensors.gyro_noise * imu_rng.normal();
            pending.push_back({tick, 0, imu});
        }
        truth.gyro_bias = bias;
        if (tick % kWheelTicks == 0) {
            const double speed = std::max(truth.vx, 0.0);
            const double resistance =
                speed > 0.5
                    ? vehicle.drag(speed) + vehicle.rolling_resistance * vehicle.mass * kGravity
                    : 0.0;
            const double force = vehicle.mass * truth.ax + resistance;
            const double split =
                force >= 0.0 ? vehicle.drive_split_front : vehicle.brake_split_front;
            const Loads loads = normal_loads(vehicle, speed, truth.ax);
            auto slip = [&](double axle_force, double fz, double mu) {
                const double mu_x = axle_force / fz;
                const double use = std::min(0.95, std::abs(mu_x) / mu);
                return mu_x / sensors.slip_stiffness / (1.0 - use * use);
            };
            const double front_along =
                truth.vx * std::cos(truth.steer) +
                (truth.vy + vehicle.lf * truth.yaw_rate) * std::sin(truth.steer);
            WheelSpeeds wheels;
            wheels.stamp = truth.stamp;
            wheels.front =
                front_along * (1.0 + slip(force * split, loads.front, vehicle.front_tire.mu)) +
                sensors.wheel_noise * wheel_rng.normal();
            wheels.rear =
                truth.vx * (1.0 + slip(force * (1.0 - split), loads.rear, vehicle.rear_tire.mu)) +
                sensors.wheel_noise * wheel_rng.normal();
            pending.push_back({tick, 1, wheels});
            SteeringSample steer;
            steer.stamp = truth.stamp;
            steer.angle = truth.steer + sensors.steering_noise * steer_rng.normal();
            pending.push_back({tick, 2, steer});
        }
        if (tick % kScanTicks == 0) {
            ConeObservations cones =
                observe_cones(track, truth.pose, truth.stamp, sensors, cone_rng);
            note_detections(cones, detections, run.mappable_since);
            pending.push_back({tick + latency_ticks, 3, std::move(cones)});
        }
    }
    std::stable_sort(pending.begin(), pending.end(), [](const Pending& a, const Pending& b) {
        return a.delivery_tick < b.delivery_tick ||
               (a.delivery_tick == b.delivery_tick && a.kind < b.kind);
    });
    run.messages.reserve(pending.size());
    for (Pending& p : pending) {
        run.messages.push_back(
            {static_cast<double>(p.delivery_tick) * kTick, std::move(p.message)});
    }
    return run;
}

}  // namespace apex::estimation::eval
