#include "apex/estimation/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "apex/core/json.hpp"
#include "apex/core/log.hpp"
#include "apex/estimation/dead_reckoning.hpp"
#include "apex/estimation/ekf_slam.hpp"
#include "apex/estimation/observation.hpp"
#include "apex/estimation/odometry.hpp"

#include "landmark_records.hpp"

namespace apex::estimation {

void to_json(nlohmann::json& j, const EstimatorConfig& config) {
    j = {{"velocity", config.velocity}, {"slam", config.slam}, {"association", config.association}};
}

void from_json(const nlohmann::json& j, EstimatorConfig& config) {
    reject_unknown_keys(j, {"velocity", "slam", "association"}, "estimator");
    read_optional(j, "velocity", config.velocity);
    read_optional(j, "slam", config.slam);
    read_optional(j, "association", config.association);
}

namespace {

void require_positive(double value, const char* name) {
    if (!(value > 0.0) || !std::isfinite(value)) {
        throw std::invalid_argument(std::string("estimator: slam.") + name + " must be positive");
    }
}

const EstimatorConfig& validated(const EstimatorConfig& config) {
    const SlamConfig& c = config.slam;
    require_positive(c.velocity_correlation_time, "velocity_correlation_time");
    require_positive(c.position_noise_density, "position_noise_density");
    require_positive(c.yaw_noise_density, "yaw_noise_density");
    require_positive(c.initial_position_std, "initial_position_std");
    require_positive(c.initial_yaw_std, "initial_yaw_std");
    require_positive(c.observation_noise_scale, "observation_noise_scale");
    require_positive(c.min_observation_std, "min_observation_std");
    require_positive(c.max_observation_range, "max_observation_range");
    require_positive(c.max_new_landmark_range, "max_new_landmark_range");
    require_positive(c.tentative_timeout, "tentative_timeout");
    require_positive(c.history_duration, "history_duration");
    if (c.min_landmark_separation < 0.0) {
        throw std::invalid_argument("estimator: slam.min_landmark_separation must be >= 0");
    }
    if (c.confirm_detections < 1) {
        throw std::invalid_argument("estimator: slam.confirm_detections must be at least 1");
    }
    return config;
}

Mat3 start_covariance(const SlamConfig& c) {
    Mat3 p = Mat3::Zero();
    p.diagonal() << c.initial_position_std * c.initial_position_std,
        c.initial_position_std * c.initial_position_std, c.initial_yaw_std * c.initial_yaw_std;
    return p;
}

}  // namespace

struct StateEstimator::Impl {
    Impl(const EstimatorConfig& cfg, const VehicleParams& nominal, const Pose2& start)
        : config(validated(cfg)),
          motion(cfg.velocity, cfg.slam, nominal),
          slam(start, start_covariance(cfg.slam)),
          associator(cfg.association) {}

    void on_imu(const ImuSample& sample) {
        const Stopwatch watch;
        if (!motion.on_imu(sample)) {
            ++stats.invalid_inputs;
            return;
        }
        // Motion older than any observation can be will not be revisited: commit it.
        OdometryBuffer& odometry = motion.odometry();
        const Time horizon = odometry.end() - config.slam.history_duration;
        if (odometry.start() < horizon) slam.predict(odometry.take_until(horizon));
        stats.predict_time.add(watch.elapsed_seconds());
    }

    void on_wheels(const WheelSpeeds& sample) {
        if (!motion.on_wheels(sample)) ++stats.invalid_inputs;
        stats.wheel_outliers = motion.velocity().wheel_outliers();
    }

    void on_steering(const SteeringSample& sample) {
        if (!motion.on_steering(sample)) ++stats.invalid_inputs;
    }

    // Late observations: the filter state lags the odometry. It is brought to the scan's own
    // stamp, the observations are fused there, and the motion after the stamp stays pending in
    // the odometry, composed onto the pose only for estimate(). Since the velocity filter does
    // not depend on the SLAM state, this is the same computation as fusing on time. A stamp
    // beyond the newest IMU sample is reached at the current velocity.
    void advance_to(Time stamp) {
        motion.extrapolate_to(stamp);
        slam.predict(motion.odometry().take_until(stamp));
    }

    void on_cones(const ConeObservations& observations) {
        const Stopwatch watch;
        const std::size_t n = observations.cones.size();
        last_associations.assign(n, -1);
        stats.observations += n;
        if (!std::isfinite(observations.stamp)) {
            ++stats.invalid_inputs;
            stats.rejected += n;
            return;
        }
        motion.start(observations.stamp);
        if (observations.stamp < motion.odometry().start()) {
            // Older than the committed filter state: it cannot be fused at its stamp.
            stats.late_dropped += n;
            stats.rejected += n;
            stats.update_time.add(watch.elapsed_seconds());
            return;
        }
        advance_to(observations.stamp);

        std::vector<ObservationModel> models;
        std::vector<std::size_t> source;  // models[k] is observations.cones[source[k]]
        models.reserve(n);
        source.reserve(n);
        for (std::size_t i = 0; i < n; ++i) {
            const PreparedObservation prepared =
                prepare_observation(observations.cones[i], config.slam);
            if (prepared.status != ObservationStatus::Usable) {
                if (prepared.status == ObservationStatus::Invalid) ++stats.invalid_inputs;
                ++stats.rejected;
                continue;
            }
            models.push_back(prepared.model);
            source.push_back(i);
        }

        const AssociationResult association = associator.associate(slam, models);
        if (association.ambiguous) ++stats.ambiguous_scans;
        if (association.budget_exhausted) ++stats.search_exhausted;

        std::vector<Pairing> pairings;
        std::vector<std::size_t> paired;  // model index of each pairing
        for (std::size_t k = 0; k < models.size(); ++k) {
            if (association.landmark[k] < 0) continue;
            pairings.push_back(
                {static_cast<std::size_t>(association.landmark[k]), models[k].z, models[k].R});
            paired.push_back(k);
        }
        bool updated = false;
        if (!pairings.empty()) {
            updated = slam.update(pairings, mapping);
            if (!updated) ++stats.failed_updates;
        }

        bool map_changed = mapping && updated;
        std::vector<char> handled(models.size(), 0);
        if (updated) {
            for (std::size_t p = 0; p < pairings.size(); ++p) {
                const std::size_t k = paired[p];
                handled[k] = 1;
                LandmarkRecord& record = records[pairings[p].landmark];
                const bool was_confirmed = record.confirmed;
                if (was_confirmed) {
                    ++stats.associated;
                    last_associations[source[k]] = record.id;
                } else {
                    ++stats.tentative_matches;
                    ++stats.rejected;
                }
                if (!mapping) continue;  // the map is frozen: no votes, no counts
                record.observe(observations.cones[source[k]], observations.stamp);
                if (!was_confirmed && record.detections >= config.slam.confirm_detections) {
                    record.confirmed = true;
                    record.id = next_id++;
                    ++stats.new_landmarks;
                }
            }
        }

        // The rest are gated out, ambiguous (between the gate and the new-landmark gate), or,
        // when they start a landmark, pending its confirmation.
        for (std::size_t k = 0; k < models.size(); ++k) {
            if (handled[k]) continue;
            ++stats.rejected;
            if (mapping && association.min_distance2[k] > associator.new_landmark_gate() &&
                start_tentative(models[k], observations.cones[source[k]], observations.stamp)) {
                map_changed = map_changed || records.back().confirmed;
            }
        }

        if (mapping) drop_stale_tentatives(observations.stamp);
        if (map_changed) rebuild_map();
        if (!slam.finite() && healthy) {
            healthy = false;
            log(LogLevel::Error, "estimator: non-finite filter state at t = {:.3f} s",
                observations.stamp);
        }
        stats.update_time.add(watch.elapsed_seconds());
    }

    // A new, unconfirmed landmark from an observation that matched nothing, if it is near
    // enough and not on top of an existing landmark.
    bool start_tentative(const ObservationModel& model, const ConeObservation& cone, Time stamp) {
        if (model.z.norm() > config.slam.max_new_landmark_range) return false;
        if (slam.landmark_count() >= config.slam.max_landmarks) return false;
        const Vec2 world = slam.pose().to_parent(model.z);
        const double separation = config.slam.min_landmark_separation;
        for (std::size_t j = 0; j < slam.landmark_count(); ++j) {
            if ((slam.landmark(j) - world).squaredNorm() < separation * separation) return false;
        }
        slam.add_landmark(model.z, model.R);
        LandmarkRecord record;
        record.observe(cone, stamp);
        if (record.detections >= config.slam.confirm_detections) {
            record.confirmed = true;
            record.id = next_id++;
            ++stats.new_landmarks;
        }
        records.push_back(record);
        ++stats.tentatives_started;
        return true;
    }

    void drop_stale_tentatives(Time stamp) {
        for (std::size_t i = records.size(); i-- > 0;) {
            if (records[i].confirmed ||
                stamp - records[i].last_seen <= config.slam.tentative_timeout) {
                continue;
            }
            remove_landmark(i);
            ++stats.tentatives_dropped;
        }
    }

    void remove_landmark(std::size_t i) {
        slam.remove_landmark(i);
        records[i] = records.back();
        records.pop_back();
    }

    void rebuild_map() {
        map.cones.clear();
        for (std::size_t i = 0; i < records.size(); ++i) {
            const LandmarkRecord& record = records[i];
            if (!record.confirmed) continue;
            MapCone cone;
            cone.id = record.id;
            cone.position = slam.landmark(i);
            cone.covariance = slam.landmark_covariance(i);
            cone.color = record.color();
            cone.color_votes = record.votes;
            cone.observations = record.detections;
            cone.truth_id = record.truth.majority();
            map.cones.push_back(cone);
        }
        std::sort(map.cones.begin(), map.cones.end(),
                  [](const MapCone& a, const MapCone& b) { return a.id < b.id; });
        ++map.revision;
    }

    void set_mapping(bool enabled) {
        if (enabled == mapping) return;
        mapping = enabled;
        if (!enabled) {
            // The landmark set freezes as confirmed: unconfirmed ones go.
            for (std::size_t i = records.size(); i-- > 0;) {
                if (!records[i].confirmed) remove_landmark(i);
            }
        }
        log(LogLevel::Info, "estimator: mapping {} with {} landmarks", enabled ? "on" : "off",
            map.cones.size());
    }

    [[nodiscard]] StateEstimate estimate() const {
        // The filter's pose (at its own, older time) followed by the pending odometry: a world
        // pose composed with a relative motion, with the same first-order propagation.
        const RelativeMotion now =
            compose({slam.pose(), slam.pose_covariance()}, motion.odometry().pending());
        const VelocityFilter& velocity = motion.velocity();
        StateEstimate e;
        e.stamp = motion.odometry().end();
        e.pose = now.delta;
        e.pose_covariance = now.covariance;
        e.vx = velocity.vx();
        e.vy = velocity.vy();
        e.yaw_rate = velocity.yaw_rate();
        e.valid = motion.started() && healthy && std::isfinite(e.pose.x) &&
                  std::isfinite(e.pose.y) && std::isfinite(e.pose.yaw) &&
                  e.pose_covariance.allFinite() && std::isfinite(e.vx) && std::isfinite(e.vy) &&
                  std::isfinite(e.yaw_rate);
        return e;
    }

    EstimatorConfig config;
    DeadReckoning motion;
    EkfSlam slam;
    Associator associator;
    std::vector<LandmarkRecord> records;  // records[i] describes the filter's landmark i
    std::int32_t next_id = 0;
    ConeMap map;
    EstimatorStats stats;
    std::vector<std::int32_t> last_associations;
    bool mapping = true;
    bool healthy = true;
};

StateEstimator::StateEstimator(const EstimatorConfig& config, const VehicleParams& nominal,
                               const SensorRig& /*rig*/, const Pose2& start)
    : impl_(std::make_unique<Impl>(config, nominal, start)) {}

StateEstimator::~StateEstimator() = default;
StateEstimator::StateEstimator(StateEstimator&&) noexcept = default;
StateEstimator& StateEstimator::operator=(StateEstimator&&) noexcept = default;

void StateEstimator::on_imu(const ImuSample& sample) {
    impl_->on_imu(sample);
}

void StateEstimator::on_wheels(const WheelSpeeds& sample) {
    impl_->on_wheels(sample);
}

void StateEstimator::on_steering(const SteeringSample& sample) {
    impl_->on_steering(sample);
}

void StateEstimator::on_cones(const ConeObservations& observations) {
    impl_->on_cones(observations);
}

StateEstimate StateEstimator::estimate() const {
    return impl_->estimate();
}

const ConeMap& StateEstimator::map() const {
    return impl_->map;
}

void StateEstimator::set_mapping(bool enabled) {
    impl_->set_mapping(enabled);
}

bool StateEstimator::mapping() const {
    return impl_->mapping;
}

const EstimatorStats& StateEstimator::stats() const {
    return impl_->stats;
}

const std::vector<std::int32_t>& StateEstimator::last_associations() const {
    return impl_->last_associations;
}

}  // namespace apex::estimation
