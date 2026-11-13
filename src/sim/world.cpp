#include "apex/sim/world.hpp"

#include <cmath>
#include <format>
#include <stdexcept>
#include <utility>

#include "apex/core/rng.hpp"
#include "apex/sim/vehicle_model.hpp"

namespace apex::sim {

namespace {

// Rng streams split from the world seed, one per sensor, so that adding draws to one sensor
// never shifts another's numbers.
enum class Stream : std::uint64_t { Imu = 1, Wheels = 2, Steering = 3, Camera = 4, Lidar = 5 };

Rng stream(std::uint64_t seed, Stream which) {
    return Rng(seed).split(static_cast<std::uint64_t>(which));
}

// The sensor period in ticks; throws unless the rate divides the physics rate exactly.
std::int64_t period_ticks(double rate_hz, const char* sensor) {
    if (!(std::isfinite(rate_hz) && rate_hz > 0.0)) {
        throw std::invalid_argument(std::format("World: {} rate must be positive", sensor));
    }
    const double ticks = 1.0 / (rate_hz * World::kTickSeconds);
    const double whole = std::round(ticks);
    if (whole < 1.0 || std::abs(ticks - whole) > 1e-9 * ticks) {
        throw std::invalid_argument(std::format(
            "World: {} rate {} Hz does not divide the 1 kHz physics rate", sensor, rate_hz));
    }
    return static_cast<std::int64_t>(whole);
}

bool finite(const Vec3& v) {
    return v.allFinite();
}

void validate_rig(const SensorRig& rig) {
    const LidarGeometry& lidar = rig.lidar;
    bool ok = finite(lidar.mount) && !lidar.elevations.empty();
    for (const double elevation : lidar.elevations) {
        ok = ok && std::isfinite(elevation) && std::abs(elevation) < 1.5;
    }
    ok = ok && std::isfinite(lidar.azimuth_min) && std::isfinite(lidar.azimuth_max) &&
         lidar.azimuth_min < lidar.azimuth_max && std::isfinite(lidar.azimuth_step) &&
         lidar.azimuth_step > 0.0 && std::isfinite(lidar.min_range) && lidar.min_range >= 0.0 &&
         std::isfinite(lidar.max_range) && lidar.max_range > lidar.min_range;
    if (!ok) throw std::invalid_argument("World: invalid LiDAR geometry");
    const CameraGeometry& camera = rig.camera;
    if (!finite(camera.mount) || !(camera.horizontal_fov > 0.0 && camera.horizontal_fov <= 6.3) ||
        !(std::isfinite(camera.max_range) && camera.max_range > 0.0)) {
        throw std::invalid_argument("World: invalid camera geometry");
    }
}

}  // namespace

struct World::Impl {
    Impl(Track track_in, const WorldConfig& config_in)
        : track(std::move(track_in)),
          config(config_in),
          vehicle(config.vehicle, config.friction_scale),
          race(track, config.vehicle),
          imu_period(period_ticks(config.rig.imu_rate_hz, "IMU")),
          wheel_period(period_ticks(config.rig.wheel_rate_hz, "wheel speed")),
          steering_period(period_ticks(config.rig.steering_rate_hz, "steering")),
          camera_period(period_ticks(config.rig.camera.rate_hz, "camera")),
          lidar_period(period_ticks(config.rig.lidar.rate_hz, "LiDAR")),
          imu_rng(stream(config.seed, Stream::Imu)),
          wheel_rng(stream(config.seed, Stream::Wheels)),
          steering_rng(stream(config.seed, Stream::Steering)),
          camera_rng(stream(config.seed, Stream::Camera)),
          lidar_rng(stream(config.seed, Stream::Lidar)) {
        validate_noise(config.noise);
        validate_rig(config.rig);
        for (std::size_t i = 0; i < track.cones.size(); ++i) {
            if (track.cones[i].id != static_cast<std::int32_t>(i)) {
                throw std::invalid_argument("World: cone ids must equal their index");
            }
        }
        imu_bias = draw_imu_bias(config.noise.imu, imu_rng);
        vehicle.reset(VehicleState{track.start_pose}, 0.0);
        race.update(vehicle.state(), 0.0);
    }

    Track track;
    WorldConfig config;
    VehicleModel vehicle;
    RaceMonitor race;  // refers to `track`, which never moves: Impl lives on the heap
    std::int64_t tick = 0;
    std::int64_t imu_period;
    std::int64_t wheel_period;
    std::int64_t steering_period;
    std::int64_t camera_period;
    std::int64_t lidar_period;
    Rng imu_rng;
    Rng wheel_rng;
    Rng steering_rng;
    Rng camera_rng;
    Rng lidar_rng;
    ImuBias imu_bias;
};

World::World(Track track, const WorldConfig& config)
    : impl_(std::make_unique<Impl>(std::move(track), config)) {}

World::~World() = default;
World::World(World&&) noexcept = default;
World& World::operator=(World&&) noexcept = default;

void World::command(const ControlCommand& command) {
    impl_->vehicle.command(command);
}

std::vector<SensorMessage> World::step() {
    Impl& w = *impl_;
    w.vehicle.step(kTickSeconds);
    ++w.tick;
    const Time now = time();
    const VehicleState& truth = w.vehicle.state();
    w.race.update(truth, now);

    std::vector<SensorMessage> messages;
    const SensorNoise& noise = w.config.noise;
    if (w.tick % w.imu_period == 0) {
        const double period = static_cast<double>(w.imu_period) * kTickSeconds;
        messages.emplace_back(simulate_imu(noise.imu, truth, now, period, w.imu_bias, w.imu_rng));
    }
    if (w.tick % w.wheel_period == 0) {
        messages.emplace_back(simulate_wheel_speeds(w.config.vehicle, noise.wheels, truth,
                                                    w.vehicle.tire_forces(), now, w.wheel_rng));
    }
    if (w.tick % w.steering_period == 0) {
        messages.emplace_back(simulate_steering(noise.steering, truth, now, w.steering_rng));
    }
    if (w.tick % w.camera_period == 0) {
        messages.emplace_back(simulate_camera(w.config.rig.camera, noise.camera, truth,
                                              w.track.cones, now, w.camera_rng));
    }
    if (w.tick % w.lidar_period == 0) {
        messages.emplace_back(simulate_lidar(w.config.rig.lidar, noise.lidar, truth, w.track.cones,
                                             now, w.lidar_rng));
    }
    return messages;
}

std::int64_t World::tick() const {
    return impl_->tick;
}
Time World::time() const {
    return static_cast<double>(impl_->tick) * kTickSeconds;
}
const VehicleState& World::truth() const {
    return impl_->vehicle.state();
}
const Track& World::track() const {
    return impl_->track;
}
const RaceMonitor& World::race() const {
    return impl_->race;
}
const WorldConfig& World::config() const {
    return impl_->config;
}

}  // namespace apex::sim
