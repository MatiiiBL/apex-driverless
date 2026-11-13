#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/core/track.hpp"
#include "apex/core/vehicle.hpp"
#include "apex/stack/driver.hpp"

namespace apex::runner::test_drivers {

// What the driver does wrong, to trigger each way a run can end.
enum class Fault : std::uint8_t {
    None,
    Stall,        // never drives off
    Veer,         // steers full left from fault_time on
    Fail,         // reports the mission Failed at fault_time
    NonFinite,    // returns a NaN command at fault_time
    FinishEarly,  // reports Finished at fault_time, laps or not
    NeverStop,    // keeps racing after the last lap
};

struct CenterlineDriverOptions {
    int laps = 1;
    double speed = 8.0;           // m/s
    double lookahead = 4.0;       // m, pure pursuit
    double start_time = 0.3;      // s staged before driving off
    double raceline_delay = 0.3;  // s in MapFinalized after lap 1, as the stack would
    Fault fault = Fault::None;
    double fault_time = 2.0;  // s
};

// One message as the driver received it, for the latency tests.
struct Delivery {
    std::string kind;  // "imu", "wheels", "steering", "camera", "lidar", "oracle_observations"
    Time stamp = 0.0;
    Time arrival = 0.0;     // the simulated time it was handed over
    std::size_t cones = 0;  // LiDAR points or observations
};

// A stand-in for the car's software that cheats: it is told the track and gets the true state
// every tick (the perfect-state oracle), then follows the true centerline with pure pursuit and
// a proportional speed loop. Its mission mimics the stack's (staged, exploring lap 1,
// map finalized, racing, stopping, finished) with laps counted from the true centerline, and
// its map is the true cones. It lets the runner's tests check timing, scoring, latency and the
// DNF rules without perception, estimation or planning.
class CenterlineDriver final : public stack::Driver {
public:
    CenterlineDriver(const Track& track, const VehicleParams& vehicle,
                     const CenterlineDriverOptions& options);

    void on_sensor(const SensorMessage& message) override;
    void on_oracle_observations(const ConeObservations& observations) override;
    void on_oracle_state(const StateEstimate& truth) override;
    [[nodiscard]] ControlCommand control(Time now) override;

    [[nodiscard]] stack::MissionState mission_state() const override { return state_; }
    [[nodiscard]] const std::vector<stack::MissionTransition>& transitions() const override {
        return transitions_;
    }
    [[nodiscard]] StateEstimate estimate() const override { return truth_; }
    [[nodiscard]] const ConeMap* map() const override { return &map_; }
    [[nodiscard]] const Trajectory* reference() const override;
    [[nodiscard]] stack::AssociationCounts association_counts() const override { return {}; }
    [[nodiscard]] nlohmann::json compute_summary() const override {
        return nlohmann::json::object();
    }

    [[nodiscard]] const std::vector<Delivery>& deliveries() const { return deliveries_; }
    [[nodiscard]] int laps_completed() const { return crossings_ > 0 ? crossings_ - 1 : 0; }

private:
    void transition(stack::MissionState to, Time now, std::string reason);
    void count_laps();

    const Track& track_;
    VehicleParams vehicle_;
    CenterlineDriverOptions options_;
    ConeMap map_;
    StateEstimate truth_;
    stack::MissionState state_ = stack::MissionState::Staged;
    std::vector<stack::MissionTransition> transitions_;
    std::vector<Delivery> deliveries_;
    Time state_since_ = 0.0;
    std::optional<double> previous_s_;
    double s_ = 0.0;
    int crossings_ = 0;
};

}  // namespace apex::runner::test_drivers
