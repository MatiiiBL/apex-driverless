#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json_fwd.hpp>

#include "apex/core/messages.hpp"
#include "apex/core/trajectory.hpp"
#include "apex/core/types.hpp"
#include "apex/core/vehicle.hpp"

// The boundary between the car's software and the closed loop that runs it. It lives in the
// stack module, not in the runner, so that the stack can implement it without depending on the
// simulator (docs/design.md, section 2): the runner steps any Driver, the Stack is the real one,
// and the runner's tests use simpler ones.
namespace apex::stack {

// The trackdrive mission of docs/design.md, section 1 (transitions in section 5.6):
//   Staged -> Exploring -> MapFinalized -> Racing -> Stopping -> Finished, or Failed.
// MapFinalized covers the time between closing lap 1 and switching to the raceline, during which
// the car keeps following the exploration planner on the frozen map.
enum class MissionState : std::uint8_t {
    Staged,
    Exploring,
    MapFinalized,
    Racing,
    Stopping,
    Finished,
    Failed,
};
inline constexpr std::size_t kMissionStateCount = 7;

[[nodiscard]] std::string_view to_string(MissionState state);
[[nodiscard]] constexpr std::size_t index_of(MissionState state) {
    return static_cast<std::size_t>(state);
}
// Finished or Failed: the driver will not move the car again.
[[nodiscard]] constexpr bool is_terminal(MissionState state) {
    return state == MissionState::Finished || state == MissionState::Failed;
}

struct MissionTransition {
    Time time = 0.0;
    MissionState from = MissionState::Staged;
    MissionState to = MissionState::Staged;
    std::string reason;
};

// Ground truth the runner substitutes for part of the car's software in ablation runs
// (docs/design.md, section 8). A driver is built knowing which oracles are on; the runner then
// feeds them through Driver::on_oracle_observations / on_oracle_state and stops delivering the
// sensor messages they replace. The car's software never reaches into the simulator for them.
struct Oracles {
    // Cone observations generated from the true cones within sensor range, with nominal noise,
    // instead of LiDAR scans and camera frames run through perception.
    bool perception = false;
    // The true state instead of the estimator's estimate (the map is then built with known
    // poses).
    bool state = false;
};

// Evaluation counters for data association, from the truth ids. They are counted, never used
// to decide anything (docs/design.md, section 4.3).
struct AssociationCounts {
    std::uint64_t observations = 0;  // observations with a true cone behind them
    std::uint64_t matched = 0;       // observations matched to a confirmed landmark
    std::uint64_t correct = 0;       // matched to the landmark that carries their truth id
    std::uint64_t matchable = 0;     // observations of a cone that already had a landmark

    // correct / matched and correct / matchable; 0 when the denominator is.
    [[nodiscard]] double precision() const;
    [[nodiscard]] double recall() const;
};

// The car's software, as the closed loop sees it.
//
// Inputs arrive in simulated-time order: sensor messages once their processing latency has
// passed (docs/design.md, section 4.4), then, every control period, a call to control(). The
// remaining accessors are read-only views for the evaluation; nothing read through them is fed
// back into the car.
class Driver {
public:
    virtual ~Driver() = default;

    virtual void on_sensor(const SensorMessage& message) = 0;
    // Oracle inputs, called only for the oracles the driver was built with. Observations come
    // at the LiDAR's rate and latency; the true state comes on every tick, stamped with it.
    virtual void on_oracle_observations(const ConeObservations& observations) = 0;
    virtual void on_oracle_state(const StateEstimate& truth) = 0;

    // The command for the next control period. Never non-finite.
    [[nodiscard]] virtual ControlCommand control(Time now) = 0;

    [[nodiscard]] virtual MissionState mission_state() const = 0;
    [[nodiscard]] virtual const std::vector<MissionTransition>& transitions() const = 0;

    // ---- Views for the evaluation ----

    // The state the driver acts on (the estimator's, or the oracle's).
    [[nodiscard]] virtual StateEstimate estimate() const = 0;
    // The cone map, or nullptr for a driver without one.
    [[nodiscard]] virtual const ConeMap* map() const = 0;
    // The path the controller is tracking, or nullptr before there is one.
    [[nodiscard]] virtual const Trajectory* reference() const = 0;
    [[nodiscard]] virtual AssociationCounts association_counts() const = 0;
    // Wall-clock compute time per module: {"<module>": LatencyStats::summary(), ...}.
    [[nodiscard]] virtual nlohmann::json compute_summary() const = 0;
};

}  // namespace apex::stack
