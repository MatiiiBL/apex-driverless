#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include <nlohmann/json.hpp>

#include "apex/stack/driver.hpp"

namespace apex::runner::test_drivers {

// Wraps a driver and relabels every truth id it is given (LiDAR points, camera detections,
// oracle observations) through a fixed random permutation of the cone ids; -1 stays -1. The car's
// software may pass truth ids along but never decide anything on them (docs/design.md, section
// 4.3), so the wrapped driver must command exactly what it would without the relabelling.
class ShuffledTruthIds final : public stack::Driver {
public:
    ShuffledTruthIds(stack::Driver& inner, std::size_t cone_count, std::uint64_t seed);

    void on_sensor(const SensorMessage& message) override;
    void on_oracle_observations(const ConeObservations& observations) override;
    void on_oracle_state(const StateEstimate& truth) override { inner_.on_oracle_state(truth); }
    [[nodiscard]] ControlCommand control(Time now) override { return inner_.control(now); }

    [[nodiscard]] stack::MissionState mission_state() const override {
        return inner_.mission_state();
    }
    [[nodiscard]] const std::vector<stack::MissionTransition>& transitions() const override {
        return inner_.transitions();
    }
    [[nodiscard]] StateEstimate estimate() const override { return inner_.estimate(); }
    [[nodiscard]] const ConeMap* map() const override { return inner_.map(); }
    [[nodiscard]] const Trajectory* reference() const override { return inner_.reference(); }
    [[nodiscard]] stack::AssociationCounts association_counts() const override {
        return inner_.association_counts();
    }
    [[nodiscard]] nlohmann::json compute_summary() const override {
        return inner_.compute_summary();
    }

    [[nodiscard]] std::int32_t relabel(std::int32_t id) const;

private:
    stack::Driver& inner_;
    std::vector<std::int32_t> permutation_;
};

}  // namespace apex::runner::test_drivers
