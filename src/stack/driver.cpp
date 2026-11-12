#include "apex/stack/driver.hpp"

namespace apex::stack {

std::string_view to_string(MissionState state) {
    switch (state) {
        case MissionState::Staged:
            return "staged";
        case MissionState::Exploring:
            return "exploring";
        case MissionState::MapFinalized:
            return "map_finalized";
        case MissionState::Racing:
            return "racing";
        case MissionState::Stopping:
            return "stopping";
        case MissionState::Finished:
            return "finished";
        case MissionState::Failed:
            return "failed";
    }
    return "unknown";
}

double AssociationCounts::precision() const {
    return matched == 0 ? 0.0 : static_cast<double>(correct) / static_cast<double>(matched);
}

double AssociationCounts::recall() const {
    return matchable == 0 ? 0.0 : static_cast<double>(correct) / static_cast<double>(matchable);
}

}  // namespace apex::stack
