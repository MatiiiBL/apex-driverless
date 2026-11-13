#include "apex/control/controller.hpp"

#include <stdexcept>

#include "pure_pursuit.hpp"
#include "stanley.hpp"

namespace apex::control {

std::unique_ptr<Controller> make_controller(const ControllerConfig& config,
                                            const VehicleParams& nominal) {
    switch (config.kind) {
        case ControllerKind::PurePursuit:
            return std::make_unique<PurePursuitController>(config.pure_pursuit, nominal);
        case ControllerKind::Stanley:
            return std::make_unique<StanleyController>(config.stanley, nominal);
        case ControllerKind::Mpc:
            break;
    }
    throw std::invalid_argument("make_controller: unsupported controller kind");
}

}  // namespace apex::control
