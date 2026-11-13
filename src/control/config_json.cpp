#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "apex/control/controller.hpp"
#include "apex/core/json.hpp"

namespace apex::control {

using nlohmann::json;

std::string_view to_string(ControllerKind kind) {
    switch (kind) {
        case ControllerKind::PurePursuit:
            return "pure_pursuit";
        case ControllerKind::Stanley:
            return "stanley";
        case ControllerKind::Mpc:
            return "mpc";
    }
    return "unknown";
}

void to_json(json& j, ControllerKind kind) {
    j = std::string(to_string(kind));
}

void from_json(const json& j, ControllerKind& kind) {
    const std::string name = j.get<std::string>();
    for (const ControllerKind candidate :
         {ControllerKind::PurePursuit, ControllerKind::Stanley, ControllerKind::Mpc}) {
        if (name == to_string(candidate)) {
            kind = candidate;
            return;
        }
    }
    throw std::invalid_argument("unknown controller kind '" + name + "'");
}

// One list of fields per config drives both directions, so they cannot drift apart (the
// pattern of core's VehicleParams).
#define APEX_CONFIG_JSON(Type, context, FIELDS)                     \
    void to_json(json& j, const Type& c) {                          \
        j = json::object();                                         \
        FIELDS(APEX_WRITE_FIELD)                                    \
    }                                                               \
    void from_json(const json& j, Type& c) {                        \
        reject_unknown_keys(j, {FIELDS(APEX_FIELD_NAME)}, context); \
        FIELDS(APEX_READ_FIELD)                                     \
    }
#define APEX_WRITE_FIELD(name) j[#name] = c.name;
#define APEX_FIELD_NAME(name) #name,
#define APEX_READ_FIELD(name) read_optional(j, #name, c.name);

// clang-format off
#define APEX_SPEED_LOOP_FIELDS(X) \
    X(kp) X(ki) X(integral_limit) X(preview_time) X(standstill_speed) X(hold_brake)
#define APEX_PURE_PURSUIT_FIELDS(X) \
    X(lookahead_min) X(lookahead_time) X(lookahead_max) X(understeer_compensation) \
    X(delay_compensation) X(speed)
#define APEX_STANLEY_FIELDS(X) \
    X(gain) X(softening_speed) X(yaw_damping) X(slip_feedforward) X(delay_compensation) X(speed)
#define APEX_MPC_FIELDS(X) \
    X(horizon) X(step) X(weight_lateral) X(weight_course) X(weight_speed) X(weight_steer_rate) \
    X(weight_accel) X(terminal_scale) X(lateral_margin) X(slack_linear) X(slack_quadratic) \
    X(bound_open_references) X(grip_fraction) X(compensate_dead_time) X(compensate_lag) X(kinematic_below) X(dynamic_above) \
    X(sqp_iterations) X(initial_sqp_iterations) X(max_qp_iterations) X(qp_tolerance) \
    X(fallback_plan_age) X(fallback_decel)
#define APEX_CONTROLLER_FIELDS(X) X(kind) X(pure_pursuit) X(stanley) X(mpc)
// clang-format on

APEX_CONFIG_JSON(SpeedLoopConfig, "speed loop", APEX_SPEED_LOOP_FIELDS)
APEX_CONFIG_JSON(PurePursuitConfig, "pure pursuit", APEX_PURE_PURSUIT_FIELDS)
APEX_CONFIG_JSON(StanleyConfig, "stanley", APEX_STANLEY_FIELDS)
APEX_CONFIG_JSON(MpcConfig, "mpc", APEX_MPC_FIELDS)
APEX_CONFIG_JSON(ControllerConfig, "controller", APEX_CONTROLLER_FIELDS)

#undef APEX_CONFIG_JSON
#undef APEX_WRITE_FIELD
#undef APEX_FIELD_NAME
#undef APEX_READ_FIELD
#undef APEX_SPEED_LOOP_FIELDS
#undef APEX_PURE_PURSUIT_FIELDS
#undef APEX_STANLEY_FIELDS
#undef APEX_MPC_FIELDS
#undef APEX_CONTROLLER_FIELDS

}  // namespace apex::control
