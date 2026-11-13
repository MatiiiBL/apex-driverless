# The control module (docs/design.md, section 5.5; docs/control.md).
apex_module(control
    SOURCES
        common.cpp
        config_json.cpp
        controller.cpp
        path_follower.cpp
        pure_pursuit.cpp
        reference.cpp
        speed_controller.cpp
        stanley.cpp
        vehicle_model.cpp
    PUBLIC_DEPS apex::core)
