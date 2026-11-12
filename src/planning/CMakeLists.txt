# The planning module (docs/design.md, section 5.4; docs/planning.md).
apex_module(planning
    SOURCES
        config_json.cpp
        ggv.cpp
        speed_profile.cpp
    PUBLIC_DEPS apex::core)
