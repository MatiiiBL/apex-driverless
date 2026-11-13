# The estimation module (docs/design.md, section 5.3; docs/estimation.md).
apex_module(estimation
    SOURCES
        chi_square.cpp
        lateral_model.cpp
        velocity_filter.cpp
    PUBLIC_DEPS apex::core)

add_subdirectory(eval)
