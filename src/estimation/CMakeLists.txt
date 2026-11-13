# The estimation module (docs/design.md, section 5.3; docs/estimation.md).
apex_module(estimation
    SOURCES
        chi_square.cpp
    PUBLIC_DEPS apex::core)

add_subdirectory(eval)
