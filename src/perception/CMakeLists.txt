# The perception module (docs/design.md, section 5.2; docs/perception.md).
apex_module(perception
    SOURCES
        clustering.cpp
        ground.cpp
    PUBLIC_DEPS apex::core)
