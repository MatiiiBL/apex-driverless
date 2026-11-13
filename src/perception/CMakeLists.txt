# The perception module (docs/design.md, section 5.2; docs/perception.md).
apex_module(perception
    SOURCES
        assignment.cpp
        camera_fusion.cpp
        clustering.cpp
        cone_model.cpp
        ground.cpp
    PUBLIC_DEPS apex::core)
