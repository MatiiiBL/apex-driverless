# The perception module (docs/design.md, section 5.2; docs/perception.md).
apex_module(perception
    SOURCES
        assignment.cpp
        calibration.cpp
        camera_fusion.cpp
        clustering.cpp
        cone_model.cpp
        config.cpp
        detector.cpp
        ground.cpp
        perception.cpp
    PUBLIC_DEPS apex::core)
