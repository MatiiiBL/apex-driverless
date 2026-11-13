# The estimation module (docs/design.md, section 5.3; docs/estimation.md).
apex_module(estimation
    SOURCES
        association.cpp
        chi_square.cpp
        dead_reckoning.cpp
        ekf_slam.cpp
        estimator.cpp
        landmark_records.cpp
        observation.cpp
        lateral_model.cpp
        odometry.cpp
        slam_config.cpp
        velocity_filter.cpp
    PUBLIC_DEPS apex::core)

add_subdirectory(eval)
