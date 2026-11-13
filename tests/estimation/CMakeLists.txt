# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(estimation
    SOURCES
        chi_square_test.cpp
        consistency_test.cpp
        lateral_model_test.cpp
        odometry_test.cpp
        synthetic_test.cpp
        velocity_filter_test.cpp
    DEPS apex::estimation_eval apex::sim)
