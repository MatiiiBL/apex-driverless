# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(stack
    SOURCES
        braking_test.cpp
        known_pose_mapper_test.cpp
        lap_counter_test.cpp
        mission_test.cpp
        stack_config_test.cpp
    DEPS apex::stack)
