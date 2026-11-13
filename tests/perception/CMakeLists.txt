# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(perception
    SOURCES
        ground_test.cpp
        scene.cpp
    DEPS apex::perception)
