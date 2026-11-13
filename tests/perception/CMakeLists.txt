# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(perception
    SOURCES
        clustering_test.cpp
        ground_test.cpp
        scene.cpp
    DEPS apex::perception)
