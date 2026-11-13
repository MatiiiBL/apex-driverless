# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(perception
    SOURCES
        assignment_test.cpp
        camera_fusion_test.cpp
        clustering_test.cpp
        cone_model_test.cpp
        ground_test.cpp
        perception_test.cpp
        scene.cpp
    DEPS apex::perception)
