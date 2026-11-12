# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(planning
    SOURCES
        config_json_test.cpp
        delaunay_test.cpp
        predicates_test.cpp
        speed_profile_test.cpp
    DEPS apex::planning)
