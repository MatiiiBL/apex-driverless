# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(sim
    SOURCES
        track_gen_test.cpp
        vehicle_model_test.cpp
    DEPS apex::sim)
