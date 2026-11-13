# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(sim
    SOURCES
        camera_test.cpp
        config_json_test.cpp
        lidar_test.cpp
        race_test.cpp
        track_gen_test.cpp
        vehicle_model_test.cpp
        vehicle_sensors_test.cpp
        world_test.cpp
    DEPS apex::sim)
