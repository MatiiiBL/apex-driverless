# The sim module (docs/design.md, section 5.1): tracks, vehicle dynamics, sensors, the World.
apex_module(sim
    SOURCES
        camera.cpp
        config_json.cpp
        lidar.cpp
        race.cpp
        reference_tracks.cpp
        sensor_noise.cpp
        track_gen.cpp
        track_layout.cpp
        track_rules.cpp
        vehicle_model.cpp
        vehicle_sensors.cpp
        world.cpp
    PUBLIC_DEPS apex::core)
