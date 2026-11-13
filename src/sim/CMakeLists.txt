# The sim module (docs/design.md, section 5.1): tracks, vehicle dynamics, sensors, the World.
apex_module(sim
    SOURCES
        reference_tracks.cpp
        track_gen.cpp
        track_layout.cpp
        track_rules.cpp
    PUBLIC_DEPS apex::core)
