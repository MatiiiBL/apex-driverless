apex_module(core
    SOURCES
        geometry.cpp
        json.cpp
        log.cpp
        rng.cpp
        spline.cpp
        timing.cpp
        track.cpp
        trajectory.cpp
        types.cpp
    PUBLIC_DEPS Eigen3::Eigen nlohmann_json::nlohmann_json)
