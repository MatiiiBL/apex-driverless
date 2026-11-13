# The car's software assembled: perception, estimation, planning and control behind the mission
# state machine. It must not depend on apex::sim.
apex_module(stack
    SOURCES
        braking.cpp
        driver.cpp
        known_pose_mapper.cpp
        lap_counter.cpp
        mission.cpp
        stack.cpp
        stack_config.cpp
    PUBLIC_DEPS apex::core apex::perception apex::estimation apex::planning apex::control)
