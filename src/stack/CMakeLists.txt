# The car's software assembled: perception, estimation, planning and control behind the mission
# state machine. It must not depend on apex::sim.
apex_module(stack
    SOURCES
    PUBLIC_DEPS apex::core apex::perception apex::estimation apex::planning apex::control)
