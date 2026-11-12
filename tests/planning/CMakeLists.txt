# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(planning
    SOURCES
    DEPS apex::planning)
