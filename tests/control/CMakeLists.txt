# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(control
    SOURCES
    DEPS apex::control)
