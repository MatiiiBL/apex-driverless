# Tests may link apex::sim to generate realistic data, even for the car's modules.
apex_tests(runner
    SOURCES
        config_test.cpp
        delay_queue_test.cpp
        metrics_test.cpp
        oracle_test.cpp
        result_test.cpp
    DEPS apex::runner)
