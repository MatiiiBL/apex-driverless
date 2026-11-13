# Closed-loop runs: the simulator and the stack together, scoring and experiment sweeps.
apex_module(runner
    SOURCES
        config.cpp
        metrics.cpp
        oracle.cpp
        result.cpp
    PUBLIC_DEPS apex::core apex::sim apex::stack)
