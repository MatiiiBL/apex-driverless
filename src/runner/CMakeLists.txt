# Closed-loop runs: the simulator and the stack together, scoring and experiment sweeps.
apex_module(runner
    SOURCES
    PUBLIC_DEPS apex::core apex::sim apex::stack)
