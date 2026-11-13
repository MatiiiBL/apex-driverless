# Closed-loop runs: the simulator and the stack together, scoring and experiment sweeps.
find_package(Threads REQUIRED)
apex_module(runner
    SOURCES
        config.cpp
        csv_log.cpp
        metrics.cpp
        oracle.cpp
        replay.cpp
        result.cpp
        run.cpp
        run_stack.cpp
        summary.cpp
        sweep.cpp
        world_setup.cpp
    PUBLIC_DEPS apex::core apex::sim apex::stack
    PRIVATE_DEPS Threads::Threads)
