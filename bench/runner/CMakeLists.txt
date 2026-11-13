# Closed-loop throughput of the full stack and the sweep pool's scaling (docs/runner.md).
add_executable(apex_runner_bench runner_bench.cpp)
target_link_libraries(apex_runner_bench PRIVATE apex::runner apex_warnings)
