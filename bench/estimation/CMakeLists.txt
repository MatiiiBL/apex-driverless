# Evaluation runs and latency benchmark of the estimation module (docs/estimation.md).
add_executable(apex_estimation_bench estimation_bench.cpp)
target_link_libraries(apex_estimation_bench PRIVATE apex::estimation_world apex_warnings)
